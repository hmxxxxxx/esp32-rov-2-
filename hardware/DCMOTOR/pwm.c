#include <stdio.h>
#include<math.h>
#include <inttypes.h> 
#include "driver/ledc.h"
#include "pwm.h"
#include "feedback.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 保护状态结构体
typedef struct {
    bool over_temp;
    bool over_current;
    uint32_t last_protect_time;
} pwm_protect_t;

static pwm_protect_t protect_status = {0};

/* ［配置区］根据硬件实际连接修改这些参数 */
#define TAG "PWM_CTRL"                   // 日志标签
#define PWM_FREQ_HZ        50            // PWM频率（Hz）
#define PWM_RESOLUTION     LEDC_TIMER_14_BIT // 分辨率
#define PWM_TIMER_MODE     LEDC_LOW_SPEED_MODE // 定时器模式
#define NEUTRAL_DUTY       1229          // 1.5ms脉冲对应的占空比（14bit）
//死区参数
#define DEADZONE_US        50      // 死区时间50us（根据实际需求调整）
#define DEADZONE_LOW_MS    (1.5f - (DEADZONE_US/1000.0f/2))  // 1.475ms
#define DEADZONE_HIGH_MS   (1.5f + (DEADZONE_US/1000.0f/2)) // 1.525ms

/* GPIO通道映射表［根据实际接线修改］*/
static const struct {
    ledc_channel_t ch_num;  // LEDC通道号
    int gpio;               // 物理GPIO引脚
} CHANNEL_MAP[] = {
    {LEDC_CHANNEL_0, 13},    // 通道0 -> GPIO4
    {LEDC_CHANNEL_1, 12},    // 通道1 -> GPIO5
    {LEDC_CHANNEL_2, 14},    // 通道2 -> GPIO6
    {LEDC_CHANNEL_3, 27},    // 通道3 -> GPIO7
    {LEDC_CHANNEL_4, 26},    // 通道4 -> GPIO8
    {LEDC_CHANNEL_5, 25},    // 通道5 -> GPIO9
    {LEDC_CHANNEL_6, 33},   // 通道6 -> GPIO18
    {LEDC_CHANNEL_7, 32}    // 通道7 -> GPIO19
};

/* ［硬件定时器配置］*/
static const ledc_timer_t TIMER_MAP[] = {
    LEDC_TIMER_0,  // 通道0-3使用定时器0
    LEDC_TIMER_1   // 通道4-7使用定时器1
};

/* ［全局状态跟踪］*/
bool pwm_initialized = false; // 初始化为未初始化状态

/**
 * @brief 初始化PWM硬件控制器
 * @note 必须优先调用此初始化函数
 */
void pwm_init(void) {
    if(pwm_initialized) {
        ESP_LOGW(TAG, "PWM already initialized");
        return;
    }

    // 安装LEDC渐变服务（无中断）
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ESP_LOGI(TAG, "LEDC fade service installed");

    // 配置定时器
    for(int i=0; i<sizeof(TIMER_MAP)/sizeof(TIMER_MAP[0]); i++) {
        ledc_timer_config_t timer_cfg = {
            .speed_mode = PWM_TIMER_MODE,
            .duty_resolution = PWM_RESOLUTION,
            .timer_num = TIMER_MAP[i],
            .freq_hz = PWM_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    }
    ESP_LOGI(TAG, "Timers configured");

    // 配置所有通道
    for(int ch=0; ch<sizeof(CHANNEL_MAP)/sizeof(CHANNEL_MAP[0]); ch++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num = CHANNEL_MAP[ch].gpio,
            .speed_mode = PWM_TIMER_MODE,
            .channel = CHANNEL_MAP[ch].ch_num,
            .timer_sel = TIMER_MAP[ch/4], // 每4通道共享一个定时器
            .duty = NEUTRAL_DUTY,         // 初始占空比
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
        
        // 立即应用中位值
        ledc_set_duty_and_update(PWM_TIMER_MODE, 
                                CHANNEL_MAP[ch].ch_num, 
                                NEUTRAL_DUTY, 0);
    }
    ESP_LOGI(TAG, "%d channels configured", 
            sizeof(CHANNEL_MAP)/sizeof(CHANNEL_MAP[0]));

    pwm_initialized = true;
    ESP_LOGI(TAG, "PWM initialization complete");
}

/**
 * @brief 安全设置PWM输出
 * @param ch 通道号（0-7）
 * @param duty 目标占空比（0-16383）
 * @param fade_ms 渐变时间（毫秒，0表示立即切换）
 */
void pwm_set_duty(uint8_t ch, uint32_t duty, uint32_t fade_ms) {
    if(!pwm_initialized) {
        ESP_LOGE(TAG, "PWM not initialized!");
        return;
    }

    if(ch >= sizeof(CHANNEL_MAP)/sizeof(CHANNEL_MAP[0])) {
        ESP_LOGE(TAG, "Invalid channel: %d", ch);
        return;
    }

    // 计算安全范围（1ms-2ms脉冲）
    const uint32_t min_duty = (uint32_t)((1.0/20.0)*(1<<14));  // 1ms
    const uint32_t max_duty = (uint32_t)((2.0/20.0)*(1<<14));  // 2ms
    duty = (duty < min_duty) ? min_duty : 
          (duty > max_duty) ? max_duty : duty;

    if(fade_ms > 0) {
        // 硬件渐变模式
        ESP_ERROR_CHECK(ledc_set_fade_time_and_start(
            PWM_TIMER_MODE,
            CHANNEL_MAP[ch].ch_num,
            duty,
            fade_ms,
            LEDC_FADE_NO_WAIT));
ESP_LOGD(TAG, "Channel %" PRIu8 " fading to %" PRIu32 " in %" PRIu32 "ms", 
        (uint8_t)ch, duty, fade_ms);
    } else {
        // 立即切换模式
        ESP_ERROR_CHECK(ledc_set_duty_and_update(
            PWM_TIMER_MODE,
            CHANNEL_MAP[ch].ch_num,
            duty,
            0));
ESP_LOGD(TAG, "Channel %" PRIu8 " set to %" PRIu32 " immediately", ch, duty);
    }
}


/**
 * @brief 安全停止所有PWM输出（渐变停止）
 * @param fade_ms 停止渐变时间（毫秒）
 */
void pwm_stop_all(uint32_t fade_ms) {
    if(!pwm_initialized) return;

    for(int ch=0; ch<sizeof(CHANNEL_MAP)/sizeof(CHANNEL_MAP[0]); ch++) {
        pwm_set_duty(ch, NEUTRAL_DUTY, fade_ms);
    }
    ESP_LOGI(TAG, "All channels returning to neutral");
}

/**
 * @brief 设置推进器输出功率（带自动方向控制）
 * @param ch 通道号（0-7）
 * @param power 输出功率百分比[-100, 100]
 *             -100: 最大反向转速
 *              0:   停止
 *              100: 最大正向转速
 * @param fade_ms 渐变时间（毫秒）
 * 50Hz 时约 400-1000ms
 * 保持 2-5Hz 的更新频率，可获得最佳控制效果与系统稳定性。
 */
void set_thruster_power(uint8_t ch, float power, uint32_t fade_ms) {
    // 参数有效性检查
    if(!pwm_initialized || ch >= sizeof(CHANNEL_MAP)/sizeof(CHANNEL_MAP[0])) {
        ESP_LOGE(TAG, "Invalid channel or PWM not initialized");
        return;
    }

    // 功率范围限制
    power = (power < -100.0f) ? -100.0f : 
           (power > 100.0f) ? 100.0f : power;

    // 计算占空比（基于线性映射）
    const float pulse_min = 1.0f;   // 1ms
    const float pulse_max = 2.0f;   // 2ms
    const float pulse_neutral = 1.5f; // 1.5ms
    
    // 计算脉冲宽度（毫秒）
    float pulse_width;

//|-----反向区间-----|--死区--|-----正向区间-----|
//1.0ms          1.475ms 1.5ms 1.525ms         2.0ms

     if(fabs(power) < 0.1f) {  // 接近零功率时直接设为死区
        pulse_width = 1.5f;
    } else {
        if(power > 0) {
            // 正向区间映射（考虑死区上限）
            pulse_width = DEADZONE_HIGH_MS + 
                         (2.0f - DEADZONE_HIGH_MS) * (power / 100.0f);
        } else {
            // 反向区间映射（考虑死区下限）
            pulse_width = DEADZONE_LOW_MS + 
                         (1.0f - DEADZONE_LOW_MS) * (fabsf(power) / 100.0f);
        }
    }

    // 强制死区处理（双重保险）
    if(pulse_width > DEADZONE_LOW_MS && pulse_width < DEADZONE_HIGH_MS) {
        pulse_width = 1.5f;
       // ESP_LOGW(TAG, "Ch%d in deadzone, force neutral", ch);
    }

    // 转换为占空比（14bit分辨率）
    const float period_ms = 20.0f; // 50Hz周期
    uint32_t duty = (uint32_t)((pulse_width / period_ms) * (1 << 14));

    // 设置占空比
    pwm_set_duty(ch, duty, fade_ms);
    
    ESP_LOGD(TAG, "Ch%d Power:%.1f%% => Pulse:%.2fms Duty:%" PRIu32,
         ch, power, pulse_width, duty);
}

/**
 * @brief 设置所有推进器统一功率
 * @param power 输出功率百分比[-100, 100]
 * @param fade_ms 渐变时间（毫秒）
 */
void set_all_thrusters(float power, uint32_t fade_ms) {
    for(int ch=0; ch<8; ch++) {
        set_thruster_power(ch, power, fade_ms);
    }
}

/**
 * @brief 初始化电机设置。
 *
 * 此函数将所有推进器设置为停止状态（0%功率），并保持该状态3秒钟，以确保电调初始化完成。
 * 可选地，可以在初始化完成后添加提示音或LED指示。
 */
void initialize_motors(void)
{
    // 1. 最大油门信号（2ms）
    set_all_thrusters(100.0f, 800);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 2. 最小油门信号（1ms）
    set_all_thrusters(-100.0f, 800);
    vTaskDelay(pdMS_TO_TICKS(1000));


    // 立即设置所有推进器到停止位（0%功率）
    set_all_thrusters(0.0f, 400);

    // 保持停止位3秒用于电调初始化
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 可选：初始化完成后可添加提示音或LED指示
}
