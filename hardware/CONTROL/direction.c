// hardware/CONTROL/direction.c
#include "direction.h"
#include "pwm.h"
#include "esp_log.h"
#include <esp_err.h>
#include <math.h>
#include <string.h>

#define TAG "DIRECTION_CTRL"

// 推进器配置结构体
/**
 * @brief 推进器配置结构体
 *
 * 该结构体用于存储六个自由度运动的增益系数配置参数：
 * - 三个平移运动：前进、横向、垂直
 * - 三个旋转运动：偏航、横滚、俯仰
 *
 * 每个增益系数用于调节对应方向的推进器输出强度
 */
typedef struct
{
    float forward_gain;   // 前进增益系数
    float strafe_gain;    // 横向增益系数
    float vertical_gain;  // 垂直增益系数
    float yaw_gain;       // 偏航增益系数
    float roll_gain;      // 横滚增益系数
    float pitch_gain;     // 俯仰增益系数
} ThrusterConfig;

// ROV运动控制变量
 ROV_Motion current_motion = {
    .power = 0.0f,
    .forward = 0.0f,
    .strafe = 0.0f,
    .vertical = 0.0f,
    .yaw = 0.0f
};

// 八推进器配置 (根据实际布局调整系数)
/**
 * @brief 推进器配置数组,定义了8个推进器的方向矢量
 *
 * 数组包含8个推进器的配置:
 * - 4个水平推进器(前左、前右、后左、后右)
 * - 4个垂直推进器(前左垂、前右垂、后左垂、后右垂)
 *
 * 每个推进器配置包含6个浮点数,分别表示:
 * [前进分量, 横移分量, 垂直分量, 偏航分量, 横滚分量, 俯仰分量]
 *
 * 数值范围为[-1.0, 1.0],表示该方向的推力贡献比例
 */
static const ThrusterConfig thruster_config[THRUSTER_COUNT] = {
    // 水平推进器 (前左、前右、后左、后右)
    {-1.0f, -1.0f, 0.0f, 1.0f, 0.0f, -1.0f}, // FRONT_LEFT
    {-1.0f, 1.0f, 0.0f, -1.0f, 0.0f, -1.0f}, // FRONT_RIGHT
    {1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f},    // BACK_LEFT
    {1.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f},  // BACK_RIGHT

    // 垂直推进器 (前左垂、前右垂、后左垂、后右垂)
    {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, -1.0f}, // VERTICAL_FRONT_LEFT
    {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f},  // VERTICAL_FRONT_RIGHT
    {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f},  // VERTICAL_BACK_LEFT
    {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f}    // VERTICAL_BACK_RIGHT
};

void direction_control_init(void) {
    ESP_LOGI(TAG, "Direction control system initialized");
}

void set_rov_direction(const ROV_Motion* motion, uint32_t fade_ms) {
    // 参数检查
    if (!motion) {
        ESP_LOGE(TAG, "Invalid motion parameters");
        return;
    }

    // 计算各推进器输出
    float thruster_power[THRUSTER_COUNT] = {0};
    float max_power = 0;
    
    for (int i = 0; i < THRUSTER_COUNT; i++) {
        thruster_power[i] = 
            motion->forward * thruster_config[i].forward_gain +
            motion->strafe * thruster_config[i].strafe_gain +
            motion->vertical * thruster_config[i].vertical_gain +
            motion->yaw * thruster_config[i].yaw_gain +
            motion->roll * thruster_config[i].roll_gain +
            motion->pitch * thruster_config[i].pitch_gain;
        
        // 应用全局功率缩放
        thruster_power[i] *= (motion->power / 100.0f);
        
        // 记录最大功率用于归一化
        if (fabsf(thruster_power[i]) > max_power) {
            max_power = fabsf(thruster_power[i]);
        }
    }

    // 归一化处理 (防止超过100%功率)
    if (max_power > 100.0f) {
        float scale = 100.0f / max_power;
        for (int i = 0; i < THRUSTER_COUNT; i++) {
            thruster_power[i] *= scale;
        }
        ESP_LOGW(TAG, "Power scaled down to avoid saturation");
    }

    // 设置各推进器功率
    for (int i = 0; i < THRUSTER_COUNT; i++) {
        set_thruster_power(i, thruster_power[i], fade_ms);
        ESP_LOGD(TAG, "Thruster %d power: %.1f%%", i, thruster_power[i]);
    }
}

/**
 * @brief 使ROV平滑停止运动
 *
 * @param fade_ms 停止过程的渐变时间(毫秒)
 *
 * 该函数将ROV的所有运动参数(推进力、前进、平移、垂直、偏航、横滚、俯仰)
 * 平滑地降至0,实现柔和停止。停止过程持续时间由fade_ms参数控制。
 */
void stop_rov(uint32_t fade_ms)
{
    ROV_Motion stop_motion = {
        .power = 0.0f,
        .forward = 0.0f,
        .strafe = 0.0f,
        .vertical = 0.0f,
        .yaw = 0.0f,
        .roll = 0.0f,
        .pitch = 0.0f};
    set_rov_direction(&stop_motion, fade_ms);
    ESP_LOGI(TAG, "ROV stopped smoothly");
}

