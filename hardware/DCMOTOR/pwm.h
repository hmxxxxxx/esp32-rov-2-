#ifndef PWM_H
#define PWM_H
#include <stdint.h>
#include <stdbool.h>  // 添加这行引入bool类型


// 全局变量声明
extern bool pwm_initialized;

void pwm_init(void);  // 确保与定义完全一致

/**
 * @brief 设置PWM通道占空比
 * @param ch 通道号 (0 ~ PWM_MAX_CHANNELS-1)
 * @param duty 占空比值 (14位分辨率)
 * @param fade_ms 渐变时间(ms)，0表示立即生效
 */
void pwm_set_duty(uint8_t ch, uint32_t duty, uint32_t fade_ms);

/**
 * @brief 安全停止所有PWM输出（渐变停止）
 * @param fade_ms 停止渐变时间（毫秒）
 */
void pwm_stop_all(uint32_t fade_ms);

void set_all_thrusters(float power, uint32_t fade_ms); //设置所有推进器统一功率

void set_thruster_power(uint8_t ch, float power, uint32_t fade_ms); //设置推进器输出功率（带自动方向控制）

void initialize_motors(void); //初始化电机

#endif 