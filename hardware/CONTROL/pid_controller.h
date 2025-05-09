#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float target;       // 目标值
    float last_error;   // 上一次误差
    float integral;     // 积分项
    float output;       // 输出值
    
    float max_output;   // 最大输出限制
    float max_integral; // 积分限幅
} PID_Controller;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 最大输出限制
 * @param max_integral 积分限幅
 */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, 
              float max_output, float max_integral);

/**
 * @brief 设置PID目标值
 * @param pid PID控制器指针
 * @param target 目标值
 */
void PID_SetTarget(PID_Controller* pid, float target);

/**
 * @brief 更新PID控制器
 * @param pid PID控制器指针
 * @param feedback 反馈值
 * @param dt 时间步长(秒)
 * @return PID输出值
 */
float PID_Update(PID_Controller* pid, float feedback, float dt);

/**
 * @brief 重置PID控制器
 * @param pid PID控制器指针
 */
void PID_Reset(PID_Controller* pid);

#endif // PID_CONTROLLER_H