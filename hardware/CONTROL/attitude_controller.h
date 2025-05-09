#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "pid_controller.h"
#include "direction.h"  // 运动控制接口
#include <stdbool.h>

// 姿态控制器结构体
typedef struct {
    PID_Controller roll_pid;    // Roll轴PID控制器
    PID_Controller pitch_pid;   // Pitch轴PID控制器
    PID_Controller yaw_pid;     // Yaw轴PID控制器
    
    float control_period;       // 控制周期(秒)
    bool enabled;               // 控制器使能状态
} AttitudeController;

/**
 * @brief 初始化姿态控制器
 * @param ctrl 姿态控制器指针
 * @param control_period 控制周期(秒)
 */
void AttitudeController_Init(AttitudeController* ctrl, float control_period);


/**
 * @brief 设置目标姿态
 * @param ctrl 姿态控制器指针
 * @param roll 目标Roll角度(度，范围-180~180)
 * @param pitch 目标Pitch角度(度，范围-180~180)
 * @param yaw 目标Yaw角度(度，范围-180~180)
 */
void AttitudeController_SetTarget(AttitudeController* ctrl, 
                                 float roll, float pitch, float yaw);

/**
 * @brief 更新姿态控制器
 * @param ctrl 姿态控制器指针
 * @param roll_feedback Roll轴反馈角度(度)
 * @param pitch_feedback Pitch轴反馈角度(度)
 * @param yaw_feedback Yaw轴反馈角度(度)
 */
void AttitudeController_Update(AttitudeController* ctrl,
                              float roll_feedback, 
                              float pitch_feedback,
                              float yaw_feedback);

/**
 * @brief 启用/禁用控制器
 * @param ctrl 姿态控制器指针
 * @param enabled 启用状态
 */
void AttitudeController_SetEnabled(AttitudeController* ctrl, bool enabled);

#endif // ATTITUDE_CONTROLLER_H