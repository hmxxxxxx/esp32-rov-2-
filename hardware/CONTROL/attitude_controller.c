#include "attitude_controller.h"
#include <math.h>

// PID参数默认值
#define DEFAULT_KP 2.0f
#define DEFAULT_KI 0.05f
#define DEFAULT_KD 0.5f
#define DEFAULT_MAX_OUTPUT 100.0f
#define DEFAULT_MAX_INTEGRAL 50.0f

void AttitudeController_Init(AttitudeController* ctrl, float control_period) {
    // 初始化PID控制器
    PID_Init(&ctrl->roll_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 
             DEFAULT_MAX_OUTPUT, DEFAULT_MAX_INTEGRAL);
    PID_Init(&ctrl->pitch_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 
             DEFAULT_MAX_OUTPUT, DEFAULT_MAX_INTEGRAL);
    PID_Init(&ctrl->yaw_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 
             DEFAULT_MAX_OUTPUT, DEFAULT_MAX_INTEGRAL);
    
    ctrl->control_period = control_period;
    ctrl->enabled = false;
}

/**
 * @brief 将角度规范化到-180~180范围
 * @param angle 输入角度(度)
 * @return 规范化后的角度(度)
 */
static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 设置姿态控制器的目标角度
 * @param ctrl 姿态控制器指针
 * @param roll 目标横滚角(度)
 * @param pitch 目标俯仰角(度)
 * @param yaw 目标偏航角(度)
 * @return 无
 * @note 输入角度会自动规范化到-180~180度范围
 */
void AttitudeController_SetTarget(AttitudeController *ctrl,
                                  float roll, float pitch, float yaw)
{
    // 规范化角度输入
    roll = normalize_angle(roll);
    pitch = normalize_angle(pitch);
    yaw = normalize_angle(yaw);
    
    PID_SetTarget(&ctrl->roll_pid, roll);
    PID_SetTarget(&ctrl->pitch_pid, pitch);
    PID_SetTarget(&ctrl->yaw_pid, yaw);
}

/**
 * @brief 更新姿态控制器状态并执行控制
 *
 * @param ctrl 姿态控制器结构体指针
 * @param roll_feedback 横滚角反馈值
 * @param pitch_feedback 俯仰角反馈值
 * @param yaw_feedback 偏航角反馈值
 *
 * @details 该函数执行以下操作:
 *          1. 检查控制器是否启用
 *          2. 分别更新横滚、俯仰、偏航三个方向的PID控制器
 *          3. 将PID输出转换为实际的运动控制指令
 *
 * @note 控制器必须先启用才能执行控制
 */
void AttitudeController_Update(AttitudeController *ctrl,
                               float roll_feedback,
                               float pitch_feedback,
                               float yaw_feedback)
{
    if (!ctrl->enabled) {
        return;
    }

    // 获取当前姿态角度(使用卡尔曼滤波)
   // float roll_feedback, pitch_feedback, yaw_feedback;
   // Kalman_GetEuler_Angle(&roll_feedback, &pitch_feedback, &yaw_feedback);
    
    // 更新PID控制器
    float roll_output = PID_Update(&ctrl->roll_pid, roll_feedback, ctrl->control_period);
    float pitch_output = PID_Update(&ctrl->pitch_pid, pitch_feedback, ctrl->control_period);
    float yaw_output = PID_Update(&ctrl->yaw_pid, yaw_feedback, ctrl->control_period);

    // 创建运动控制指令
    ROV_Motion motion = {
        .power = 100.0f,  // 使用最大功率
        .forward = 0.0f,  // 前进分量(由其他控制模块设置)
        .strafe = 0.0f,   // 横向分量(由其他控制模块设置)
        .vertical = 0.0f, // 垂直分量(由其他控制模块设置)
        .yaw = yaw_output, 
        .roll = roll_output,
        .pitch = pitch_output
    };

    // 设置死区阈值(±2%)
    #define DEADZONE 2.0f
    if (fabsf(motion.yaw) < DEADZONE) motion.yaw = 0.0f;
    if (fabsf(motion.roll) < DEADZONE) motion.roll = 0.0f;
    if (fabsf(motion.pitch) < DEADZONE) motion.pitch = 0.0f;

    // 应用非线性增益(平方曲线)
    motion.yaw = copysignf(motion.yaw * motion.yaw / 100.0f, motion.yaw);
    motion.roll = copysignf(motion.roll * motion.roll / 100.0f, motion.roll);
    motion.pitch = copysignf(motion.pitch * motion.pitch / 100.0f, motion.pitch);

    // 限制最大输出(±80%以防饱和)
    #define MAX_OUTPUT 80.0f
    motion.yaw = fmaxf(fminf(motion.yaw, MAX_OUTPUT), -MAX_OUTPUT);
    motion.roll = fmaxf(fminf(motion.roll, MAX_OUTPUT), -MAX_OUTPUT);
    motion.pitch = fmaxf(fminf(motion.pitch, MAX_OUTPUT), -MAX_OUTPUT);

    // 设置渐变时间(毫秒)
    const uint32_t fade_ms = (uint32_t)(ctrl->control_period * 1000);

    // 发送控制指令
    set_rov_direction(&motion, fade_ms);
}

/**
 * @brief 设置姿态控制器的启用状态
 *
 * @param ctrl 姿态控制器指针
 * @param enabled 是否启用控制器
 *
 * @details 当禁用控制器时,会重置所有PID控制器的状态。
 *          如果当前状态与目标状态相同,函数会直接返回。
 */
void AttitudeController_SetEnabled(AttitudeController *ctrl, bool enabled)
{
    if (ctrl->enabled == enabled)
    {
        return;
    }

    ctrl->enabled = enabled;
    
    // 禁用时重置PID控制器
    if (!enabled) {
        PID_Reset(&ctrl->roll_pid);
        PID_Reset(&ctrl->pitch_pid);
        PID_Reset(&ctrl->yaw_pid);
    }
}