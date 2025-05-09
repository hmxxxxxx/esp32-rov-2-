#include "pid_controller.h"
#include <math.h>

void PID_Init(PID_Controller* pid, float kp, float ki, float kd, 
              float max_output, float max_integral) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;
    pid->target = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_SetTarget(PID_Controller* pid, float target) {
    pid->target = target;
}

/**
 * @brief 更新PID控制器并计算输出值
 *
 * @param pid PID控制器结构体指针
 * @param feedback 当前反馈值
 * @param dt 采样时间间隔(单位:秒)
 *
 * @details 该函数执行以下操作:
 *          1. 计算误差并处理角度环绕(-180°到180°)
 *          2. 计算比例、积分、微分三项
 *          3. 应用积分限幅
 *          4. 计算并限制最终输出
 *
 * @return 返回PID控制器的输出值
 */
float PID_Update(PID_Controller *pid, float feedback, float dt)
{
    // 计算误差，考虑角度环绕问题(如从359°到0°)
    float error = pid->target - feedback;

    // 处理角度环绕问题(适用于-180°到180°范围)
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    
    // 计算比例项
    float proportional = pid->kp * error;
    
    // 计算积分项(带积分限幅)
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    float integral = pid->ki * pid->integral;
    
    // 计算微分项
    float derivative = pid->kd * (error - pid->last_error) / dt;
    pid->last_error = error;
    
    // 计算总输出
    pid->output = proportional + integral + derivative;
    
    // 限制输出范围
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }
    
    return pid->output;
}

void PID_Reset(PID_Controller* pid) {
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}