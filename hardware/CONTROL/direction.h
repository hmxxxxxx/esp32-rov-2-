// hardware/CONTROL/direction.h
#ifndef __DIRECTION_H__
#define __DIRECTION_H__

#include <stdint.h>

// 运动方向定义
typedef enum {
    DIR_STOP = 0,    // 停止
    DIR_FORWARD,     // 前进
    DIR_BACKWARD,    // 后退
    DIR_LEFT,        // 左移
    DIR_RIGHT,       // 右移
    DIR_UP,          // 上浮
    DIR_DOWN,        // 下潜
    DIR_YAW_LEFT,    // 左转
    DIR_YAW_RIGHT,   // 右转
    DIR_COMBINED     // 组合运动
} ROV_Direction;

// 八推进器布局定义
typedef enum {
    // 推进器位置枚举（8推进器布局）
    THRUSTER_FRONT_LEFT = 0,     // 前左水平推进器（X轴平移/偏航控制）
    THRUSTER_FRONT_RIGHT=1,        // 前右水平推进器（对称布局，与左前形成力矩对）
    THRUSTER_BACK_LEFT=2,          // 后左水平推进器（X轴反向推力/Z轴力矩补偿）
    THRUSTER_BACK_RIGHT=3,         // 后右水平推进器（与后左形成横滚稳定对）
    
    // 垂直推进器组（控制Z轴运动）
    THRUSTER_VERTICAL_FRONT_LEFT=4,  // 前左垂直推进器（俯仰调节/深度保持）
    THRUSTER_VERTICAL_FRONT_RIGHT=5, // 前右垂直推进器（与前左垂推形成俯仰力矩）
    THRUSTER_VERTICAL_BACK_LEFT=6,   // 后左垂直推进器（横滚平衡/紧急上浮）
    THRUSTER_VERTICAL_BACK_RIGHT=7,  // 后右垂直推进器（与后左垂推形成横滚力矩）
    
    THRUSTER_COUNT=8               // 推进器总数（用于数组边界检查）
} ThrusterPosition;


// 运动控制参数结构体
typedef struct {
    float power;        // 总体功率百分比 (0-100)
    float forward;      // 前进分量 (-100 to 100)
    float strafe;       // 横向移动分量 (-100 to 100)
    float vertical;     // 垂直移动分量 (-100 to 100)
    float yaw;          // 偏航旋转分量 (-100 to 100)
    float roll;         // 横滚旋转分量 (-100 to 100)
    float pitch;        // 俯仰旋转分量 (-100 to 100)
} ROV_Motion;

/**
 * @brief 初始化方向控制系统
 */
void direction_control_init(void);

/**
 * @brief 设置ROV运动方向
 * @param motion 运动参数结构体指针
 * @param fade_ms 渐变时间(毫秒)
 */
void set_rov_direction(const ROV_Motion* motion, uint32_t fade_ms);

/**
 * @brief 平滑停止ROV运动
 * @param fade_ms 停止渐变时间(毫秒)
 */
void stop_rov(uint32_t fade_ms);

/**
 * @brief 当前ROV运动状态
 */
extern ROV_Motion current_motion;

#endif // __DIRECTION_H__