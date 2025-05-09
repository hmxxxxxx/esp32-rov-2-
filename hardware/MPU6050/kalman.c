#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include <esp_err.h>
#include "esp_log.h"  // 定义 ESP_LOGE 等日志宏
#include <math.h>
#include"kalman.h"

/*各轴偏移量变量*/
float ax_offset = 0;
float ay_offset = 0;
float az_offset = 0;
float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

float Roll ;
float Pitch ;
float Yaw ;

#define AccSPL        16384.0f    //加速度计灵敏度：±2g量程下为16384 LSB/g
#define GyroSPL       131.0f      //陀螺仪灵敏度：±250°/s量程下为131 LSB/(°/s)


/**
  * 函数功能：读取MPU6050加速度和角速度数据
  * 入口参数：AccX ：获取X轴加速度
  * 入口参数：AccY ：获取Y轴加速度
  * 入口参数：AccX ：获取Z轴加速度
  * 入口参数：GyroX：获取X轴角速度
  * 入口参数：GyroY：获取Y轴角速度
  * 入口参数：GyroZ：获取Z轴角速度
  * 返 回 值：无
  */
void Kalman_GetMPU6050_Data(float *AccX, float *AccY, float *AccZ,
                            float *GyroX, float *GyroY, float *GyroZ)
{
   
 	MPU6050_GetData(&mpu6050,&AX, &AY, &AZ, &GX, &GY, &GZ);

    /*** 加速度值 = 原始数据 / 灵敏度 ***/
    *AccX = AX / AccSPL - ax_offset;    
    *AccY = AY / AccSPL - ay_offset;
    *AccZ = AZ / AccSPL - az_offset;
    
    /*** 角速度值 = 原始数据 / 灵敏度 ***/
    *GyroX = GX / GyroSPL - gx_offset;
    *GyroY = GY / GyroSPL - gy_offset;
    *GyroZ = GZ / GyroSPL - gz_offset;
    
}

/**
  * 函数功能：获取MPU6050水平时各轴的偏移量
  * 入口参数：Offset_ax：获取加速度X轴偏移量
  * 入口参数：Offset_ay：获取加速度Y轴偏移量
  * 入口参数：Offset_az：获取加速度Z轴偏移量
  * 入口参数：Offset_gx：获取角速度X轴偏移量
  * 入口参数：Offset_gy：获取角速度Y轴偏移量
  * 入口参数：Offset_gz：获取角速度Z轴偏移量
  * 返 回 值：无
  */
void Kalman_GetMPU6050_Offset(float *Offset_ax, float *Offset_ay, float *Offset_az, 
                                float *Offset_gx, float *Offset_gy, float *Offset_gz)
{
    /*定义求平均值的个数*/
    uint16_t i;                 //重复次数变量
    float Count = 50.0;         //求平均数个数变量
    
    /*定义加速度角速度暂存变量*/
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    
    /*定义偏移量暂存变量*/
    static float Offset_a[3];
    static float Offset_g[3];
    
   // Delay_ms(10);
   vTaskDelay(pdMS_TO_TICKS(1)); // 延迟1毫秒
    /*获取数据总和*/
    for (i = 0; i < Count; i++)
    {
        /*获取加速度角速度数据*/
        //Delay_us(20);
        Kalman_GetMPU6050_Data(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ); 
        
        /*获取数据总和*/
        Offset_a[0] += AccX;
        Offset_a[1] += AccY;
        Offset_a[2] += AccZ;
        Offset_g[0] += GyroX;
        Offset_g[1] += GyroY;
        Offset_g[2] += GyroZ; 
    }
    
    /*求出平均值*/
    Offset_a[0] /= Count;
    Offset_a[1] /= Count;
    Offset_a[2] /= Count;
    Offset_g[0] /= Count;
    Offset_g[1] /= Count;
    Offset_g[2] /= Count;
    
    /*求出偏移量*/
    *Offset_ax = Offset_a[0] - 0.0;
    *Offset_ay = Offset_a[1] - 0.0;
    *Offset_az = Offset_a[2] - 1.0;  // Z轴加速度标准是等于1g
    *Offset_gx = Offset_g[0] - 0.0;
    *Offset_gy = Offset_g[1] - 0.0;
    *Offset_gz = Offset_g[2] - 0.0; 

}    

/**
  * 函数功能：卡尔曼滤波初始化
  * 入口参数：无
  * 返 回 值：无
  */
void Kalman_Init(void)
{
    /*计算水平时MPU6050的偏移量*/
    Kalman_GetMPU6050_Offset(&ax_offset , &ay_offset, &az_offset, &gx_offset, &gy_offset, &gz_offset);   
}

/**
  * 函数功能：获取加速度、角速度角度
  * 入口参数：Roll_A ：获取加速度横滚角度
  * 入口参数：Pitch_A：获取加速度俯仰角度
  * 入口参数：Roll_G ：获取角速度横滚角度
  * 入口参数：Pitch_G：获取角速度俯仰角度
  * 入口参数：Yaw_G  ：获取角速度偏航角度
  * 返 回 值：无
  */
void Kalman_GetEuler_TempAngle(float *Roll_A, float *Pitch_A,
                                float *Roll_G, float *Pitch_G, float *Yaw_G)
{
    /*定义积分时间变量*/
    static float dt = 0.01;
    static uint32_t last_time = 0;
    uint32_t now = xTaskGetTickCount();
    
    // 动态计算实际的dt值
    if (last_time != 0) {
        dt = (float)(now - last_time) / 1000.0f; // 转换为秒
    }
    last_time = now;
    
    /*定义暂存变量*/
    float AccX, AccY, AccZ;
    static float GyroX, GyroY, GyroZ;
 
    /*读取加速度和角速度*/
    Kalman_GetMPU6050_Data(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
    
    // 数据有效性检查
    float acc_total_vector = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ);
    if (acc_total_vector > 0.7f && acc_total_vector < 1.3f) { // 检查加速度是否在合理范围内（约1g）
        /*加速度求夹角*/
        /* 横滚角 = atan2(AccY, Accz) * 180 / Π  */
        *Roll_A = atan2f(AccY, AccZ) * 57.295779513f; // 使用更精确的常数
        
        /* 俯仰角 = -atan2(AccX, sqrt(AccY * AccY + AccZ * Accz)) * 180 / Π   */
        float denominator = sqrt(AccY * AccY + AccZ * AccZ);
        if (denominator != 0) {
            *Pitch_A = -atan2f(AccX, denominator) * 57.295779513f;
        }
        
        // 限制角度范围
        if (*Roll_A > 90.0f) *Roll_A = 90.0f;
        if (*Roll_A < -90.0f) *Roll_A = -90.0f;
        if (*Pitch_A > 90.0f) *Pitch_A = 90.0f;
        if (*Pitch_A < -90.0f) *Pitch_A = -90.0f;
    }
    
  
    if (fabs(GyroX) > 0.05f) *Roll_G += GyroX * dt;
    if (fabs(GyroY) > 0.05f) *Pitch_G += GyroY * dt;
    
    // 角度归一化处理
    *Roll_G = fmod(*Roll_G, 360.0f);
    *Pitch_G = fmod(*Pitch_G, 360.0f);
    *Yaw_G = fmod(*Yaw_G, 360.0f);
    
}

/**
  * 函数功能：卡尔曼滤波Roll
  * 入口参数：Angle_a：加速度Roll
  * 入口参数：Angle_g：角速度Roll
  * 返 回 值：无
  */
float Kalman_Filter_Roll(float Angle_a, float Angle_g)
{
   // ================== 结构体封装 ==================
    typedef struct {
        float angle;     // 估计角度（横滚角）
        float gyro_bias; // 陀螺仪零偏
    } KalmanState;

    static KalmanState state = {0};  // 初始状态

    // ================== 参数配置 ==================
    static const float dt = 0.01f;       // 固定采样周期10ms（如需动态dt需改造）
    static const float Q_angle = 0.0002f; // 角度过程噪声（状态不确定性）
    static const float Q_gyro = 0.0015f;  // 零偏过程噪声（零偏变化率）
    static const float R_angle = 0.3f;    // 加速度计观测噪声

    // ================== 协方差矩阵 ==================
    static float P[2][2] = { {1.0f, 0.0f},   // 初始协方差矩阵
                             {0.0f, 1.0f} }; // 保持对称性

    // ================== 预测阶段 ==================
    // 1. 状态预测（显式使用状态结构体）
    state.angle += (Angle_g - state.gyro_bias) * dt;

    // 2. 协方差预测（严格按 P = F*P*F^T + Q 计算）
    const float F[2][2] = {{1.0f, -dt},    // 状态转移矩阵
                           {0.0f, 1.0f}};  // [角度, 零偏]
    
    // 中间计算：FP = F * P
    float FP[2][2] = {
        {F[0][0]*P[0][0] + F[0][1]*P[1][0], F[0][0]*P[0][1] + F[0][1]*P[1][1]},
        {F[1][0]*P[0][0] + F[1][1]*P[1][0], F[1][0]*P[0][1] + F[1][1]*P[1][1]}
    };

    // P = FP * F^T + Q
    P[0][0] = FP[0][0]*F[0][0] + FP[0][1]*F[0][1] + Q_angle;
    P[0][1] = FP[0][0]*F[1][0] + FP[0][1]*F[1][1];
    P[1][0] = P[0][1];  // 强制保持对称性
    P[1][1] = FP[1][0]*F[1][0] + FP[1][1]*F[1][1] + Q_gyro;

    // ================== 更新阶段 ==================
    // 3. 卡尔曼增益计算（增加分母保护）
    const float denominator = P[0][0] + R_angle;
    if(fabsf(denominator) < 1e-6f) return state.angle; // 防止除零错误
    
    const float K_angle = P[0][0] / denominator;
    const float K_bias = P[1][0] / denominator;  // 利用协方差对称性

    // 4. 状态更新（使用显式误差变量）
    const float angle_error = Angle_a - state.angle;
    state.angle   += K_angle * angle_error;
    state.gyro_bias += K_bias * angle_error;

    // 5. 协方差更新（严格按 P = (I - K*H)*P 计算）
    const float I_KH[2][2] = {{1.0f - K_angle, 0.0f},
                              {-K_bias,        1.0f}};
                              
    const float P_new[2][2] = {
        {I_KH[0][0]*P[0][0] + I_KH[0][1]*P[1][0], 
         I_KH[0][0]*P[0][1] + I_KH[0][1]*P[1][1]},
        {I_KH[1][0]*P[0][0] + I_KH[1][1]*P[1][0],
         I_KH[1][0]*P[0][1] + I_KH[1][1]*P[1][1]}
    };

    // 更新协方差矩阵并保持对称
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];

    return state.angle;
}

/**
  * 函数功能：卡尔曼滤波Pitch
  * 入口参数：Angle_a：加速度Pitch
  * 入口参数：Angle_g：角速度Pitch
  * 返 回 值：无
  */
float Klaman_Filter_Pitct(float Angle_a, float Angle_g)
{
    // 定义状态结构体，提高可读性（可选）
    typedef struct {
        float angle;      // 估计角度
        float gyro_bias;  // 陀螺仪零偏
    } State;

    static State state = {0}; // 初始状态（角度和零偏）

    // 参数定义（增加注释说明物理意义）
    static const float dt = 0.01f;       // 采样周期10ms
    static const float Q_angle = 0.0002f; // 角度过程噪声（状态不确定性）
    static const float Q_gyro = 0.0015f;  // 零偏过程噪声（零偏变化率）
    static const float R_angle = 0.3f;    // 加速度计观测噪声

    // 协方差矩阵（对称矩阵，只需存储上三角）
    static float P[2][2] = { 
        {1.0f, 0.0f},   // P[0][0]: 角度方差, P[0][1]: 角度与零偏协方差
        {0.0f, 1.0f}    // P[1][0]: 协方差对称, P[1][1]: 零偏方差
    };

    //------------------------ 预测阶段 ---------------------------
    // 1. 状态预测：使用陀螺仪角速度（扣除零偏）更新角度
    state.angle += (Angle_g - state.gyro_bias) * dt;

    // 2. 协方差预测: P = F * P * F^T + Q
    // 定义状态转移矩阵F = [[1, -dt], [0, 1]]
    // 手动展开矩阵乘法避免复杂计算
    float F[2][2] = {{1.0f, -dt}, {0.0f, 1.0f}};
    
    // 计算 F * P
    float FP[2][2] = {
        {F[0][0]*P[0][0] + F[0][1]*P[1][0], F[0][0]*P[0][1] + F[0][1]*P[1][1]},
        {F[1][0]*P[0][0] + F[1][1]*P[1][0], F[1][0]*P[0][1] + F[1][1]*P[1][1]}
    };
    
    // 计算 (F * P) * F^T
    P[0][0] = FP[0][0]*F[0][0] + FP[0][1]*F[0][1] + Q_angle;
    P[0][1] = FP[0][0]*F[1][0] + FP[0][1]*F[1][1];
    P[1][0] = FP[1][0]*F[0][0] + FP[1][1]*F[0][1]; // 对称性 P[1][0] = P[0][1]
    P[1][1] = FP[1][0]*F[1][0] + FP[1][1]*F[1][1] + Q_gyro;

    //------------------------ 更新阶段 ---------------------------
    // 3. 计算卡尔曼增益 K = P * H^T / (H * P * H^T + R)
    // 观测矩阵 H = [1, 0]
    float denominator = P[0][0] + R_angle;
    float K_angle = P[0][0] / denominator;  // 角度增益
    float K_bias = P[1][0] / denominator;   // 零偏增益（利用协方差对称性）

    // 4. 状态更新：使用加速度计观测值修正
    float angle_error = Angle_a - state.angle;
    state.angle += K_angle * angle_error;
    state.gyro_bias += K_bias * angle_error;

    // 5. 协方差更新: P = (I - K * H) * P
    float I_KH[2][2] = {{1 - K_angle, 0}, {-K_bias, 1}}; // I - K * H
    float P_new[2][2] = {
        {I_KH[0][0]*P[0][0] + I_KH[0][1]*P[1][0], I_KH[0][0]*P[0][1] + I_KH[0][1]*P[1][1]},
        {I_KH[1][0]*P[0][0] + I_KH[1][1]*P[1][0], I_KH[1][0]*P[0][1] + I_KH[1][1]*P[1][1]}
    };
    // 更新P并保持对称
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0]; 
    P[1][1] = P_new[1][1];

    return state.angle;
}


/**
  * 函数功能：获取卡尔曼滤波后的欧拉角
  * 入口参数： Roll：获取经过卡尔曼滤波的横滚角
  * 入口参数：Pitch：获取经过卡尔曼滤波的俯仰角
  * 入口参数：  Yaw：获取没有经过卡尔曼滤波的偏航角(这个角度不准)
  * 返 回 值：无
  */
void Kalman_GetEuler_Angle(float *Roll, float *Pitch, float *Yaw)
{
    /*定义暂存加速度欧拉角变量*/
    static float Roll_a;
    static float Pitch_a;
    //static float Yaw_a;  
    
    /*定义暂存角速度欧拉角变量*/
    static float Roll_g;
    static float Pitch_g;
    static float Yaw_g;
     
    /*读取加速度和角速度的原始欧拉角*/
    Kalman_GetEuler_TempAngle(&Roll_a, &Pitch_a, &Roll_g, &Pitch_g, &Yaw_g);
    
    /*对加速度和角速度欧拉角进行卡尔曼滤波*/
    *Roll  = Kalman_Filter_Roll(Roll_a, Roll_g);
    *Pitch = Klaman_Filter_Pitct(Pitch_a, Pitch_g);
    *Yaw   = Yaw_g * 5.62;
     
}