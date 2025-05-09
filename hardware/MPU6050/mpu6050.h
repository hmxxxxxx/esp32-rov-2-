#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "sdkconfig.h"

#include "myiic.h"

#define MPU6050_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define	MPU6050_PWR_MGMT_2		    0x6C     //电源管理2
#define MPU6050_RESET_BIT           7
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

// Constants for sensor scaling
#define AccSPL 16384.0f    // ±2g range: 16384 LSB/g
#define GyroSPL 131.0f     // ±250°/s range: 131 LSB/(°/s)
#define RAD_TO_DEG 57.295779513f  // 180/PI，弧度转角度
#define FILTER_ALPHA 0.96f    // 互补滤波系数，调整加速度计和陀螺仪的权重

// 数据转换函数声明
void MPU6050_GetAccel_Value(float *Accel_Value);  // 获取加速度值，单位：g
void MPU6050_GetGyro_Value(float *Gyro_Value);    // 获取角速度值，单位：°/s

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} MPU6050_Handle;
extern MPU6050_Handle mpu6050;  // 创建MPU6050_Handle结构体实例

// 变量定义
extern int16_t AX, AY, AZ; // 加速度计原始数据
extern int16_t GX, GY, GZ; // 陀螺仪原始数据

// 函数声明
/**
 * @brief 向 MPU6050 设备的指定寄存器写入一个字节的数据。
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);


// 修改函数签名，增加总线句柄参数
esp_err_t MPU6050_Init(MPU6050_Handle *mpu6050, 
                      i2c_master_bus_handle_t bus_handle);  // 新增参数

/**
 * @brief 从 MPU6050 传感器读取数据。
 *
 * 此函数用于从 MPU6050 传感器读取加速度计、陀螺仪和温度数据。
 */
void MPU6050_GetData(MPU6050_Handle *mpu6050, int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
                     

#endif