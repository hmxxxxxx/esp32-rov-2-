/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <portmacro.h>
#include "mpu6050.h"
#include <esp_err.h>
#include <math.h>
#include "myiic.h"

const char *TAG = "example";
MPU6050_Handle mpu6050;  // 创建MPU6050_Handle结构体实例

// 变量定义
int16_t AX, AY, AZ; // 加速度计数据
int16_t GX, GY, GZ; // 陀螺仪数据


/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return myiic_read_bytes(dev_handle, reg_addr, data, len);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    return myiic_write_byte(dev_handle, reg_addr, data);
}

/**
 * @brief 初始化I2C设备（不再包含总线初始化）
 */
static esp_err_t i2c_master_init(i2c_master_bus_handle_t bus_handle,  // 接收已初始化的总线
                                i2c_master_dev_handle_t *dev_handle, 
                                uint8_t sensor_addr) 
{
    return myiic_add_device(bus_handle, dev_handle, sensor_addr);
}

/**
 * @brief MPU6050初始化函数（接收外部总线句柄）
 */
esp_err_t MPU6050_Init(MPU6050_Handle *mpu6050, 
                      i2c_master_bus_handle_t bus_handle)  // 新增总线参数
{
    uint8_t data[2];
    esp_err_t ret;

    // 初始化I2C设备（不再初始化总线）
    ret = i2c_master_init(bus_handle, &mpu6050->dev_handle, MPU6050_SENSOR_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device init failed");
        return ret;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    // ---- 以下为原有配置逻辑 ----
    // 验证设备ID
    ret = mpu6050_register_read(mpu6050->dev_handle, MPU6050_WHO_AM_I_REG_ADDR, data, 1);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    // 配置寄存器
    ret = mpu6050_register_write_byte(mpu6050->dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00);
    if (ret != ESP_OK) return ret;

    // 设置采样率为100Hz (1000Hz / (9 + 1) = 100Hz)
    ret = mpu6050_register_write_byte(mpu6050->dev_handle, MPU6050_SMPLRT_DIV, 0x09);
    if (ret != ESP_OK) return ret;

    // 设置数字低通滤波器带宽为5Hz，延迟18.6ms
    ret = mpu6050_register_write_byte(mpu6050->dev_handle, MPU6050_CONFIG, 0x06);
    if (ret != ESP_OK) return ret;

    // 设置陀螺仪量程为±500°/s，提高精度
    ret = mpu6050_register_write_byte(mpu6050->dev_handle, MPU6050_GYRO_CONFIG, 0x08);
    if (ret != ESP_OK) return ret;

    // 设置加速度计量程为±2g，提高精度
    ret = mpu6050_register_write_byte(mpu6050->dev_handle, MPU6050_ACCEL_CONFIG, 0x00);
    return ret;
}

/**
 * @brief 从MPU6050传感器读取加速度和陀螺仪数据
 *
 * 该函数从MPU6050传感器读取加速度计和陀螺仪的X、Y、Z轴数据，并将结果存储在提供的指针中。
 *
 * @param mpu6050 指向MPU6050句柄的指针
 * @param AccX 指向存储加速度计X轴数据的int16_t类型变量的指针
 * @param AccY 指向存储加速度计Y轴数据的int16_t类型变量的指针
 * @param AccZ 指向存储加速度计Z轴数据的int16_t类型变量的指针
 * @param GyroX 指向存储陀螺仪X轴数据的int16_t类型变量的指针
 * @param GyroY 指向存储陀螺仪Y轴数据的int16_t类型变量的指针
 * @param GyroZ 指向存储陀螺仪Z轴数据的int16_t类型变量的指针
 */
void MPU6050_GetData(MPU6050_Handle *mpu6050, int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t data[14];  // 用于存储所有传感器数据的缓冲区
    
    // 从ACCEL_XOUT_H开始连续读取14个字节的数据
    // 这包含了所有的加速度计和陀螺仪数据
    esp_err_t ret = mpu6050_register_read(mpu6050->dev_handle, MPU6050_ACCEL_XOUT_H, data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return;
    }

    // 合并高低字节并转换为16位有符号整数
    *AccX = (int16_t)((data[0] << 8) | data[1]);
    *AccY = (int16_t)((data[2] << 8) | data[3]);
    *AccZ = (int16_t)((data[4] << 8) | data[5]);
    
    // 跳过温度数据（data[6]和data[7]）
    
    *GyroX = (int16_t)((data[8] << 8) | data[9]);
    *GyroY = (int16_t)((data[10] << 8) | data[11]);
    *GyroZ = (int16_t)((data[12] << 8) | data[13]);
}

/**
  * 函数功能：线性标度变换
  * 入口参数：Sample_Value: 采样回来的原始数值 
  * 入口参数：URV：         量程上限      
  * 入口参数：LRV：         量程下限
  * 返 回 值：变换后的数据
  */
float Scale_Transform(float Sample_Value, float URV, float LRV)
{
    float Data;             //定义用来保存变换后的数据变量
    float Value_L = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
    float Value_U = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
    
    /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
    Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
           
    return Data;
}

/**
  * 函数功能：读取转换后加速度计数值
  * 入口参数：Accel_Value：加速度值转换后保存的变量
  * 返 回 值：无
  */
void MPU6050_GetAccel_Value(float *Accel_Value)
{
    /*直接使用AccSPL进行转换，量程为±2g*/
    Accel_Value[0] = (float)AX / AccSPL;  // X轴加速度，单位：g
    Accel_Value[1] = (float)AY / AccSPL;  // Y轴加速度，单位：g
    Accel_Value[2] = (float)AZ / AccSPL;  // Z轴加速度，单位：g
}

/**
  * 函数功能：读取转换后陀螺仪数值
  * 入口参数：Gyro_Value：陀螺仪值转换后保存的变量
  * 返 回 值：无
  */
void MPU6050_GetGyro_Value(float *Gyro_Value)
{
    /*直接使用GyroSPL进行转换，量程为±250°/s*/
    Gyro_Value[0] = (float)GX / GyroSPL;  // X轴角速度，单位：°/s
    Gyro_Value[1] = (float)GY / GyroSPL;  // Y轴角速度，单位：°/s
    Gyro_Value[2] = (float)GZ / GyroSPL;  // Z轴角速度，单位：°/s
}