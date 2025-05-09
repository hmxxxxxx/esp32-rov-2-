
#ifndef MS5837_H
#define MS5837_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "myiic.h"

// MS5837 地址和命令
#define MS5837_ADDR                 0x76  // MS5837默认I2C地址
#define MS5837_RESET_CMD           0x1E  // 复位命令
#define MS5837_ADC_READ           0x00  // 读取ADC
#define MS5837_PROM_READ          0xA0  // 读取PROM基址
#define MS5837_CONVERT_D1_8192    0x48  // 转换D1(压力) OSR=8192
#define MS5837_CONVERT_D2_8192    0x58  // 转换D2(温度) OSR=8192

// MS5837数据结构
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint16_t C[7];        // 校准系数 C1-C6 (C[0]未使用)
    int32_t dT;          // 实际和参考温度之间的差异
    int32_t TEMP;        // 实际温度 * 100 (摄氏度)
    int64_t OFF;         // 实际温度下的压力偏移
    int64_t SENS;        // 实际温度下的压力灵敏度
    int32_t P;           // 实际压力 * 10 (mbar)
    int32_t initial_P;   // 初始化时的大气压力值（用于深度计算）
    uint32_t D1_Pres;    // 数字压力值
    uint32_t D2_Temp;    // 数字温度值
} MS5837_Handle;

extern MS5837_Handle ms5837;  // 声明MS5837句柄实例

// 函数声明
esp_err_t ms5837_init(MS5837_Handle *ms5837, i2c_master_bus_handle_t bus_handle);
esp_err_t ms5837_read_pressure_temperature(MS5837_Handle *ms5837, float *pressure, float *temperature);
esp_err_t ms5837_calculate_depth(float pressure, float fluid_density, float *depth);

#endif /* MS5837_H */