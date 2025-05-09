#ifndef PCA9685_H
#define PCA9685_H
 
#include <stdint.h>
#include "driver/i2c_master.h"
#include "sdkconfig.h"

#define PCA9685_SENSOR_ADDR  0x40          // 默认地址
// PCA9685寄存器定义
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
// 添加必要的寄存器定义
#define PCA9685_LED0_ON_L  0x06
#define PCA9685_LED0_ON_H  0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09

// 模式寄存器配置
#define MODE1_AI_BIT    (1 << 5)  // 自动地址递增
#define MODE1_SLEEP_BIT (1 << 4)  
#define MODE2_OUTDRV    (1 << 2)  // 推挽输出模式

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} PCA9685_Handle;

extern PCA9685_Handle pca9685;  // 创建MPU6050_Handle结构体实例

esp_err_t pca9685_init(PCA9685_Handle *pca9685, 
                      i2c_master_bus_handle_t bus_handle,
                      uint8_t pwm_freq);
esp_err_t pca9685_set_pwm(PCA9685_Handle *pca9685, uint8_t channel, float duty_cycle);

#endif
