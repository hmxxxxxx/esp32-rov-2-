#include "pca9685.h"
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

static const char *TAG1 = "PCA9685_MODULE";


PCA9685_Handle pca9685;  // 创建PCA9685_Handle结构体实例

 esp_err_t pca9685_init(PCA9685_Handle *pca9685, 
                      i2c_master_bus_handle_t bus_handle,  // 接收已初始化的总线句柄
                      uint8_t pwm_freq) 
{
    esp_err_t ret;
    
    // 不再初始化总线！直接使用传入的 bus_handle
    pca9685->bus_handle = bus_handle;  // 存储总线句柄（如果需要）

    // 添加设备到总线
    ret = myiic_add_device(bus_handle, 
                          &pca9685->dev_handle, 
                          PCA9685_SENSOR_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG1, "Failed to add PCA9685 device");
        return ret;
    }

    // ---- 以下为原有配置逻辑 ----
    // 步骤1: 进入睡眠模式
    ret = myiic_write_byte(pca9685->dev_handle, PCA9685_MODE1, MODE1_SLEEP_BIT);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    // 步骤2: 设置PWM频率
    uint8_t prescale_val = (25000000 / (4096 * pwm_freq)) - 1;
    ret = myiic_write_byte(pca9685->dev_handle, PCA9685_PRESCALE, prescale_val);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(500));  // 关键延迟！

    // 步骤3: 唤醒设备
    ret = myiic_write_byte(pca9685->dev_handle, PCA9685_MODE1, MODE1_AI_BIT);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    // 步骤4: 配置MODE2
    ret = myiic_write_byte(pca9685->dev_handle, PCA9685_MODE2, MODE2_OUTDRV);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}



// PWM设置函数
esp_err_t pca9685_set_pwm(PCA9685_Handle *pca9685, uint8_t channel, float duty_cycle) {
    // 参数有效性检查
    if(channel > 15) {
        ESP_LOGE(TAG1, "Invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    if(duty_cycle < 0 || duty_cycle > 100.0f) {
        ESP_LOGE(TAG1, "Duty cycle out of range: %.2f", duty_cycle);
        return ESP_ERR_INVALID_ARG;
    }

    // 计算12位PWM值
    uint16_t off_count;
    if(duty_cycle >= 100.0f) {
        // 全开
        off_count = 0x1000;
    } else if(duty_cycle <= 0.0f) {
        // 全关
        off_count = 0x0000;
    } else {
        off_count = (uint16_t)(duty_cycle * 4095.0f / 100.0f);
    }

    // 计算寄存器地址
    uint8_t reg_base = PCA9685_LED0_ON_L + (channel << 2); // 每个通道4个寄存器
    uint8_t pwm_data[] = {
        0x00,        // LED_ON_L (低8位)
        0x00,        // LED_ON_H (高4位)
        off_count & 0xFF,         // LED_OFF_L
        (off_count >> 8) & 0x0F   // LED_OFF_H
    };

    // 发送I2C命令
    esp_err_t ret = myiic_write_bytes(pca9685->dev_handle, reg_base, pwm_data, sizeof(pwm_data));
    
    if(ret == ESP_OK) {
        ESP_LOGI(TAG1, "Channel %d set to %.2f%%", channel, duty_cycle);
    } else {
        ESP_LOGI(TAG1, "Failed to set PWM: 0x%x", ret);
    }
    return ret;
}
