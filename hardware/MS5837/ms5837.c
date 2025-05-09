
#include "ms5837.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "myiic.h"
#include <esp_err.h>

static const char *TAG = "MS5837";
MS5837_Handle ms5837;

static esp_err_t ms5837_reset(MS5837_Handle *ms5837)
{
    uint8_t cmd = MS5837_RESET_CMD;
    ESP_LOGI(TAG, "Resetting MS5837...");
    esp_err_t ret = myiic_write_bytes(ms5837->dev_handle, cmd, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); // 复位后必须延时20ms
    return ret;
}

static esp_err_t ms5837_read_prom(MS5837_Handle *ms5837)
{
    esp_err_t ret;
    uint8_t data[2];

    // 读取所有校准系数
   for (uint8_t i = 0; i < 7; i++) {
    ret = myiic_read_bytes(ms5837->dev_handle, MS5837_PROM_READ + (i << 1), data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PROM word %d", i);
        return ret;
    }
    ms5837->C[i] = ((uint16_t)data[0] << 8) | data[1];
    ESP_LOGI(TAG, "C[%d] = %d (0x%04X)", i, ms5837->C[i], ms5837->C[i]);  // 确保i从0开始
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }

   if (ms5837->C[0] == 0 || ms5837->C[0] == 0xFFFF) {
        ESP_LOGE(TAG, "Invalid calibration data");
        return ESP_ERR_INVALID_STATE;
    } 

    return ESP_OK;
}

static esp_err_t ms5837_read_adc(MS5837_Handle *ms5837, uint8_t cmd, uint32_t *adc_value)
{
    esp_err_t ret;
    uint8_t data[3];
    
    // 发送转换命令
    ret = myiic_write_bytes(ms5837->dev_handle, cmd, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send conversion command 0x%02X", cmd);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 等待转换完成

    // 读取ADC值
    ret = myiic_read_bytes(ms5837->dev_handle, MS5837_ADC_READ, data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC value");
        return ret;
    }

    *adc_value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
   // ESP_LOGI(TAG, "ADC Read - CMD: 0x%02X, Value: %lu (0x%06lX)", 
    //         cmd, *adc_value, *adc_value);

    return ESP_OK;
}

esp_err_t ms5837_init(MS5837_Handle *ms5837, i2c_master_bus_handle_t bus_handle)
{
    esp_err_t ret;

    // 添加设备到I2C总线
    ret = myiic_add_device(bus_handle, &ms5837->dev_handle, MS5837_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MS5837 device to I2C bus");
        return ret;
    }

    // 复位设备
    ret = ms5837_reset(ms5837);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed");
        return ret;
    }

    // 读取校准数据
    ret = ms5837_read_prom(ms5837);
   if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    // 读取初始压力值作为参考
    float initial_pressure, temp;
    for (int i = 0; i < 5; i++) {
        ret = ms5837_read_pressure_temperature(ms5837, &initial_pressure, &temp);
        if (ret != ESP_OK) {
            continue;
        }
        ms5837->initial_P += (int32_t)(initial_pressure * 10);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ms5837->initial_P /= 5;
    ESP_LOGI(TAG, "Initial atmospheric pressure: %.2f mbar", ms5837->initial_P / 10.0f);

    return ESP_OK;
}

esp_err_t ms5837_read_pressure_temperature(MS5837_Handle *ms5837, float *pressure, float *temperature)
{
    esp_err_t ret;
    
    // 读取原始压力值 (D1)
    ret = ms5837_read_adc(ms5837, MS5837_CONVERT_D1_8192, &ms5837->D1_Pres);
    if (ret != ESP_OK) return ret;

    // 读取原始温度值 (D2)
    ret = ms5837_read_adc(ms5837, MS5837_CONVERT_D2_8192, &ms5837->D2_Temp);
    if (ret != ESP_OK) return ret;

    // 计算温度差值和补偿
    if (ms5837->D2_Temp > ((uint32_t)ms5837->C[5] << 8)) {
        ms5837->dT = ms5837->D2_Temp - ((uint32_t)ms5837->C[5] << 8);
        ms5837->TEMP = 2000 + ((int64_t)ms5837->dT * ms5837->C[6]) / 8388608;
        ms5837->OFF = ((int64_t)ms5837->C[2] << 16) + ((int64_t)ms5837->C[4] * ms5837->dT) / 128;
        ms5837->SENS = ((int64_t)ms5837->C[1] << 15) + ((int64_t)ms5837->C[3] * ms5837->dT) / 256;
    } else {
        ms5837->dT = ((uint32_t)ms5837->C[5] << 8) - ms5837->D2_Temp;
        ms5837->TEMP = 2000 - ((int64_t)ms5837->dT * ms5837->C[6]) / 8388608;
        ms5837->OFF = ((int64_t)ms5837->C[2] << 16) - ((int64_t)ms5837->C[4] * ms5837->dT) / 128;
        ms5837->SENS = ((int64_t)ms5837->C[1] << 15) - ((int64_t)ms5837->C[3] * ms5837->dT) / 256;
    }

    // 二阶温度补偿
    if (ms5837->TEMP < 2000) {
        int64_t temp_diff = ms5837->TEMP - 2000;
        int64_t temp_diff_sq = temp_diff * temp_diff;
        int64_t T2 = (3 * ((int64_t)ms5837->dT * ms5837->dT)) / 8589934592LL;
        int64_t OFF2 = (3 * temp_diff_sq) / 2;
        int64_t SENS2 = (5 * temp_diff_sq) / 8;

        if (ms5837->TEMP < -1500) {
            temp_diff = ms5837->TEMP + 1500;
            temp_diff_sq = temp_diff * temp_diff;
            OFF2 += 7 * temp_diff_sq;
            SENS2 += 4 * temp_diff_sq;
        }

        ms5837->TEMP -= T2;
        ms5837->OFF -= OFF2;
        ms5837->SENS -= SENS2;
    } else {
        int64_t temp_diff = ms5837->TEMP - 2000;
        int64_t temp_diff_sq = temp_diff * temp_diff;
        int64_t T2 = (2 * ((int64_t)ms5837->dT * ms5837->dT)) / 137438953472LL;
        int64_t OFF2 = temp_diff_sq / 16;
        int64_t SENS2 = 0;

        ms5837->TEMP -= T2;
        ms5837->OFF -= OFF2;
        ms5837->SENS -= SENS2;
    }

    // 计算最终压力值
    ms5837->P = (((ms5837->D1_Pres * ms5837->SENS) / 2097152 - ms5837->OFF) / 8192) / 10;

    // 转换为实际值
    *temperature = ms5837->TEMP / 100.0f;
    *pressure = ms5837->P;

  //  ESP_LOGI(TAG, "Temperature: %.2f°C, Pressure: %.2f mbar", *temperature, *pressure);
    return ESP_OK;
}

esp_err_t ms5837_calculate_depth(float pressure, float fluid_density, float *depth)
{
    // 使用初始大气压力作为参考
    *depth = 0.983615f * (pressure - (ms5837.initial_P / 10.0f));
    
    // 验证深度值
    if (*depth < 0) {
        *depth = 0;
    }
    
  //  ESP_LOGI(TAG, "Depth calculation: P=%.2f mbar, density=%.2f kg/m3, depth=%.2f m",
    //         pressure, fluid_density, *depth);
    
    return ESP_OK;
}