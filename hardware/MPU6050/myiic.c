
#include "myiic.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h" 
#include "driver/i2c_master.h"
#include <esp_err.h>
#include "sdkconfig.h"
#include <string.h> 

static const char *TAG = "MYIIC";

  i2c_master_bus_handle_t i2c_bus;

esp_err_t myiic_bus_init(i2c_master_bus_handle_t *bus_handle)
{
    ESP_LOGI(TAG, "Initializing I2C master bus...");
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    return ESP_OK;
}

/**
 * @brief 向 I2C 总线添加设备
 *
 * @param bus_handle I2C 总线句柄
 * @param dev_handle 指向 I2C 设备句柄的指针
 * @param device_address 设备地址
 *
 * @return
 *     - ESP_OK: 成功
 *     - 其他: 失败
 *
 * 该函数用于在指定的 I2C 总线上添加一个新的设备。它配置设备的地址长度为7位，
 * 并使用预定义的 I2C 主频率。函数成功时返回 ESP_OK，失败时返回错误代码。
 */
esp_err_t myiic_add_device(i2c_master_bus_handle_t bus_handle,
                           i2c_master_dev_handle_t *dev_handle,
                           uint8_t device_address)
{
    ESP_LOGI(TAG, "Adding I2C device with address 0x%02X", device_address);
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C device added successfully");
    return ESP_OK;
}

/**
 * @brief 从 I2C 设备读取多个字节数据
 * 
 * @param dev_handle I2C 主机设备句柄
 * @param reg_addr   要读取的寄存器地址
 * @param data       存储读取数据的缓冲区指针
 * @param len        要读取的字节数
 * 
 * @return esp_err_t 
 *         - ESP_OK: 读取成功
 *         - 其他: 读取失败
 */
esp_err_t myiic_read_bytes(i2c_master_dev_handle_t dev_handle, 
                          uint8_t reg_addr,
                          uint8_t *data,
                          size_t len)
{
    return i2c_master_transmit_receive(dev_handle, 
                                     &reg_addr,
                                     1,
                                     data,
                                     len,
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief 通过I2C总线向指定寄存器写入多个字节数据
 *
 * @param dev_handle I2C设备句柄
 * @param reg_addr 目标寄存器地址
 * @param data 要写入的数据缓冲区
 * @param len 要写入的数据长度
 *
 * @return
 *     - ESP_OK: 写入成功
 *     - ESP_ERR_NO_MEM: 内存分配失败
 *     - 其他: I2C通信错误
 */
esp_err_t myiic_write_bytes(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr,
                            uint8_t *data,
                            size_t len)
{
    uint8_t *write_buf = (uint8_t *)malloc(len + 1);
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for write buffer");
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    
    esp_err_t ret = i2c_master_transmit(dev_handle,
                                       write_buf,
                                       len + 1,
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    free(write_buf);
    return ret;
}

esp_err_t myiic_write_byte(i2c_master_dev_handle_t dev_handle,
                          uint8_t reg_addr,
                          uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle,
                              write_buf,
                              sizeof(write_buf),
                              I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}