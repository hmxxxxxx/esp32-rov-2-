
#ifndef MYIIC_H
#define MYIIC_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

// I2C配置参数
#define I2C_MASTER_SCL_IO           GPIO_NUM_4      /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_5      /*!< GPIO number for I2C master data */
#define I2C_MASTER_NUM              I2C_NUM_0       /*!< I2C master port number */
#define I2C_MASTER_FREQ_HZ          50000          /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C master timeout value */

extern    i2c_master_bus_handle_t i2c_bus;

/**
 * @brief 初始化I2C主机总线
 * 
 * @param bus_handle 指向I2C总线句柄的指针
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t myiic_bus_init(i2c_master_bus_handle_t *bus_handle);

/**
 * @brief 添加I2C设备到总线
 * 
 * @param bus_handle I2C总线句柄
 * @param dev_handle 指向设备句柄的指针
 * @param device_address 设备I2C地址
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t myiic_add_device(i2c_master_bus_handle_t bus_handle, 
                          i2c_master_dev_handle_t *dev_handle,
                          uint8_t device_address);
/**
 * @brief 从I2C设备读取数据
 * 
 * @param dev_handle 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区指针
 * @param len 要读取的数据长度
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t myiic_read_bytes(i2c_master_dev_handle_t dev_handle, 
                          uint8_t reg_addr,
                          uint8_t *data,
                          size_t len);

/**
 * @brief 向I2C设备写入数据
 * 
 * @param dev_handle 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @param len 数据长度
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t myiic_write_bytes(i2c_master_dev_handle_t dev_handle,
                           uint8_t reg_addr,
                           uint8_t *data,
                           size_t len);

/**
 * @brief 向I2C设备写入单个字节
 * 
 * @param dev_handle 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t ESP_OK表示成功，其他值表示错误
 */
esp_err_t myiic_write_byte(i2c_master_dev_handle_t dev_handle,
                          uint8_t reg_addr,
                          uint8_t data);

#endif 