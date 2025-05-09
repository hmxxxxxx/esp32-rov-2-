#include "feedback.h"
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h" 
#include <esp_err.h>

static const char *TAG = "feedback";

/**
 * @brief CRC8校验计算
 * @param data 数据指针
 * @param len 数据长度
 * @return uint8_t CRC校验值
 */
static uint8_t calculate_crc(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
        }
    }
    return crc;
}

uint8_t parse_feedback_data(const uint8_t data[10], MotorFeedback* feedback) {
    // 检查CRC校验
    uint8_t crc = calculate_crc(data, 9);
    if (crc != data[9]) {
        return 1; // CRC校验失败
    }

    // 解析温度(Byte 0)
    feedback->temperature = (float)data[0];

    // 解析电压(Byte 1-2)
    feedback->voltage = (float)((data[1] << 8) | data[2]) * 0.01f;

    // 解析电流(Byte 3-4)
    feedback->current = (float)((data[3] << 8) | data[4]) * 0.01f;

    // 解析电量(Byte 5-6)
    feedback->capacity = (data[5] << 8) | data[6];

    // 解析电转速(Byte 7-8)
    feedback->erpm = (data[7] << 8) | data[8];
    
    // 计算实际转速
    if (feedback->pole_pairs > 0) {
        feedback->actual_rpm = (float)feedback->erpm * 100.0f / (float)feedback->pole_pairs;
    } else {
        feedback->actual_rpm = 0.0f;
    }

    // 存储CRC值
    feedback->crc = data[9];

    return 0; // 解析成功
}

void set_motor_pole_pairs(MotorFeedback* feedback, uint8_t pole_pairs) {
    feedback->pole_pairs = pole_pairs;
    
    // 如果已有erpm值，更新实际转速
    if (feedback->erpm > 0 && pole_pairs > 0) {
        feedback->actual_rpm = (float)feedback->erpm * 100.0f / (float)pole_pairs;
    }
}



// 电机反馈数据结构
static MotorFeedback motor_feedback;

/**
 * @brief 初始化UART0
 */
void motor_uart_init() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

   // 设置抗干扰参数
    uart_set_line_inverse(UART_PORT_NUM, UART_SIGNAL_INV_DISABLE);
    uart_set_rx_full_threshold(UART_PORT_NUM, 120);  // 提高接收阈值
    
    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 
                                       UART_BUF_SIZE, 0, NULL, 0));
    // 设置日志级别
    esp_log_level_set("FEEDBACK", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);


    ESP_LOGI(TAG, "UART initialized");
}

/**
 * @brief 读取电机反馈数据
 * @param timeout_ms 读取超时时间(毫秒)
 * @return int 0:成功 -1:超时 -2:CRC校验失败
 */
int read_motor_feedback(int timeout_ms) {
    uint8_t data[PACKET_SIZE];
    int received = 0;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
       // 清空缓冲区旧数据（关键修改）
    uart_flush_input(UART_PORT_NUM); 
    // 读取数据包
    while (received < PACKET_SIZE) {
        int len = uart_read_bytes(UART_PORT_NUM, data + received, 
                                 PACKET_SIZE - received, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            received += len;
                      // 重置超时计时器（关键修改）
            start_time = xTaskGetTickCount() * portTICK_PERIOD_MS; 
        }
            
    // 调试输出原始数据（仅在DEBUG级别）
  //  ESP_LOGD(TAG, "Raw data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //        data[0], data[1], data[2], data[3], data[4],
     //       data[5], data[6], data[7], data[8], data[9]);
        
        // 检查超时
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) > timeout_ms) {
            ESP_LOGE(TAG, "Read timeout");
            return -1;
        }
    }
    
    // 解析数据
    if (parse_feedback_data(data, &motor_feedback) != 0) {
        ESP_LOGE(TAG, "CRC check failed");
        return -2;
    }
    
    return 0;
}

/**
 * @brief 获取电机反馈数据
 * @return MotorFeedback* 电机反馈数据结构指针
 */
MotorFeedback* get_motor_feedback() {
    return &motor_feedback;
}

/**
 * @brief 设置电机极对数
 * @param pole_pairs 极对数
 */
void set_motor_pole_pairs_config(uint8_t pole_pairs) {
    set_motor_pole_pairs(&motor_feedback, pole_pairs);
}