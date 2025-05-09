#ifndef __FEEDBACK_H__
#define __FEEDBACK_H__

#include <stdint.h>

// UART配置参数
#define UART_PORT_NUM    UART_NUM_0
#define UART_BAUD_RATE   115200
#define UART_RX_PIN      3
#define UART_TX_PIN      1
#define UART_BUF_SIZE    256
#define PACKET_SIZE      10  // 协议数据包大小
typedef struct {
    float temperature;    // 温度(℃)
    float voltage;        // 电压(V)
    float current;        // 电流(A)
    uint16_t capacity;    // 电量(mAh)
    uint16_t erpm;        // 电转速(100Rpm)
    float actual_rpm;     // 实际转速(Rpm)
    uint8_t crc;          // CRC校验值
    uint8_t pole_pairs;   // 电机极对数
} MotorFeedback;

/**
 * @brief 解析接收到的数据
 * @param data 接收到的10字节数据
 * @param feedback 解析结果存储结构体
 * @return uint8_t 0:成功 1:CRC校验失败
 */
uint8_t parse_feedback_data(const uint8_t data[10], MotorFeedback* feedback);

/**
 * @brief 设置电机极对数
 * @param feedback 反馈数据结构体指针
 * @param pole_pairs 极对数
 */
void set_motor_pole_pairs(MotorFeedback* feedback, uint8_t pole_pairs);
/**
 * @brief 初始化电机UART通信
 */
void motor_uart_init(void);

/**
 * @brief 读取电机反馈数据
 * @param timeout_ms 读取超时时间(毫秒)
 * @return int 0:成功 -1:超时 -2:CRC校验失败
 */
int read_motor_feedback(int timeout_ms);

/**
 * @brief 获取电机反馈数据
 * @return MotorFeedback* 电机反馈数据结构指针
 */
MotorFeedback* get_motor_feedback(void);

/**
 * @brief 设置电机极对数
 * @param pole_pairs 极对数
 */
void set_motor_pole_pairs_config(uint8_t pole_pairs);


#endif // __FEEDBACK_H__