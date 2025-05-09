#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h" 
#include "esp_system.h"
#include <esp_err.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "mpu6050.h"
#include "kalman.h"
#include "driver/ledc.h"
#include "pwm.h"
#include "pca9685.h"
#include "ms5837.h"
#include "feedback.h" 
#include "direction.h"
#include "pid_controller.h"
#include "attitude_controller.h"

// 全局变量定义
static const char *TAG = "ROV_Control";
AttitudeController attitude_ctrl;
ROV_Motion target_motion = {0};
float current_roll, current_pitch, current_yaw;
float pressure, temperature, depth;
MotorFeedback motor_feedback;

// 硬件初始化函数
void hardware_init() {
    // 初始化I2C总线
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(myiic_bus_init(&bus_handle));

    // 初始化MPU6050
    ESP_ERROR_CHECK(MPU6050_Init(&mpu6050, bus_handle));
    Kalman_Init();

    // 初始化MS5837
    ESP_ERROR_CHECK(ms5837_init(&ms5837, bus_handle));

    // 初始化PCA9685
    ESP_ERROR_CHECK(pca9685_init(&pca9685, bus_handle, 50)); // 50Hz PWM频率

    // 初始化电机PWM
    pwm_init();
    // 初始化电调
    initialize_motors();

    // 初始化电机反馈UART
    motor_uart_init();
   set_motor_pole_pairs_config(7); 
    // 初始化方向控制系统
    direction_control_init();
    
    // 初始化姿态控制器
    AttitudeController_Init(&attitude_ctrl, 0.02f); // 20ms控制周期
     vTaskDelay(pdMS_TO_TICKS(10)); // 10ms采样周期
}

// 获取传感器数据任务
void sensor_task(void *pvParameters) {
    while(1) {
        // 获取MPU6050数据并计算姿态
      
    
        Kalman_GetEuler_Angle(&current_roll, &current_pitch, &current_yaw);
        vTaskDelay(5); // 10ms采样周期
        // 获取MS5837数据
       ms5837_read_pressure_temperature(&ms5837, &pressure, &temperature);
       ms5837_calculate_depth(pressure, 1000.0f, &depth); // 假设水密度为1000kg/m³

        // 获取电机反馈数据
        if(read_motor_feedback(10) == 0) {
            motor_feedback = *get_motor_feedback();
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 10ms采样周期
    }
}

// 姿态控制任务
void attitude_control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms控制周期
    
    while(1) {
        // 更新姿态控制器
        AttitudeController_Update(&attitude_ctrl, current_roll, current_pitch, current_yaw);
        
        // 设置目标运动
        set_rov_direction(&target_motion, 0); // 立即生效
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief 设置ROV运动参数
 * @param power 总体功率百分比 (0-100)
 * @param forward 前进分量 (-100到100)
 * @param strafe 横向移动分量 (-100到100)
 * @param vertical 垂直移动分量 (-100到100)
 * @return int 0:成功 -1:参数无效
 */
int set_rov_motion_params(float power, float forward, float strafe, float vertical) {
    // 参数有效性检查
    if (power < 0 || power > 100 || 
        forward < -100 || forward > 100 ||
        strafe < -100 || strafe > 100 ||
        vertical < -100 || vertical > 100) {
        ESP_LOGE(TAG, "Invalid motion parameters: power=%.1f, forward=%.1f, strafe=%.1f, vertical=%.1f",
                power, forward, strafe, vertical);
        return -1;
    }
    
    // 更新目标运动参数
    target_motion.power = power;
    target_motion.forward = forward;
    target_motion.strafe = strafe;
    target_motion.vertical = vertical;

         set_rov_direction(&target_motion, 0); // 立即生效  

   // ESP_LOGI(TAG, "Motion params updated: power=%.1f%%, forward=%.1f, strafe=%.1f, vertical=%.1f",
   //         power, forward, strafe, vertical);
    return 0;
}

// 主控制任务
void control_task(void *pvParameters) {
   
    while(1) { // 必须添加无限循环
        float custom_power = 50.0f;
        float custom_forward = 50.0f;
        float custom_strafe = 00.0f;
        float custom_vertical = 50.0f;
 
        if(set_rov_motion_params(custom_power, custom_forward, 
                               custom_strafe, custom_vertical) == 0) {
            //ESP_LOGI(TAG, "Control task running...");
        }
         vTaskDelay(pdMS_TO_TICKS(10)); // 10ms采样周期
    }
   
  
    
 
}

// 打印状态任务（用于调试）
void print_task(void *pvParameters) {
    while(1) {
        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
                current_roll, current_pitch, current_yaw);
        ESP_LOGI(TAG, "Pressure: %.2f mbar, Temp: %.2f C, Depth: %.2f m", 
               pressure, temperature, depth);
        ESP_LOGI(TAG, "Motor RPM: %.2f, Current: %.2f A", 
                motor_feedback.actual_rpm, motor_feedback.current);
        
        vTaskDelay(pdMS_TO_TICKS(500)); // 1秒打印一次
    }
}


void app_main(void) {
    // 初始化日志系统
  

    // 初始化硬件
    hardware_init();
       
    ESP_LOGI(TAG, "Hardware initialized successfully");

    // 创建任务
    if (xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor_task");
    }
     if (xTaskCreate(control_task, "control_task", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control_task");
    }   
    if (xTaskCreate(print_task, "print_task", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create print_task");
    }

    ESP_LOGI(TAG, "All tasks created successfully");

}