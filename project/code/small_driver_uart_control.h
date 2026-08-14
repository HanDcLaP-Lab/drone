#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_2        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART2_RX_P10_0)

#define SMALL_DRIVER_TX                         (UART2_TX_P10_1)

// ================= 获取模式切换宏定义 =================
// 0 = 获取转速 (字节协议, 0xA5 帧, 4×int16 转速)
// 1 = 获取电压/电流/温度 (字符串协议)
#define SMALL_DRIVER_GET_SPEED                  0
#define SMALL_DRIVER_GET_VOLTAGE                1
#define SMALL_DRIVER_GET_MODE                   1   // 默认获取转速, 改为 SMALL_DRIVER_GET_VOLTAGE 则获取电压电流温度

// 字符串接收缓冲长度 (电压响应一行约 70 字节, 留余量)
#define SMALL_DRIVER_STRING_BUFFER_SIZE         80

// 字符串指令定义
#define SMALL_DRIVER_CMD_GET_VOLTAGE            "GET-VOLTAGE\r\n"

typedef struct
{
    uint8 send_data_buffer[11];                 // 发送缓冲数组

    uint8 receive_data_buffer[11];              // 接收缓冲数组 (字节协议)

    uint8 receive_data_count;                   // 接收计数 (字节协议)

    uint8 sum_check_data;                       // 校验位 (字节协议)

    int16 receive_speed_data[4];                // 接收到的电机速度数据

    uint8 speed_data_updated;                   // 1=新转速就绪(UART回调置位,IMU消费清零)

    uint8 receive_string_buffer[SMALL_DRIVER_STRING_BUFFER_SIZE];  // 字符串行接收缓冲

    uint8 receive_string_count;                 // 字符串行接收计数

    float battery_voltage;                      // 电池电压 (V)

    float bus_current;                          // 母线电流 (A)

    float board_temperature;                    // 板载温度 (℃)

    uint8 voltage_data_updated;                 // 1=新电压电流温度就绪(UART回调置位,消费清零)

}small_device_value_struct;

extern small_device_value_struct motor_value;



void uart_control_callback(void);                                   // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4);      // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息 (字节协议)

void small_driver_get_voltage(void);                                // 无刷驱动 获取电压电流温度信息 (字符串协议)

void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化

void driver_uart_test(void);


#endif
