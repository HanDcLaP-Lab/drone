#ifndef _WIRELESS_UART_H
#define _WIRELESS_UART_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>

void wireless_uart_init_();
void wireless_uart_get_();
void wireless_uart_send_int(int32_t send_a);
void wireless_uart_send_float(float send_a);
void wireless_uart_output_status(void);
void wireless_uart_output_imu(void);
void wireless_uart_output_pid(void);
void wireless_uart_output_yaw(void);
void wireless_uart_output_motor(void);
void wireless_uart_motor_average_sample(void); // PIT_CH1中motor_pwm_set()之后调用
void wireless_uart_output_motor_average(void); // CM7_0主循环调用
void wireless_uart_output_groud(void);
void wireless_uart_output_car_target_dist(void); // 打印小车-信标地面距离 (cm)
void wireless_uart_output_imu_sample_rate(void);
void wireless_uart_output_height(void);
void wireless_uart_output_feedforward_recv(void); // [新增] 刚收到原始前馈角时立刻打印一次 (FFD_RECV)
void wireless_uart_output_feedforward_rx(void);   // [新增] 确认采纳前馈事件时打印一次 (FFD_ACK)
void wireless_uart_output_driver_status(void);  // [新增] 打印驱动板电压/电流/温度 (无线)
#endif
