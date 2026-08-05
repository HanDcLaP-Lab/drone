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
void wireless_uart_output_imu_sample_rate(void);
void wireless_uart_output_height(void);
void wireless_uart_output_beacon_brightness(void);
#endif
