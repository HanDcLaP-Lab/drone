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
void wireless_uart_output_groud(void);
void wireless_uart_output_height(void);
void wireless_uart_output_duplex(void);   // 板间双向通讯收发统计: 成功数,失败数,原因码
#endif