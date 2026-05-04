#ifndef _UPIXELS_H_
#define _UPIXELS_H_

#include "zf_common_headfile.h"

#define UPIXEL_UART UART_3 
#define UPIXEL_UART_TX UART3_TX_P13_1
#define UPIXEL_UART_RX UART3_RX_P13_0

// 光流输出数据结构体，参考官方协议定义[cite: 1]
typedef struct 
{
    int16   flow_x_integral;        // X像素点累计时间内的累加位移 (radians*10000)
    int16   flow_y_integral;        // Y像素点累计时间内的累加位移 (radians*10000)
    uint16  integration_timespan;   // 累计时间 (us)
    uint16  ground_distance;        // 预留，默认为999 (0x03E7)
    uint8   valid;                  // 状态值: 0(0x00)为不可用, 245(0xF5)为可用
    uint8   version;                // 版本号
    float opt_vel_x;                //移动速度，cm/s
    float opt_vel_y;                //移动速度，cm/s
} upixels_data_t;

// 外部引用的全局光流数据实例
extern upixels_data_t upixels_data;

// 函数声明
void    upixels_init();
uint8   upixels_parse_byte(uint8 data);
void    upixels_get_data_loop(void);

#endif