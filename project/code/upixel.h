#ifndef _UPIXELS_H_
#define _UPIXELS_H_

#include "zf_common_headfile.h"

#define UPIXEL_UART UART_5 
#define UPIXEL_UART_TX UART5_TX_P02_1
#define UPIXEL_UART_RX UART5_RX_P02_0

// ================= 光流解算配置 =================
#define OPT_MIN_VALID_HEIGHT_CM   80.0f  // 门限保护: 高度低于80cm或valid=0时主动归零并复位滤波
#define OPT_GYRO_COMP_ENABLE      1      // 1: 启用陀螺仪角速度解耦补偿 (消除纯旋转产生的假速度)
#define OPT_TILT_COMP_ENABLE      1      // 1: 启用机体倾角高度修正
#define OPT_GYRO_SIGN_X           1.0f   // X轴陀螺补偿符号 (对应 Pitch, 若补偿反向可改为 -1.0f)
#define OPT_GYRO_SIGN_Y           1.0f   // Y轴陀螺补偿符号 (对应 Roll, 若补偿反向可改为 -1.0f)

// 光流输出数据结构体，参考官方协议定义[cite: 1]
typedef struct 
{
    int16   flow_x_integral;        // X像素点累计时间内的累加位移 (radians*10000)
    int16   flow_y_integral;        // Y像素点累计时间内的累加位移 (radians*10000)
    uint16  integration_timespan;   // 累计时间 (us)
    uint16  ground_distance;        // 预留，默认为999 (0x03E7)
    uint8   valid;                  // 状态值: 0(0x00)为不可用, 245(0xF5)为可用
    uint8   version;                // 版本号
    float opt_vel_x;                //移动速度，cm/s (原始)
    float opt_vel_y;                //移动速度，cm/s (原始)
    float filt_vel_x;               //移动速度，cm/s (低通滤波后)
    float filt_vel_y;               //移动速度，cm/s (低通滤波后)
} upixels_data_t;

// 外部引用的全局光流数据实例
extern upixels_data_t upixels_data;
extern volatile uint8_t upixels_frame_ready; // 完整帧解析完成标志位
extern volatile uint16_t upixels_frame_count; // 累计接收帧数
extern volatile uint16_t upixels_count_500ms; // 过去 500ms 内的更新次数

// 函数声明
void    upixels_init(void);
uint8   upixels_parse_byte(uint8 data);
void    upixels_calc_velocity(float current_height_cm);
void    upixels_poll_and_calc(float current_height_cm);
void    upixels_get_data_loop(void);

#endif