#ifndef _UPIXELS_H_
#define _UPIXELS_H_

#include "zf_common_headfile.h"

#define UPIXEL_UART UART_5 
#define UPIXEL_UART_TX UART5_TX_P02_1
#define UPIXEL_UART_RX UART5_RX_P02_0

// ================= 光流与机体坐标系安装映射 =================
// 物理安装校准: 光流 Y+ 朝向飞机后方(机体X-), 光流 X+ 朝向飞机右方(机体Y+)
// 姿态角定义: Pitch 机头抬起为正(gpitch>0), Roll 左侧抬起(右倾)为正(groll>0)
#define OPT_MIN_VALID_HEIGHT_CM   0  // 门限保护: 高度低于落地关停高度或valid=0时主动归零并复位滤波
#define OPT_GYRO_COMP_ENABLE      1      // 1: 启用陀螺仪角速度解耦补偿 (消除纯旋转产生的假速度)
#define OPT_TILT_COMP_ENABLE      1      // 1: 启用机体倾角高度修正

// 符号微调系数
// 注意: 光流方程平移流 = -V/R (地面纹理相对相机朝机体运动反方向移动), 故需取负恢复实际速度。
// 2026-xx: 由 +1 改为 -1, 修复前馈中光流速度方向相反、由"补偿"变"助长"的问题。
#define OPT_SIGN_BODY_X           (-1.0f) // 机体前向速度符号
#define OPT_SIGN_BODY_Y           (-1.0f) // 机体右向速度符号
#define OPT_GYRO_SIGN_PITCH       (1.0f) // 俯仰角速度补偿符号
#define OPT_GYRO_SIGN_ROLL        (1.0f) // 横滚角速度补偿符号
// 一阶低通滤波去毛刺 (alpha 越小滤波越强)
#define FLOW_LPF_ALPHA  0.2f
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
uint8   upixels_poll_and_calc(float current_height_cm);
void    upixels_get_data_loop(void);

#endif