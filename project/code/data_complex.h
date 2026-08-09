#ifndef _DATA_COMPLEX_H
#define _DATA_COMPLEX_H

#include "zf_common_headfile.h"
#include "image_process.h"

// ================= 板间通讯模式编译期开关 =================
// DUPLEX_SWITCH: 板间通讯模式总开关 (编译期生效)
//   1 = 启用双向 duplex_comm 主从请求-应答通讯 (无人机=主机, 小车=从机)
//   0 = 回退到原单向发送 (无人机 → 小车, 仅 Board_Comm_Init/Board_Comm_Send_Data)
// 回退用途: 双向链路异常或联调对比时改为 0 重新编译, 即可退回已验证的原单向逻辑。
// 注意: 本开关必须与小车端 car_board_comm.h 的 DUPLEX_SWITCH 保持一致, 否则帧格式不匹配。
#define DUPLEX_SWITCH 1

// ================= 板间通讯硬件配置 (发送端: UART4 → 小车UART1) =================
#define BOARD_UART       UART_4          
#define BOARD_BAUDRATE   1000000
#define BOARD_TX_PIN     UART4_TX_P14_1  
#define BOARD_RX_PIN     UART4_RX_P14_0  
#define UART_DATA_LENGTH 12 // 下传数组长度 (12个float)
// 协议帧格式: 0xAA 0x55 + 48字节(12×float) + 1字节累加校验和 + 0x7F
// 数组索引映射详见 data_complex.c 中 Float_Buffer_write()，小车端对应 car_board_comm.h
//============================================================
#define M7_x_DATA_LENGTH 20

// ================= share_data_from_0[20] 索引定义 (Core 0 → Core 1) =================
// 由 M7_0_data_send() 写入，Core 1 只读
#define S0_IMU_ROLL       0   // imu_data.roll              横滚角 (deg)
#define S0_IMU_PITCH      1   // imu_data.pitch             俯仰角 (deg)
#define S0_IMU_YAW        2   // imu_data.yaw               偏航角 (deg)
#define S0_IMU_HEIGHT     3   // imu_data.z                 高度 (cm)
#define S0_DRONE_STATE    4   // current_drone_state        飞行模式/状态
#define S0_MOTOR_LF       5   // motor_out.lf               左前电机PWM
#define S0_MOTOR_RF       6   // motor_out.rf               右前电机PWM
#define S0_MOTOR_LB       7   // motor_out.lb               左后电机PWM
#define S0_MOTOR_RB       8   // motor_out.rb               右后电机PWM
#define S0_TARGET_ROLL    9   // flight_target.target_roll  目标横滚角 (deg)
#define S0_TARGET_PITCH   10  // flight_target.target_pitch 目标俯仰角 (deg)
#define S0_TARGET_YAW     11  // flight_target.target_yaw   目标偏航角 (deg)
#define S0_DEBUG_ERR_X    12  // dataC.debug_earth_err_x    调试: 地面误差X
#define S0_DEBUG_ERR_Y    13  // dataC.debug_earth_err_y    调试: 地面误差Y
// 14-19 reserved

// ================= share_data_from_1[20] 索引定义 (Core 1 → Core 0) =================
// 由 M7_1_data_send() 写入，Core 0 只读 (Core 0 不再写回该区域，见 S1_FRAME_SEQ)
// 握手协议: Core1 先写全部数据, 最后写 S1_FRAME_SEQ 递增序号并 CleanDCache;
// Core0 读序号→整帧拷贝到 vision_snap→复核序号(防新旧帧撕裂)→消费快照。
#define S1_CAR_CENTER_Y    0   // cam_down.car_center_y      小车中心Y坐标 (像素)
#define S1_CAR_CENTER_X    1   // cam_down.car_center_x      小车中心X坐标 (像素)
#define S1_CAR_DOT_NUM     2   // cam_down.car_dot_num       小车识别点数/面积
#define S1_CAR_RAW_X       3   // pos.raw_car.x              小车未滤波位置X (cm)
#define S1_CAR_RAW_Y       4   // pos.raw_car.y              小车未滤波位置Y (cm)
#define S1_TARGET_X        5   // pos.k_target.x             目标(信标)Kalman位置X (cm)
#define S1_TARGET_Y        6   // pos.k_target.y             目标(信标)Kalman位置Y (cm)
#define S1_RAW_TARGET_X    7   // pos.raw_target[0].x        主信标未滤波位置X (cm)
#define S1_SNAPSHOT_YAW    8   // img_imu_snap.yaw           快照偏航角 (deg)
#define S1_K_CAR_X         9   // pos.k_car.x                卡尔曼滤波后小车X (cm)
#define S1_RAW_TARGET_Y    10  // pos.raw_target[0].y        主信标未滤波位置Y (cm)
// 11 reserved
#define S1_K_CAR_Y         12  // pos.k_car.y                卡尔曼滤波后小车Y (cm)
#define S1_CAR_TARGET_DIST 13  // dataC.car_target_dist      小车-信标距离
#define S1_LOCKED_COUNT    14  // locked_state               0=全丢, 1=仅小车, 2=仅信标, 3=都有
#define S1_FRAME_SEQ       15  // 帧序号 (float 单调递增, 数据写完最后写入; 到2^23回绕)
#define S1_RAW_TARGET2_X   16  // pos.raw_target[1].x        第二信标未滤波位置X (cm)
#define S1_RAW_TARGET2_Y   17  // pos.raw_target[1].y        第二信标未滤波位置Y (cm)
#define S1_RAW_TARGET3_X   18  // pos.raw_target[2].x        第三信标未滤波位置X (cm)
#define S1_RAW_TARGET3_Y   19  // pos.raw_target[2].y        第三信标未滤波位置Y (cm)

// ================= 高度保护参数 =================
#define VISION_POSITION_MIN_HEIGHT_CM       35.0f // 低于此高度才让视觉坐标失效
#define VISION_LOW_HEIGHT_HOLD_FRAMES       5U    // 图像约50Hz，5帧约100ms
#define CAR_ENABLE_MIN_HEIGHT_CM            90.0f // 低于此高度下传car_en=0

// ================= 视觉失联保护 =================
// 基于 dataC.pit0_cnt (1ms) 的时间计数，取代依赖主循环负载的循环计数
#define VISION_LOST_TIMEOUT_MS              500U  // 超过此毫秒未收到新视觉帧 → 回平悬停保护

typedef struct {
    float debug_earth_err_x,debug_earth_err_y;
    float debug_body_track_x,debug_body_track_y;
    float car_target_dist; //1
    uint32_t pit0_cnt; //0
    float camera_offset_x,camera_offset_y; //0
}Data_Complex_t;

extern Data_Complex_t dataC;
extern volatile float share_data_from_0[M7_x_DATA_LENGTH];
extern volatile float share_data_from_1[M7_x_DATA_LENGTH];
extern volatile float vision_snap[M7_x_DATA_LENGTH];  // Core0 侧: 视觉帧一致性快照 (仅 CM7_0 构建定义)
extern uint8_t car_en;
extern uint8_t car_en_height;

// ================= 函数声明 =================
void M7_0_data_send(volatile float* data_out);
void Float_Buffer_write(float* buffer, volatile float* share);
void M7_1_data_send(volatile float* data_out);

void Board_Comm_Init(void);
void Board_Comm_Send_Data(volatile float *data_array);
#endif
