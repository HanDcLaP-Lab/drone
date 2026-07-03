#ifndef _DATA_COMPLEX_H
#define _DATA_COMPLEX_H

#include "zf_common_headfile.h"
#include "image_process.h"

// ================= 板间通讯硬件配置 (发送端: UART4 → 小车UART1) =================
#define BOARD_UART       UART_4          
#define BOARD_BAUDRATE   1000000
#define BOARD_TX_PIN     UART4_TX_P14_1  
#define BOARD_RX_PIN     UART4_RX_P14_0  
#define UART_DATA_LENGTH 8  // 下传数组长度 (8个float)
// 协议帧格式: 0xAA 0x55 + 32字节(8×float) + 1字节累加校验和 + 0x7F
// 数组索引映射详见 data_complex.c 中 Float_Buffer_write()，小车端对应 car_board_comm.h
//============================================================
#define M7_x_DATA_LENGTH 16

// ================= share_data_from_0[16] 索引定义 (Core 0 → Core 1) =================
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
// 14-15 reserved

// ================= share_data_from_1[16] 索引定义 (Core 1 → Core 0) =================
// 由 M7_1_data_send() 写入，Core 0 只读
#define S1_CAR_CENTER_Y    0   // cam_down.car_center_y      小车中心Y坐标 (像素)
#define S1_CAR_CENTER_X    1   // cam_down.car_center_x      小车中心X坐标 (像素)
#define S1_CAR_DOT_NUM     2   // cam_down.car_dot_num       小车识别点数/面积
#define S1_CAR_RAW_X       3   // pos.car.x                  小车原始位置X (cm)
#define S1_CAR_RAW_Y       4   // pos.car.y                  小车原始位置Y (cm)
#define S1_TARGET_X        5   // pos.target.x               目标(信标)位置X (cm)
#define S1_TARGET_Y        6   // pos.target.y               目标(信标)位置Y (cm)
#define S1_RAW_CAR_X       7   // pos.raw_car.x              小车未滤波X (cm)
#define S1_SNAPSHOT_YAW    8   // img_imu_snap.yaw           快照偏航角 (deg)
#define S1_K_CAR_X         9   // pos.k_car.x                卡尔曼滤波后小车X (cm)
// 10-11 reserved
#define S1_K_CAR_Y         12  // pos.k_car.y                卡尔曼滤波后小车Y (cm)
#define S1_CAR_TARGET_DIST 13  // dataC.car_target_dist      小车-信标距离
#define S1_LOCKED_COUNT    14  // locked_state               0=全丢, 1=仅小车, 2=仅信标, 3=都有, 4=近距离融合盲冲
#define S1_PROCESS_DONE    15  // 图像处理完成标志 (1.0=完成, 0.0=未完成)

typedef struct {
    float debug_earth_err_x,debug_earth_err_y; //0
    float car_target_dist; //1
    uint32_t pit0_cnt; //0
    float camera_offset_x,camera_offset_y; //0
}Data_Complex_t;

extern Data_Complex_t dataC;
extern volatile float share_data_from_0[M7_x_DATA_LENGTH];
extern volatile float share_data_from_1[M7_x_DATA_LENGTH];
extern uint8_t car_en;

// ================= 函数声明 =================
void M7_0_data_send(volatile float* data_out);
void Float_Buffer_write(float* buffer, volatile float* share);
void M7_1_data_send(volatile float* data_out);

void Board_Comm_Init(void);
void Board_Comm_Send_Data(volatile float *data_array);
#endif
