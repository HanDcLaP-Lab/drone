#ifndef _DATA_COMPLEX_H
#define _DATA_COMPLEX_H

#include "zf_common_headfile.h"
#include "image_process.h"

// ================= 板间通讯硬件配置 (发送端) =================
#define BOARD_UART       UART_4          
#define BOARD_BAUDRATE   115200          
#define BOARD_TX_PIN     UART4_TX_P14_1  
#define BOARD_RX_PIN     UART4_RX_P14_0  
#define UART_DATA_LENGTH 8  // 数组数据长度
//============================================================
#define M7_x_DATA_LENGTH 16

typedef struct {
    float debug_earth_err_x,debug_earth_err_y; //0
    float car_target_dist; //1
    uint32_t pit0_cnt; //0
    float camera_offset_x,camera_offset_y; //0
}Data_Complex_t;

extern Data_Complex_t dataC;
extern volatile float share_data_from_0[M7_x_DATA_LENGTH];
extern volatile float share_data_from_1[M7_x_DATA_LENGTH];

// ================= 函数声明 =================
void M7_0_data_send(volatile float* data_out);
void Float_Buffer_write(float* buffer, volatile float* share);
void M7_1_data_send(volatile float* data_out);

void Board_Comm_Init(void);
void Board_Comm_Send_Data(volatile float *data_array);
#endif