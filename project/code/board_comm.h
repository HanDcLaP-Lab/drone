#ifndef _BOARD_COMM_H
#define _BOARD_COMM_H

#include "zf_common_headfile.h"

// ================= 板间通讯硬件配置 (发送端) =================
#define BOARD_UART       UART_4          
#define BOARD_BAUDRATE   115200          
#define BOARD_TX_PIN     UART4_TX_P14_1  
#define BOARD_RX_PIN     UART4_RX_P14_0  
#define UART_DATA_LENGTH 8  // 数组数据长度
// ================= 函数声明 =================
void Board_Comm_Init(void);

// 传入 float 指针
void Board_Comm_Send_Data(volatile float *data_array);
#endif