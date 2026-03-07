#include "board_comm.h"
#include "zf_common_headfile.h"

// ================= 板间通讯协议配置 =================
#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL    0x7F

// 发送缓冲区：2字节帧头 + 32字节(8个float) + 1字节校验和 + 1字节帧尾 = 36字节
static uint8_t send_buffer[36]; 

// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    // 初始化配置好的串口
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
}

// ================= 打包并发送函数 =================
// 只要传入长度为 8 的 float 数组首地址即可
void Board_Comm_Send_Data(volatile float *data_array)
{
    // 1. 填入帧头
    send_buffer[0] = FRAME_HEADER1;
    send_buffer[1] = FRAME_HEADER2;
    
    // 2. 将传入的 float 数组按字节拷贝到发送缓冲区 (强转为void*以消除volatile警告)
    memcpy(&send_buffer[2], (void *)data_array, sizeof(float) * 8);
    
    // 3. 计算简单的累加校验和 (只校验数据区的32字节)
    uint8_t checksum = 0;
    for (int i = 2; i < 34; i++) {
        checksum += send_buffer[i];
    }
    
    // 4. 填入校验和与帧尾
    send_buffer[34] = checksum;
    send_buffer[35] = FRAME_TAIL;
    
    // 5. 物理发送整包数据 (36字节)
    uart_write_buffer(BOARD_UART, send_buffer, sizeof(send_buffer));
}