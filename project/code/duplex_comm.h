/*********************************************************************************************************************
 * duplex_comm.h  无人机端(主机) 双向板间通讯模块接口
 *
 * 角色: 无人机 = 主机(Master)。无人机每帧视觉处理完成时触发一次「请求-应答」:
 *       主机经 UART4(RS485 半双工) 发出 CMD_MASTER 请求帧(载荷=下传数据),
 *       小车(从机)收到后回 CMD_SLAVE 应答帧(载荷=小车上传数据), 主机接收并缓存。
 *
 * 触发时机: 由主循环在视觉事件分支调用 Duplex_Comm_Trigger() 发起(不使用固定周期)。
 * 收发处理: 接收中断 -> Duplex_Comm_On_Uart_Rx() 入 FIFO; 主循环 Duplex_Comm_Poll()
 *           排空解析应答并做超时判丢包。每次触发至多发一帧, 超时(< 通讯周期)判丢包。
 *
 * 编译期开关 DUPLEX_SWITCH(定义于 data_complex.h): 1=启用本模块双向通讯, 0=回退原单向发送。
 * 协议帧与编解码已并入本文件(.h/.c, 原 board_float8_frame.* ), 与小车端/参考案例字节级一致。
 ********************************************************************************************************************/
#ifndef _DUPLEX_COMM_H_
#define _DUPLEX_COMM_H_

#include "zf_common_headfile.h"
#include <stdint.h>

// ================= 协议帧定义 (原 board_float8_frame.h, 已并入本模块) =================
// 帧布局 (共 38 字节): 0xAA 0x55 + cmd + seq + 8×float(小端) + 累加校验(cmd+seq+32数据 模256) + 0x7F
#define BOARD_FLOAT8_COUNT        8u                                          // 数据载荷 float 个数
#define BOARD_FLOAT8_DATA_BYTES   (BOARD_FLOAT8_COUNT * 4u)                   // 数据区字节数 = 32
#define BOARD_FLOAT8_FRAME_SIZE   (2u + 1u + 1u + BOARD_FLOAT8_DATA_BYTES + 1u + 1u) // 整帧字节数 = 38

#define BOARD_FLOAT8_CMD_MASTER   0x10u   // 命令字: 主机(无人机)请求帧
#define BOARD_FLOAT8_CMD_SLAVE    0x20u   // 命令字: 从机(小车)应答帧

// 解码后的帧内容 (帧头/帧尾/校验已剥离, 仅保留有效字段)
typedef struct {
    uint8_t cmd;                      // 命令字 (CMD_MASTER / CMD_SLAVE)
    uint8_t seq;                      // 序列号
    float data[BOARD_FLOAT8_COUNT];   // 8 个 float 数据载荷
} board_float8_frame_t;

void Board_Float8_Frame_Encode(uint8_t cmd, uint8_t seq, const float data[BOARD_FLOAT8_COUNT], uint8_t frame[BOARD_FLOAT8_FRAME_SIZE]);
uint8_t Board_Float8_Frame_Decode(const uint8_t frame[BOARD_FLOAT8_FRAME_SIZE], board_float8_frame_t *out);

// ================= 硬件配置 (Master = 无人机, UART4, 与参考案例一致) =================
#define DUPLEX_UART          UART_4            // 板间通讯串口
#define DUPLEX_TX_PIN        UART4_TX_P14_1    // 发送引脚
#define DUPLEX_RX_PIN        UART4_RX_P14_0    // 接收引脚
#define DUPLEX_DIR_PIN       P19_2             // RS485 方向引脚 (高=发送, 低=接收)
#define DUPLEX_BAUDRATE      115200            // 波特率

// ================= 角色与协议 (主机) =================
#define DUPLEX_SELF_CMD      BOARD_FLOAT8_CMD_MASTER   // 主机发送命令字 0x10
#define DUPLEX_PEER_CMD      BOARD_FLOAT8_CMD_SLAVE    // 主机接收命令字 0x20

// ================= 缓冲与时序配置 =================
#define DUPLEX_RX_FIFO_SIZE      128u    // 接收 FIFO 容量 (字节)
#define DUPLEX_TX_HOLD_US        200u    // 发送后等待移位完成的 DE 保持时间 (us)
#define DUPLEX_DIR_SETUP_US      20u     // 方向切到发送态后的建立时间 (us)
#define DUPLEX_REPLY_TIMEOUT_MS  16u     // 应答超时 (须 < 通讯周期, 视觉约 20ms)

// ================= 对外状态 / 计数器 =================
extern float    duplex_uplink_data[BOARD_FLOAT8_COUNT]; // 小车上传(占位)接收缓冲
extern volatile uint8_t  duplex_uplink_update_flag;     // 收到有效应答置 1
extern volatile uint32_t duplex_request_count;          // 已发起请求数
extern volatile uint32_t duplex_reply_ok_count;         // 收到有效应答数
extern volatile uint32_t duplex_timeout_count;          // 超时(丢包)数
extern volatile uint32_t duplex_decode_fail_count;      // 解码失败数
extern volatile uint32_t duplex_cmd_mismatch_count;     // 命令字不匹配数
extern volatile uint32_t duplex_rx_fifo_drop_count;     // 接收 FIFO 溢出数
extern volatile uint8_t  duplex_last_seq;               // 最近请求 seq

// 最近一次通讯失败的原因码 (供调试打印 wireless_uart_output_duplex 使用)
//   0 = 无错误(最近一次成功收到应答)   1 = 解码失败(帧头/帧尾/校验)
//   2 = 命令字不匹配   3 = 应答超时   4 = 接收 FIFO 溢出
#define DUPLEX_ERR_NONE     0u
#define DUPLEX_ERR_DECODE   1u
#define DUPLEX_ERR_CMD      2u
#define DUPLEX_ERR_TIMEOUT  3u
#define DUPLEX_ERR_FIFO     4u
extern volatile uint8_t  duplex_last_err;               // 最近一次失败原因码

// ================= 对外接口 =================
void Duplex_Comm_Init(void);                       // UART/方向引脚/FIFO 初始化, 进入接收态
void Duplex_Comm_On_Uart_Rx(void);                 // uart4_isr 接收分支调用: 收字节入 FIFO
void Duplex_Comm_Poll(void);                       // 主循环调用: 排空 FIFO 解析应答 + 超时判定
void Duplex_Comm_Trigger(const float downlink[BOARD_FLOAT8_COUNT]); // 视觉事件调用: 发起一次请求
void Duplex_Comm_Set_Now_Ms(uint32_t ms);          // 喂入毫秒时基 (来自 dataC.pit0_cnt)
const float* Duplex_Comm_Get_Uplink(void);         // 读取上传数据缓冲
void Duplex_Comm_Reset_Stats(void);                // 清零收发统计计数 (启动稳定后调用)

#endif
