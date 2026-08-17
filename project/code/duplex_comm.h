#ifndef _DUPLEX_COMM_H
#define _DUPLEX_COMM_H

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * duplex_comm  无人机端(主机) 双向板间通讯
 *
 * 角色: 无人机 = 主机(Master)。每个视觉帧完成时触发一次「请求-应答」:
 *       主机经 UART4(RS485 半双工) 发 CMD_MASTER 请求帧(载荷 = 12 float 下传数据),
 *       小车(从机)收到后回 CMD_SLAVE 应答帧(载荷 = 3 float 上传数据), 主机接收并缓存。
 *
 * 触发时机: 主循环视觉事件分支调用 Duplex_Comm_Trigger() 发起, 不使用固定周期。
 * 收发处理: uart4_isr 调用 Duplex_Comm_On_Uart_Rx() 收字节入 FIFO;
 *           主循环 Duplex_Comm_Poll() 排空 FIFO 解析应答并做超时判丢包。
 *
 * 编译期开关 DUPLEX_SWITCH (定义于 data_complex.h): 1=双向, 0=回退原单向发送。
 *
 * 下传载荷索引映射与原单向协议完全一致, 见 data_complex.c 的 Float_Buffer_write()。
 ********************************************************************************************************************/

// ================= 协议帧定义 =================
// 帧布局: 0xAA 0x55 + cmd + seq + N×float(小端) + 累加校验(cmd+seq+数据区 模256) + 0x7F
// 下行(主机→从机) N = DUPLEX_DOWNLINK_COUNT = 13, 整帧 58 字节
// 上行(从机→主机) N = DUPLEX_UPLINK_COUNT   =  4, 整帧 22 字节
// 两个方向帧长不同, 靠 cmd 区分; 各自状态机只按本方向的帧长累积。
#define DUPLEX_DOWNLINK_COUNT     UART_DATA_LENGTH   // 下行 float 个数 (13, 与原协议一致)
#define DUPLEX_UPLINK_COUNT       4U                 // 上行 float 个数 (IMU roll/pitch/yaw + 前馈角)

#define DUPLEX_FLOAT_BYTES        4U
#define DUPLEX_HEADER1            0xAAu
#define DUPLEX_HEADER2            0x55u
#define DUPLEX_TAIL               0x7Fu

// 帧内偏移 (两个方向共用): [0]=0xAA [1]=0x55 [2]=cmd [3]=seq [4..]=data
#define DUPLEX_CMD_OFFSET         2U
#define DUPLEX_SEQ_OFFSET         3U
#define DUPLEX_DATA_OFFSET        4U

// 按 float 个数换算整帧长度: 帧头2 + cmd1 + seq1 + 数据区 + 校验1 + 帧尾1
#define DUPLEX_FRAME_SIZE(n)      (DUPLEX_DATA_OFFSET + (n) * DUPLEX_FLOAT_BYTES + 2U)
#define DUPLEX_DOWNLINK_FRAME_SIZE  DUPLEX_FRAME_SIZE(DUPLEX_DOWNLINK_COUNT)   // 58
#define DUPLEX_UPLINK_FRAME_SIZE    DUPLEX_FRAME_SIZE(DUPLEX_UPLINK_COUNT)     // 22

#define DUPLEX_CMD_MASTER         0x10u   // 命令字: 主机(无人机)请求帧
#define DUPLEX_CMD_SLAVE          0x20u   // 命令字: 从机(小车)应答帧

// 主机视角: 自己发 MASTER, 期望收 SLAVE
#define DUPLEX_SELF_CMD           DUPLEX_CMD_MASTER
#define DUPLEX_PEER_CMD           DUPLEX_CMD_SLAVE

// ================= 硬件配置 =================
// UART4 与原单向发送同一串口/引脚 (BOARD_UART/BOARD_TX_PIN/BOARD_RX_PIN, 见 data_complex.h)。
// P19_2 原为 main_cm7_0.c 的 UART_KEY(初始化为高), 双向模式下作 RS485 方向引脚: 高=发送, 低=接收。
#define DUPLEX_DIR_PIN            P19_2

// ================= 时序配置 =================
// DE 建立/保持时间。
//
// [修复] TX_HOLD 曾设为 200us, 理由是"末字节可能仍在 FIFO/移位寄存器内, 过早拉低 DE
//   会截断尾字节"。该理由不成立: uart_write_byte 每字节都忙等 Cy_SCB_IsTxComplete(),
//   而该函数的语义是「TX FIFO 与移位寄存器双双为空」(见 cy_scb_common.h:667
//   "Checks if the TX FIFO and Shifter are empty"), 所以 uart_write_buffer 返回时
//   最后一位已经完整发上总线, 无需任何额外保持。
//
//   而这 200us 的白等有实际危害: 它让无人机的 DE 在发送完成后仍多占总线 200us,
//   与小车的应答发送窗口重叠。小车主循环解析到完整请求后立刻回复, 若该时刻落在这
//   200us 内, 两端驱动器同时驱动差分对, 且无人机接收器此时仍关闭 ——
//   应答开头整段被吞掉(实测约 16 字节), 接收器使能后从帧中途开始收, 位边界错乱。
//   实测特征: ld=0 (前导一个都没正确解出) + j 中出现可复现的错位字节对(91BF/75D5)。
//
//   改为 20us: 仅留应对时钟抖动与收发器输入到输出传播延迟的余量 (@1Mbps 相当于 2 字节时间),
//   总线占用窗口从 200us 压到 20us, 与应答重叠的概率大幅降低。
#define DUPLEX_DIR_SETUP_US       20U
#define DUPLEX_TX_HOLD_US         20U

// 应答超时。@1Mbps 下行 58B≈0.58ms + 上行 22B≈0.22ms + DE 切换 0.2ms ≈ 1ms,
// 实测典型往返 1~3ms; 但主循环里的 printf 是阻塞式的 (每字节忙等 TxComplete),
// 一行调试信息就能占住主循环数毫秒, 期间 Poll 不执行 → 应答被推迟处理 → rtt 被顶高。
// 实测 max_rtt 曾达 12ms, 正是这个原因 (printf 默认关闭后 rtt 回到毫秒级)。
//
// 100Hz 摄像头下视觉周期为 10ms, 取 8ms 的依据: 需同时满足
//   ① 大于典型 RTT (1~3ms) 与主循环调度抖动, 避免正常应答被误判成丢包;
//   ② 小于视觉周期 (10ms), 保证下一次 Trigger 到来前 Poll 已完成超时判定,
//      否则 Trigger 会抢先把"上一次请求"判超时, 且迟到的旧应答再记一次 seq 不符 → 双计数。
// 另有 seq 匹配作为第二道保险 (见 Duplex_Process_Full_Frame): 迟到的应答即便跨过了超时线,
// 只要 seq 与最近一次请求不符就不会被错认成本次应答, 因此阈值可适度放宽到 8ms。
#define DUPLEX_REPLY_TIMEOUT_MS   8U

#define DUPLEX_RX_FIFO_SIZE       128U    // 接收 FIFO 容量 (字节), 远大于单帧 22B

// 调试打印周期, 由 Duplex_Comm_Print_Stats 内部限频。
// printf 是阻塞式的 (每字节忙等 TxComplete), 一行约 90 字节 @115200 要占住主循环约 7.8ms,
// 期间 Poll/Trigger 都不执行 → 应答处理被推迟 → rtt 被顶高甚至误判超时。
// 联调期取 5000ms: 把这个扰动摊薄到几乎不影响统计, 同时仍能持续观察链路质量。
#define DUPLEX_PRINT_PERIOD_MS    5000U

// ================= 失败原因码 =================
#define DUPLEX_ERR_NONE           0U   // 最近一次成功收到应答
#define DUPLEX_ERR_DECODE         1U   // 解码失败 (帧尾或校验和)
#define DUPLEX_ERR_CMD            2U   // 命令字不匹配 (含自身请求帧回环)
#define DUPLEX_ERR_TIMEOUT        3U   // 应答超时
#define DUPLEX_ERR_FIFO           4U   // 接收 FIFO 溢出
#define DUPLEX_ERR_SEQ            5U   // seq 与最近一次请求不符 (迟到的旧应答)

// ================= 解码结果 =================
typedef struct {
    uint8_t cmd;                            // 命令字
    uint8_t seq;                            // 序列号
    float   data[DUPLEX_UPLINK_COUNT];      // 上行载荷
} duplex_uplink_frame_t;

// ================= 对外状态 / 统计 =================
// duplex_uplink_data 索引映射 (小车→无人机上传协议, 应答帧载荷):
//   [0] imu roll  — 小车横滚角 (deg)
//   [1] imu pitch — 小车俯仰角 (deg)
//   [2] imu yaw   — 小车偏航角 (deg)
//   [3] ff_deg    — 小车前馈方向角 (deg, -1=无前馈, 0°为有效方向) [新增, 已接入飞控, 供无线串口打印观察]
extern float duplex_uplink_data[DUPLEX_UPLINK_COUNT];
// 前馈采纳反馈: 1 = 最近一次成功解码的应答载荷 [3] 已被飞控采纳, 0 = 未采纳 (含收到-1/无应答)。
// 由 Duplex_Process_Full_Frame 与 Car_Position_Predict_Feedforward 协同刷新, 下传帧 [12] 反馈标志据此生成。
extern float duplex_ff_pending_deg;             // [新增] 最近收到但尚未被飞控采纳的有效前馈角 (deg, -1=无)
extern volatile uint8_t duplex_ff_deg_received;
extern volatile uint8_t  duplex_uplink_update_flag;   // 收到有效应答置 1 (由消费方清零)
extern volatile uint32_t duplex_request_count;        // 已发起请求数
extern volatile uint32_t duplex_reply_ok_count;       // 收到有效应答数
extern volatile uint32_t duplex_timeout_count;        // 超时(丢包)数
extern volatile uint32_t duplex_decode_fail_count;    // 解码失败数
extern volatile uint32_t duplex_cmd_mismatch_count;   // 命令字不匹配数
extern volatile uint32_t duplex_rx_fifo_drop_count;   // 接收 FIFO 溢出数
extern volatile uint32_t duplex_seq_mismatch_count;   // seq 不符被丢弃的应答数 (迟到的旧应答)
extern volatile uint32_t duplex_last_rtt_ms;          // 最近一次往返时延 (ms)
extern volatile uint32_t duplex_max_rtt_ms;           // 往返时延最大值 (ms)
extern volatile uint8_t  duplex_last_seq;             // 最近请求 seq
extern volatile uint8_t  duplex_last_err;             // 最近一次失败原因码


// 说明: 8.9a 调试期曾另有一组深度诊断埋点 (UART 中断进入次数、原始字节数、SCB 硬件 RX FIFO
//   溢出次数、HEADER 状态丢弃字节数与连续丢弃形态快照、DE 引脚回读、上电首字节留存),
//   用于把"上行收不到应答"逐层切分到硬件层/帧同步层。链路定位完成后已移除, 详见 README 8.9a。
//   若日后上行再度异常, 从 8.9a 提交取回这些埋点即可复现当时的诊断手段。

// ================= 对外接口 =================
void Duplex_Comm_Init(void);          // UART/方向引脚/FIFO 初始化, 进入接收态
void Duplex_Comm_On_Uart_Rx(void);    // uart4_isr 接收分支调用: 收字节入 FIFO
void Duplex_Comm_Poll(void);          // 主循环调用: 排空 FIFO 解析应答 + 超时判定
void Duplex_Comm_Trigger(const float *downlink);   // 视觉事件调用: 发起一次请求
void Duplex_Comm_Set_Now_Ms(uint32_t ms);          // 喂入毫秒时基 (dataC.pit0_cnt)
void Duplex_Comm_Reset_Stats(void);                // 清零收发统计 (启动稳定后调用)
void Duplex_Comm_Print_Stats(void);                // 有线 printf 输出通讯质量 (CM7_0主循环调用, 内部限频)

#endif
