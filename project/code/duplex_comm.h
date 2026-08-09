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
// 下行(主机→从机) N = DUPLEX_DOWNLINK_COUNT = 12, 整帧 54 字节
// 上行(从机→主机) N = DUPLEX_UPLINK_COUNT   =  3, 整帧 18 字节
// 两个方向帧长不同, 靠 cmd 区分; 各自状态机只按本方向的帧长累积。
#define DUPLEX_DOWNLINK_COUNT     UART_DATA_LENGTH   // 下行 float 个数 (12, 与原协议一致)
#define DUPLEX_UPLINK_COUNT       3U                 // 上行 float 个数 (测试阶段: IMU roll/pitch/yaw)

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
#define DUPLEX_DOWNLINK_FRAME_SIZE  DUPLEX_FRAME_SIZE(DUPLEX_DOWNLINK_COUNT)   // 54
#define DUPLEX_UPLINK_FRAME_SIZE    DUPLEX_FRAME_SIZE(DUPLEX_UPLINK_COUNT)     // 18

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

// 应答超时。@1Mbps 下行 54B≈0.54ms + 上行 18B≈0.18ms + DE 切换 0.2ms ≈ 1ms,
// 实测典型往返 1~3ms; 但主循环里的 printf 是阻塞式的 (每字节忙等 TxComplete),
// 一行调试信息就能占住主循环数毫秒, 期间 Poll 不执行 → 应答被推迟处理 → rtt 被顶高。
// 实测 max_rtt 曾达 12ms, 正是这个原因。
//
// 取 15ms 的依据: 需同时满足
//   ① 远大于实测 max_rtt (12ms), 避免把"处理被推迟"误判成丢包;
//   ② 小于视觉周期 (约 20ms), 保证下一次 Trigger 到来前已完成超时判定。
// 另有 seq 匹配作为第二道保险 (见 Duplex_Process_Full_Frame): 迟到的应答即便跨过了超时线,
// 只要 seq 与最近一次请求不符就不会被错认成本次应答, 因此放宽阈值是安全的。
#define DUPLEX_REPLY_TIMEOUT_MS   15U

#define DUPLEX_RX_FIFO_SIZE       128U    // 接收 FIFO 容量 (字节), 远大于单帧 18B

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
// car_uplink_data 索引映射 (小车→无人机上传协议, 应答帧载荷):
//   [0] imu roll  — 小车横滚角 (deg)
//   [1] imu pitch — 小车俯仰角 (deg)
//   [2] imu yaw   — 小车偏航角 (deg)
// 测试阶段仅供观察, 不接入飞控任何逻辑; 后续前馈控制改传小车速度相关量。
extern float duplex_uplink_data[DUPLEX_UPLINK_COUNT];
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

// ================= 底层诊断埋点 =================
// 用途: 把"收不到应答"的原因切成三段, 一眼定位到硬件层还是软件层。
//   isr  = 0        → UART4 接收中断从未触发。总线上没有电平变化, 或 RX 引脚/中断配置问题。
//   isr >0 且 raw=0 → 中断触发了但 uart_query_byte 取不到数据 (异常, 一般是错误中断)。
//   raw >0 且 ok=0  → 字节能进来, 问题在帧同步/内容, 看 dec/cmd 定位。
extern volatile uint32_t duplex_rx_isr_count;         // uart4_isr 接收分支进入次数
extern volatile uint32_t duplex_rx_byte_count;        // 成功取到并入 FIFO 的原始字节数
extern volatile uint8_t  duplex_first_bytes[4];       // 最先收到的 4 个原始字节 (判帧头/波特率)
extern volatile uint8_t  duplex_first_byte_len;       // duplex_first_bytes 已记录的个数
extern volatile uint8_t  duplex_tx_de_high_readback;  // 拉高 DE 后回读电平 (期望 1)
extern volatile uint8_t  duplex_tx_de_low_readback;   // 拉低 DE 后回读电平 (期望 0)

// ================= 帧同步诊断埋点 =================
// 用途: 定位"应答帧到了但帧头被打坏"的具体形态。
//   hdr  = 在 HEADER1/HEADER2 状态被丢弃的字节数。正常每帧应答会贡献
//          DUPLEX_PEER_PREAMBLE_LEN 个 (前导字节本就该在此被吸收);
//          hdrx = hdr - ok × 前导长度, 即真正因帧头损坏而被丢弃的字节。
//   ovf  = SCB 硬件 RX FIFO 溢出次数。库把 OVERFLOW 中断屏蔽了, 只能主动查状态位。
//          若此值增长, 说明字节被硬件丢弃 (ISR 被高优先级中断挤住), 与帧同步无关。
//   run  = 最近一次"连续丢弃"的总字节数 (从上次找到 0xAA 到下次找到 0xAA 之间)。
//          正常帧 = 前导长度; 明显更大 = 该帧整体没能同步上。
//   lead = 该次连续丢弃开头连续等于前导字节值的个数 (即前导有几个正确解出)。
//   j[8] = 跳过上述前导后的头 8 个字节。这才是真正需要看的"垃圾形态":
//            j 全 0 且 run == 前导长度        → 正常帧, 无异常
//            j = AA 55 20 ...                → 帧头其实完好, 是别处的问题
//            j 首字节乱、其后是 55 20 ...     → 仅首字节损坏 (单字节故障)
//            j 整段都乱                      → 整帧位边界错乱 (采样时刻/边沿问题)
// 说明: 前导与垃圾分开记录, 是因为前导长度提到 8 后, 若混在一起会把 j[] 占满,
//       看不到真正有价值的后续字节。
#define DUPLEX_JUNK_CAPTURE_LEN   8U

extern volatile uint32_t duplex_hdr_drop_count;       // HEADER 状态丢弃的字节数
extern volatile uint32_t duplex_rx_overflow_count;    // SCB RX FIFO 溢出次数
extern volatile uint8_t  duplex_junk[DUPLEX_JUNK_CAPTURE_LEN]; // 跳过前导后的头 8 字节
extern volatile uint8_t  duplex_junk_len;             // duplex_junk 有效个数
extern volatile uint8_t  duplex_junk_lead;            // 开头连续等于前导值的个数
extern volatile uint16_t duplex_junk_run_total;       // 最近一次连续丢弃的总字节数
extern volatile uint16_t duplex_junk_run_max;         // 连续丢弃字节数的历史最大值

// 小车端应答帧的前导字节配置。仅用于把 hdr 换算成 hdrx (诊断用),
// 协议正确性不依赖它 —— 状态机对任何非 0xAA 字节都会丢弃。
// 注意: 若改动小车端 car_board_comm.h 的 BOARD_TX_PREAMBLE_LEN / BYTE, 这两个要同步改,
//       否则 hdrx 和 lead 会算错 (只影响诊断读数, 不影响通讯)。
#define DUPLEX_PEER_PREAMBLE_LEN   8U
#define DUPLEX_PEER_PREAMBLE_BYTE  0x00u

// ================= 对外接口 =================
void Duplex_Comm_Init(void);          // UART/方向引脚/FIFO 初始化, 进入接收态
void Duplex_Comm_On_Uart_Rx(void);    // uart4_isr 接收分支调用: 收字节入 FIFO
void Duplex_Comm_Poll(void);          // 主循环调用: 排空 FIFO 解析应答 + 超时判定
void Duplex_Comm_Trigger(const float *downlink);   // 视觉事件调用: 发起一次请求
void Duplex_Comm_Set_Now_Ms(uint32_t ms);          // 喂入毫秒时基 (dataC.pit0_cnt)
void Duplex_Comm_Reset_Stats(void);                // 清零收发统计 (启动稳定后调用)
void Duplex_Comm_Print_Stats(void);                // 有线 printf 输出通讯质量 (CM7_0主循环调用, 内部限频)

#endif
