#include "duplex_comm.h"
#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * duplex_comm.c  无人机端(主机) 双向板间通讯实现
 *
 * 协议帧与时序说明见 duplex_comm.h。
 * 本文件只在 DUPLEX_SWITCH=1 时参与链路; DUPLEX_SWITCH=0 时函数体仍编译但无人调用,
 * 原单向 Board_Comm_Init/Board_Comm_Send_Data (data_complex.c) 继续生效。
 ********************************************************************************************************************/

// ================= 对外状态 / 统计 =================
float duplex_uplink_data[DUPLEX_UPLINK_COUNT] = {-1.0f, 0.0f, 0.0f, -1.0f}; // [3]=-1: 无前馈/未收到, 0°为有效方向
volatile uint8_t duplex_ff_deg_received = 0;   // 前馈采纳反馈 (由 image_ctrl 控制)
volatile uint8_t  duplex_uplink_update_flag = 0;
volatile uint32_t duplex_request_count      = 0;
volatile uint32_t duplex_reply_ok_count     = 0;
volatile uint32_t duplex_timeout_count      = 0;
volatile uint32_t duplex_decode_fail_count  = 0;
volatile uint32_t duplex_cmd_mismatch_count = 0;
volatile uint32_t duplex_rx_fifo_drop_count = 0;
volatile uint32_t duplex_seq_mismatch_count = 0;
volatile uint32_t duplex_last_rtt_ms        = 0;
volatile uint32_t duplex_max_rtt_ms         = 0;
volatile uint8_t  duplex_last_seq           = 0;
volatile uint8_t  duplex_last_err           = DUPLEX_ERR_NONE;

// ================= 内部状态 =================
static fifo_struct duplex_rx_fifo;
static uint8_t     duplex_rx_buffer[DUPLEX_RX_FIFO_SIZE];
static uint8_t     duplex_rx_byte;                                  // uart4_isr 单字节接收暂存

static uint8_t     duplex_tx_frame[DUPLEX_DOWNLINK_FRAME_SIZE];     // 请求帧发送缓冲
static uint8_t     duplex_rx_frame[DUPLEX_UPLINK_FRAME_SIZE];       // 应答帧累积缓冲

// 接收状态机: 仅靠帧头 0xAA 0x55 重同步, 累满整帧后再整帧校验
typedef enum {
    DUPLEX_STEP_HEADER1 = 0,   // 等待 0xAA
    DUPLEX_STEP_HEADER2,       // 等待 0x55
    DUPLEX_STEP_BODY           // 累积帧体至 DUPLEX_UPLINK_FRAME_SIZE
} duplex_rx_step_e;

static duplex_rx_step_e duplex_rx_step = DUPLEX_STEP_HEADER1;
static uint8_t  duplex_rx_idx = 0;          // 帧体已累积字节数 (含帧头)

// 请求-应答时序
static volatile uint32_t duplex_now_ms      = 0;   // 由 Duplex_Comm_Set_Now_Ms 喂入 (dataC.pit0_cnt)
static uint8_t  duplex_awaiting_reply       = 0;   // 1 = 已发请求, 等待应答
static uint32_t duplex_request_ms           = 0;   // 本次请求发出时刻
static uint8_t  duplex_seq                  = 0;   // 请求序列号 (自增, 自然回绕)

// ================= 帧编解码 =================
// 编码: 按 count 个 float 组帧。校验和覆盖 cmd + seq + 数据区。
static void Duplex_Frame_Encode(uint8_t cmd, uint8_t seq, const float *data, uint8_t count, uint8_t *frame)
{
    uint32_t data_bytes = (uint32_t)count * DUPLEX_FLOAT_BYTES;
    uint32_t i;
    uint8_t  checksum = 0;

    frame[0] = DUPLEX_HEADER1;
    frame[1] = DUPLEX_HEADER2;
    frame[DUPLEX_CMD_OFFSET] = cmd;
    frame[DUPLEX_SEQ_OFFSET] = seq;

    // 用 memcpy 写入数据区: 帧缓冲是 uint8_t 数组, 无对齐前提, 不能直接按 float* 赋值
    memcpy(&frame[DUPLEX_DATA_OFFSET], data, data_bytes);

    checksum += frame[DUPLEX_CMD_OFFSET];
    checksum += frame[DUPLEX_SEQ_OFFSET];
    for (i = 0; i < data_bytes; i++) {
        checksum += frame[DUPLEX_DATA_OFFSET + i];
    }

    frame[DUPLEX_DATA_OFFSET + data_bytes]      = checksum;
    frame[DUPLEX_DATA_OFFSET + data_bytes + 1U] = DUPLEX_TAIL;
}

// 解码上行应答帧: 校验帧尾与校验和, 剥离协议字段。返回 1 = 通过, 0 = 失败。
// 帧头已由状态机保证, 故失败必为帧尾或校验和。
static uint8_t Duplex_Frame_Decode(const uint8_t *frame, duplex_uplink_frame_t *out)
{
    const uint32_t data_bytes = (uint32_t)DUPLEX_UPLINK_COUNT * DUPLEX_FLOAT_BYTES;
    uint32_t i;
    uint8_t  checksum = 0;

    if (frame[DUPLEX_DATA_OFFSET + data_bytes + 1U] != DUPLEX_TAIL) {
        return 0;
    }

    checksum += frame[DUPLEX_CMD_OFFSET];
    checksum += frame[DUPLEX_SEQ_OFFSET];
    for (i = 0; i < data_bytes; i++) {
        checksum += frame[DUPLEX_DATA_OFFSET + i];
    }
    if (checksum != frame[DUPLEX_DATA_OFFSET + data_bytes]) {
        return 0;
    }

    out->cmd = frame[DUPLEX_CMD_OFFSET];
    out->seq = frame[DUPLEX_SEQ_OFFSET];
    memcpy(out->data, &frame[DUPLEX_DATA_OFFSET], data_bytes);
    return 1;
}

// ================= 初始化 =================
void Duplex_Comm_Init(void)
{
    // 方向引脚先进接收态 (低), 避免上电瞬间占用总线
    gpio_init(DUPLEX_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    fifo_init(&duplex_rx_fifo, FIFO_DATA_8BIT, duplex_rx_buffer, DUPLEX_RX_FIFO_SIZE);
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
    uart_rx_interrupt(BOARD_UART, 1);

    duplex_rx_step = DUPLEX_STEP_HEADER1;
    duplex_rx_idx  = 0;
    duplex_awaiting_reply = 0;
}

// ================= 时基喂入 =================
void Duplex_Comm_Set_Now_Ms(uint32_t ms)
{
    duplex_now_ms = ms;
}

// ================= 接收中断 =================
// uart4_isr 接收分支调用: 收一个字节入 FIFO, 解析全部放主循环 Duplex_Comm_Poll。
void Duplex_Comm_On_Uart_Rx(void)
{
    if (uart_query_byte(BOARD_UART, &duplex_rx_byte)) {
        if (fifo_write_buffer(&duplex_rx_fifo, &duplex_rx_byte, 1) != FIFO_SUCCESS) {
            duplex_rx_fifo_drop_count++;
            duplex_last_err = DUPLEX_ERR_FIFO;
        }
    }
}

// ================= 发起请求 =================
// 视觉事件调用: 发一帧 CMD_MASTER 请求 (载荷 = 下传数据), 随后切回接收态等应答。
void Duplex_Comm_Trigger(const float *downlink)
{
    if (!downlink) return;

    // 上一次请求还没等到应答就又被触发 → 判上一次丢包
    // (正常不会发生: 超时 8ms < 视觉周期 10ms @100Hz, Poll 会先判超时)
    if (duplex_awaiting_reply) {
        duplex_timeout_count++;
        duplex_last_err = DUPLEX_ERR_TIMEOUT;
        duplex_awaiting_reply = 0;
    }

    duplex_seq++;
    duplex_last_seq = duplex_seq;
    Duplex_Frame_Encode(DUPLEX_SELF_CMD, duplex_seq, downlink, DUPLEX_DOWNLINK_COUNT, duplex_tx_frame);

    // RS485 半双工: 拉高 DE 发送 → 等移位完成 → 拉低回接收态
    gpio_high(DUPLEX_DIR_PIN);
    system_delay_us(DUPLEX_DIR_SETUP_US);
    uart_write_buffer(BOARD_UART, duplex_tx_frame, sizeof(duplex_tx_frame));
    system_delay_us(DUPLEX_TX_HOLD_US);
    gpio_low(DUPLEX_DIR_PIN);

    duplex_request_count++;
    duplex_request_ms     = duplex_now_ms;
    duplex_awaiting_reply = 1;

    // [修复] 此处曾重置接收状态机 (step=HEADER1, idx=0), 理由写的是"清掉发送期间的自身
    // 回环残字节"。该理由不成立且造成约 3.7% 的上行丢包, 已删除:
    //   ① 不存在回环: DE 与 RE# 接在一起, 发送期间收发器的接收端物理关闭。实测 cmd 计数
    //      全程为 0 也证实了这点 (若有回环, 自身请求帧会因 cmd=MASTER≠SLAVE 被记入 cmd)。
    //   ② 重置会劈开半帧: 应答 22B @1Mbps 耗时 220us, 若 Poll 恰在传输中途读走前半帧
    //      (状态机停在 BODY), 紧接着视觉帧触发本函数把状态机打回 HEADER1, 则后半字节
    //      再也找不到帧头, 被逐个静默丢弃 —— 整帧丢失且不计入任何错误计数器。
    //      现象特征: raw 字节数明显多于 ok×18, 而 dec/cmd/fifo 全为 0。
}

// ================= 整帧处理 =================
static void Duplex_Process_Full_Frame(void)
{
    duplex_uplink_frame_t decoded;

    if (!Duplex_Frame_Decode(duplex_rx_frame, &decoded)) {
        duplex_decode_fail_count++;
        duplex_last_err = DUPLEX_ERR_DECODE;
        return;
    }

    // 命令字判别: 只认从机应答。半双工总线上自身请求帧回环会落在此分支。
    if (decoded.cmd != DUPLEX_PEER_CMD) {
        duplex_cmd_mismatch_count++;
        duplex_last_err = DUPLEX_ERR_CMD;
        return;
    }

    // [新增] seq 匹配: 从机应答会回显请求的 seq。只认与最近一次请求相符的应答,
    // 避免把上一轮迟到的旧应答错认成本次应答 (那会导致 rtt 统计失真、上行数据滞后一帧)。
    // 有了这道校验, 超时阈值才可以放宽到大于实测 max_rtt 而不引入错配风险。
    if (decoded.seq != duplex_last_seq) {
        duplex_seq_mismatch_count++;
        duplex_last_err = DUPLEX_ERR_SEQ;
        return;
    }

    // 缓存上行载荷
    for (uint8_t i = 0; i < DUPLEX_UPLINK_COUNT; i++) {
        duplex_uplink_data[i] = decoded.data[i];
    }
    duplex_uplink_update_flag = 1;

    if (duplex_awaiting_reply) {
        duplex_last_rtt_ms = duplex_now_ms - duplex_request_ms;
        if (duplex_last_rtt_ms > duplex_max_rtt_ms) {
            duplex_max_rtt_ms = duplex_last_rtt_ms;
        }
        duplex_awaiting_reply = 0;
    }

    duplex_reply_ok_count++;
    duplex_last_err = DUPLEX_ERR_NONE;
}

// ================= 主循环轮询 =================
// 排空接收 FIFO 驱动状态机; 再做应答超时判定。
void Duplex_Comm_Poll(void)
{
    uint8_t  read_byte;
    uint32_t len;
    uint32_t primask;
    uint32_t fifo_now;

    // FIFO 的 size 被 ISR(写) 与本函数(读) 跨上下文非原子读改写: 关中断内快照一次长度并
    // 做越界钳位, 用快照长度驱动循环, 保证循环次数有界 (同 car_board_comm.c 的加固思路)。
    primask  = interrupt_global_disable();
    fifo_now = fifo_used(&duplex_rx_fifo);
    if (fifo_now > DUPLEX_RX_FIFO_SIZE) {
        fifo_clear(&duplex_rx_fifo);
        fifo_now = 0;
        duplex_rx_fifo_drop_count++;
        duplex_last_err = DUPLEX_ERR_FIFO;
    }
    interrupt_global_enable(primask);

    while (fifo_now > 0) {
        len = 1;
        primask = interrupt_global_disable();
        fifo_read_buffer(&duplex_rx_fifo, &read_byte, &len, FIFO_READ_AND_CLEAN);
        interrupt_global_enable(primask);
        if (len == 0) break;          // 防呆: 未读到数据立即退出, 杜绝空转
        fifo_now--;

        switch (duplex_rx_step) {
            case DUPLEX_STEP_HEADER1:
                // 非 0xAA 一律丢弃并停在本状态重新找帧头。
                // 小车应答帧前置的前导字节 (见 car_board_comm.h 的 BOARD_TX_PREAMBLE_*)
                // 正是在此被正常吸收。
                if (read_byte == DUPLEX_HEADER1) {
                    duplex_rx_frame[0] = read_byte;
                    duplex_rx_step = DUPLEX_STEP_HEADER2;
                }
                break;

            case DUPLEX_STEP_HEADER2:
                if (read_byte == DUPLEX_HEADER2) {
                    duplex_rx_frame[1] = read_byte;
                    duplex_rx_idx  = 2;
                    duplex_rx_step = DUPLEX_STEP_BODY;
                } else if (read_byte != DUPLEX_HEADER1) {
                    // 连续 0xAA 时停在 HEADER2 等 0x55, 其余字节退回重新找帧头
                    duplex_rx_step = DUPLEX_STEP_HEADER1;
                }
                break;

            case DUPLEX_STEP_BODY:
                duplex_rx_frame[duplex_rx_idx++] = read_byte;

                // 命令字一到位就先判别: 自身 58 字节请求帧回环时, 按上行 22 字节累积会
                // 读出 cmd=CMD_MASTER, 此处提前丢弃并重同步, 不必等累满整帧再判。
                if (duplex_rx_idx == DUPLEX_DATA_OFFSET &&
                    duplex_rx_frame[DUPLEX_CMD_OFFSET] != DUPLEX_PEER_CMD) {
                    duplex_cmd_mismatch_count++;
                    duplex_last_err = DUPLEX_ERR_CMD;
                    duplex_rx_step = DUPLEX_STEP_HEADER1;
                    duplex_rx_idx  = 0;
                    break;
                }

                if (duplex_rx_idx >= DUPLEX_UPLINK_FRAME_SIZE) {
                    Duplex_Process_Full_Frame();
                    duplex_rx_step = DUPLEX_STEP_HEADER1;
                    duplex_rx_idx  = 0;
                }
                break;

            default:
                duplex_rx_step = DUPLEX_STEP_HEADER1;
                duplex_rx_idx  = 0;
                break;
        }
    }

    // 应答超时判定 (测试阶段只统计, 不做任何降级动作)
    if (duplex_awaiting_reply &&
        (uint32_t)(duplex_now_ms - duplex_request_ms) >= DUPLEX_REPLY_TIMEOUT_MS) {
        duplex_timeout_count++;
        duplex_last_err = DUPLEX_ERR_TIMEOUT;
        duplex_awaiting_reply = 0;
    }
}

// ================= 统计清零 =================
// 上电/接线稳定前的瞬态会污染丢包率, 启动若干秒后调用一次即可得到稳态统计。
void Duplex_Comm_Reset_Stats(void)
{
    uint32_t primask = interrupt_global_disable();
    duplex_request_count      = 0;
    duplex_reply_ok_count     = 0;
    duplex_timeout_count      = 0;
    duplex_decode_fail_count  = 0;
    duplex_cmd_mismatch_count = 0;
    duplex_rx_fifo_drop_count = 0;
    duplex_seq_mismatch_count = 0;
    duplex_last_rtt_ms        = 0;
    duplex_max_rtt_ms         = 0;
    duplex_last_err           = DUPLEX_ERR_NONE;
    interrupt_global_enable(primask);
}

// ================= 通讯质量打印 (有线 printf) =================
// 由 CM7_0 主循环调用, 内部按 DUPLEX_PRINT_PERIOD_MS 限频。printf 走 UART_0 @115200,
// 与板间通讯 UART4、无线串口 UART1 均不冲突。
//
// 输出格式 (逗号分隔, \r\n 结尾):
//   dup   固定前缀, 便于上位机过滤
//   req   已发起请求数
//   ok    收到有效应答数        → 丢包率 = (req - ok) / req
//   to    应答超时(丢包)数
//   dec   解码失败数 (帧尾/校验和)
//   cmd   命令字不匹配数 (含自身请求帧回环)
//   fifo  接收 FIFO 溢出数
//   sq    seq 不符被丢弃数 (迟到的旧应答)
//   rtt   最近一次往返时延 (ms)
//   max   往返时延最大值 (ms)
//   err   最近失败原因码 (0无/1解码/2命令字/3超时/4FIFO/5seq不符)
//   u0/1/2/3 小车上传载荷 (IMU roll/pitch/yaw + 前馈角 ff_deg)
//   fb    前馈采纳反馈 (0=未采纳, 1=已采纳有效前馈角, 含0°)
//
// 注意: printf 为阻塞式 (每字节忙等 TxComplete), 本行约 80 字节 @115200 要占住主循环约 7ms。
//   默认不调用 (见 main_cm7_0.c 的注释掉的调用处), 仅在需要观察链路质量时临时开启。
void Duplex_Comm_Print_Stats(void)
{
    static uint32_t last_print_ms = 0;

    if ((uint32_t)(duplex_now_ms - last_print_ms) < DUPLEX_PRINT_PERIOD_MS) return;
    last_print_ms = duplex_now_ms;

    // 浮点参数显式转 double: 可变参数会默认提升, 显式写出与 wireless_uart.c 既有风格一致
    printf("dup,%u,%u,%u,%u,%u,%u,sq%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,fb%u\r\n",
           (unsigned)duplex_request_count,
           (unsigned)duplex_reply_ok_count,
           (unsigned)duplex_timeout_count,
           (unsigned)duplex_decode_fail_count,
           (unsigned)duplex_cmd_mismatch_count,
           (unsigned)duplex_rx_fifo_drop_count,
           (unsigned)duplex_seq_mismatch_count,
           (unsigned)duplex_last_rtt_ms,
           (unsigned)duplex_max_rtt_ms,
           (unsigned)duplex_last_err,
           (double)duplex_uplink_data[0],
           (double)duplex_uplink_data[1],
           (double)duplex_uplink_data[2],
           (double)duplex_uplink_data[3],
           (unsigned)duplex_ff_deg_received);
}
