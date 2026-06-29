/*********************************************************************************************************************
 * duplex_comm.c  无人机端(主机)双向板间通讯模块
 *
 * 本文件实现「接收链路」: 对外变量/计数器定义、接收 FIFO、UART4 收字节入队、
 * 含帧头重同步的解析状态机。请求触发/超时判丢包/Poll/RS485 方向控制等逻辑
 * 由任务 2.3 补齐(见文件末尾「2.3 待补」区域)。
 *
 * 角色: Master(无人机) —— 发送 CMD_MASTER(0x10) 请求, 接收 CMD_SLAVE(0x20) 应答。
 * 协议与编解码与参考案例 2bl3_wireless_test/board_float8_comm.c 字节级一致。
 *********************************************************************************************************************/

#include "duplex_comm.h"
#include <string.h>

// ================= 协议帧编解码 (原 board_float8_frame.c, 已并入本模块) =================
#define BOARD_FLOAT8_HEADER1      0xAAu
#define BOARD_FLOAT8_HEADER2      0x55u
#define BOARD_FLOAT8_TAIL         0x7Fu
#define BOARD_FLOAT8_CMD_INDEX    2u
#define BOARD_FLOAT8_SEQ_INDEX    3u
#define BOARD_FLOAT8_DATA_INDEX   4u
#define BOARD_FLOAT8_SUM_INDEX    (BOARD_FLOAT8_FRAME_SIZE - 2u)
#define BOARD_FLOAT8_TAIL_INDEX   (BOARD_FLOAT8_FRAME_SIZE - 1u)

// 累加校验: 对 [data, data+length) 区间按字节累加, 自然溢出取模 256
static uint8_t Board_Float8_Checksum(const uint8_t *data, uint32_t length)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

// 编码: 帧头 0xAA 0x55 + cmd + seq + 8×float(小端 IEEE754) + 累加校验 + 帧尾 0x7F
void Board_Float8_Frame_Encode(uint8_t cmd, uint8_t seq, const float data[BOARD_FLOAT8_COUNT], uint8_t frame[BOARD_FLOAT8_FRAME_SIZE])
{
    frame[0] = BOARD_FLOAT8_HEADER1;
    frame[1] = BOARD_FLOAT8_HEADER2;
    frame[BOARD_FLOAT8_CMD_INDEX] = cmd;
    frame[BOARD_FLOAT8_SEQ_INDEX] = seq;
    memcpy(&frame[BOARD_FLOAT8_DATA_INDEX], data, BOARD_FLOAT8_DATA_BYTES);
    frame[BOARD_FLOAT8_SUM_INDEX] = Board_Float8_Checksum(&frame[BOARD_FLOAT8_CMD_INDEX], 2u + BOARD_FLOAT8_DATA_BYTES);
    frame[BOARD_FLOAT8_TAIL_INDEX] = BOARD_FLOAT8_TAIL;
}

// 解码: 校验帧头/帧尾/累加校验, 失败返回 0; 成功填充 out 并返回 1
uint8_t Board_Float8_Frame_Decode(const uint8_t frame[BOARD_FLOAT8_FRAME_SIZE], board_float8_frame_t *out)
{
    uint8_t checksum;

    if (frame[0] != BOARD_FLOAT8_HEADER1 || frame[1] != BOARD_FLOAT8_HEADER2) {
        return 0;
    }
    if (frame[BOARD_FLOAT8_TAIL_INDEX] != BOARD_FLOAT8_TAIL) {
        return 0;
    }

    checksum = Board_Float8_Checksum(&frame[BOARD_FLOAT8_CMD_INDEX], 2u + BOARD_FLOAT8_DATA_BYTES);
    if (checksum != frame[BOARD_FLOAT8_SUM_INDEX]) {
        return 0;
    }

    out->cmd = frame[BOARD_FLOAT8_CMD_INDEX];
    out->seq = frame[BOARD_FLOAT8_SEQ_INDEX];
    memcpy(out->data, &frame[BOARD_FLOAT8_DATA_INDEX], BOARD_FLOAT8_DATA_BYTES);

    return 1;
}

// ================= 对外状态 / 计数器 (duplex_comm.h 中声明的全部对外变量) =================
float    duplex_uplink_data[BOARD_FLOAT8_COUNT] = {0}; // 小车上传(占位)接收缓冲
volatile uint8_t  duplex_uplink_update_flag = 0;       // 收到有效应答置 1
volatile uint32_t duplex_request_count      = 0;       // 已发起请求数
volatile uint32_t duplex_reply_ok_count     = 0;       // 收到有效应答数
volatile uint32_t duplex_timeout_count      = 0;       // 超时(丢包)数
volatile uint32_t duplex_decode_fail_count  = 0;       // 解码失败数
volatile uint32_t duplex_cmd_mismatch_count = 0;       // 命令字不匹配数
volatile uint32_t duplex_rx_fifo_drop_count = 0;       // 接收 FIFO 溢出数
volatile uint8_t  duplex_last_seq           = 0;       // 最近请求 seq
volatile uint8_t  duplex_last_err           = 0;       // 最近一次失败原因码 (调试打印用)

// ================= 接收 FIFO =================
static fifo_struct duplex_rx_fifo;                        // 接收环形缓冲管理结构
static uint8_t     duplex_rx_buffer[DUPLEX_RX_FIFO_SIZE]; // 接收 FIFO 底层缓冲区

// ================= 接收解析状态机 (含帧头 0xAA 0x55 重同步) =================
typedef enum {
    RX_STEP_HEADER1 = 0,    // 等待第一个帧头字节 0xAA
    RX_STEP_HEADER2,        // 等待第二个帧头字节 0x55
    RX_STEP_BODY            // 累积帧体, 满 38 字节后解码
} duplex_rx_step_t;

/**
 * @brief  单字节解析(纯软件逻辑, 不依赖硬件 UART/GPIO, 便于宿主机测试)
 *         逐字节驱动状态机: 帧头匹配 -> 累积帧体 -> 满 38 字节调用 Decode。
 *         命令字 == DUPLEX_PEER_CMD(CMD_SLAVE): 存入 duplex_uplink_data 并置更新标志;
 *         命令字不匹配: 递增 duplex_cmd_mismatch_count;
 *         解码失败:     递增 duplex_decode_fail_count。
 * @param  byte  待解析的接收字节
 * @note   收到有效应答时仅置 duplex_uplink_update_flag 与刷新 duplex_uplink_data;
 *         「清等待态 / 递增 duplex_reply_ok_count」由任务 2.3 的 Poll 负责(按 design)。
 */
static void Duplex_Parse_Byte(uint8_t byte)
{
    static duplex_rx_step_t step = RX_STEP_HEADER1;
    static uint8_t frame[BOARD_FLOAT8_FRAME_SIZE];
    static uint8_t index = 0;
    board_float8_frame_t decoded;

    switch (step) {
        case RX_STEP_HEADER1:
            // 仅当字节为 0xAA 时前进, 否则原地等待(噪声/中段损坏自动丢弃)
            if (byte == 0xAAu) {
                frame[0] = byte;
                step = RX_STEP_HEADER2;
            }
            break;

        case RX_STEP_HEADER2:
            if (byte == 0x55u) {
                // 完整帧头, 进入帧体累积
                frame[1] = byte;
                index = 2;
                step = RX_STEP_BODY;
            } else if (byte == 0xAAu) {
                // 连续 0xAA: 保持在 HEADER2, 等待下一个字节作为第二头字节(重同步)
            } else {
                // 非帧头, 回退重同步
                step = RX_STEP_HEADER1;
            }
            break;

        case RX_STEP_BODY:
            frame[index++] = byte;
            if (index >= BOARD_FLOAT8_FRAME_SIZE) {
                // 累积满一帧, 无论成功失败都回到 HEADER1 重新同步
                if (Board_Float8_Frame_Decode(frame, &decoded)) {
                    duplex_last_seq = decoded.seq;
                    if (decoded.cmd == DUPLEX_PEER_CMD) {
                        // 收到对端(小车)有效应答: 刷新上传缓冲并置更新标志
                        memcpy(duplex_uplink_data, decoded.data, sizeof(duplex_uplink_data));
                        duplex_uplink_update_flag = 1;
                        duplex_last_err = DUPLEX_ERR_NONE;   // 成功收到应答, 清最近错误码
                        // 注: duplex_reply_ok_count 递增与等待态清除交由 2.3 的 Poll(见 design)
                    } else {
                        // 命令字不匹配(非 CMD_SLAVE)
                        duplex_cmd_mismatch_count++;
                        duplex_last_err = DUPLEX_ERR_CMD;
                    }
                } else {
                    // 校验/帧头/帧尾损坏导致解码失败
                    duplex_decode_fail_count++;
                    duplex_last_err = DUPLEX_ERR_DECODE;
                }
                step = RX_STEP_HEADER1;
            }
            break;

        default:
            step = RX_STEP_HEADER1;
            break;
    }
}

/**
 * @brief  排空接收 FIFO 并逐字节解析(纯逻辑入口, 供任务 2.3 的 Poll 调用)
 * @note   仅依赖 FIFO 与纯软件状态机, 不直接触碰 UART/GPIO 硬件。
 */
static void Duplex_Parse_Fifo(void)
{
    uint8_t  byte;
    uint32_t len;

    while (fifo_used(&duplex_rx_fifo) > 0) {
        len = 1;
        fifo_read_buffer(&duplex_rx_fifo, &byte, &len, FIFO_READ_AND_CLEAN);
        Duplex_Parse_Byte(byte);
    }
}

// ================= 接收链路对外接口 =================

/**
 * @brief  UART4 接收中断分支调用: 读出全部可用字节写入接收 FIFO
 *         FIFO 写满则丢弃当前字节并递增 duplex_rx_fifo_drop_count。
 * @note   由 cm7_0_isr.c 的 uart4_isr() 接收分支调用(任务 5.4 接入)。
 */
void Duplex_Comm_On_Uart_Rx(void)
{
    uint8_t byte;

    while (uart_query_byte(DUPLEX_UART, &byte)) {
        if (fifo_write_buffer(&duplex_rx_fifo, &byte, 1) != FIFO_SUCCESS) {
            duplex_rx_fifo_drop_count++;   // FIFO 溢出: 丢弃当前字节, 继续接收
            duplex_last_err = DUPLEX_ERR_FIFO;
        }
    }
}

/**
 * @brief  主机模块初始化骨架: FIFO / UART4 / RS485 方向引脚, 进入接收态
 * @note   接收链路初始化在本任务(2.2)完成; 与请求/应答相关的内部状态初始化
 *         (请求态机、时基、tx_seq 等)由任务 2.3 视需要补充。
 */
void Duplex_Comm_Init(void)
{
    // 1) 接收 FIFO 初始化
    fifo_init(&duplex_rx_fifo, FIFO_DATA_8BIT, duplex_rx_buffer, DUPLEX_RX_FIFO_SIZE);

    // 2) RS485 方向引脚初始化为接收态(LOW), 空闲保持接收, 避免长期占用总线
    gpio_init(DUPLEX_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 3) UART4 初始化并开接收中断
    uart_init(DUPLEX_UART, DUPLEX_BAUDRATE, DUPLEX_TX_PIN, DUPLEX_RX_PIN);
    uart_rx_interrupt(DUPLEX_UART, 1);

    // 进入接收态(方向引脚已置 LOW)
}

/* =====================================================================================
 * 2.3 实现区域 —— 请求触发 / 超时判丢包 / Poll / RS485 方向控制 / 毫秒时基 / Uplink 读取
 * -------------------------------------------------------------------------------------
 * 主机请求-应答状态机: 视觉事件触发一次请求(至多发一帧), 进入等待应答态;
 * 主循环周期性 Poll 排空解析 FIFO, 收到有效应答则回空闲态, 超时则计丢包并回空闲态。
 * 不重写 2.2 的接收链路/解析状态机, 仅复用 Duplex_Parse_Fifo() 排空解析入口。
 * ===================================================================================== */

// ================= 主机请求状态机内部状态 =================
typedef enum {
    DUPLEX_IDLE = 0,        // 空闲: 接收态, 无未决请求, 可发起新请求
    DUPLEX_WAIT_REPLY       // 已发请求, 等待应答(截止于下一次视觉事件)
} duplex_state_t;

static duplex_state_t   duplex_state  = DUPLEX_IDLE; // 当前请求态
static uint8_t          duplex_tx_seq = 0;           // 请求自增序列号

// ================= RS485 半双工方向控制 =================
/**
 * @brief  置发送态: 方向引脚拉高 + 建立时间延时
 * @note   DE/RE 接一起: 高=发送。切到发送态后需等待收发器建立时间(Req 8.3)。
 */
static void Duplex_Rs485_Tx_Mode(void)
{
    gpio_set_level(DUPLEX_DIR_PIN, GPIO_HIGH);
    system_delay_us(DUPLEX_DIR_SETUP_US);   // 方向建立时间
}

/**
 * @brief  置接收态: 方向引脚拉低
 * @note   空闲与任何发送结束后均须回到接收态(LOW), 避免长期占用总线(Req 8.4)。
 */
static void Duplex_Rs485_Rx_Mode(void)
{
    gpio_set_level(DUPLEX_DIR_PIN, GPIO_LOW);
}

// ================= 请求触发 / 时基 / Poll / Uplink 读取 =================

/**
 * @brief  视觉事件调用: 发起一次「请求-应答」
 *         以"下一次视觉事件(下一次发送)"为判丢包的截止点:
 *         若进入本函数时上一次请求仍处于 WAIT_REPLY(到本帧仍未收到应答),
 *         则将上一次请求计为一次丢包, 然后无条件发起新请求。
 *         因此每个视觉事件都会发一帧请求, 不会因仍在等待而跳过下传。
 * @param  downlink  下传数据载荷(8 × float), 来自 Float_Buffer_write
 * @note   发送时序: 置发送态 -> 写帧 -> 发送保持延时(等移位完成) -> 置回接收态(Req 8.3/8.4)。
 */
void Duplex_Comm_Trigger(const float downlink[BOARD_FLOAT8_COUNT])
{
    uint8_t frame[BOARD_FLOAT8_FRAME_SIZE];
    uint8_t seq;

    // 上一次请求到本次视觉事件仍未收到应答 => 判为丢包(截止点 = 本次发送时刻)
    if (duplex_state == DUPLEX_WAIT_REPLY) {
        duplex_timeout_count++;
        duplex_last_err = DUPLEX_ERR_TIMEOUT;
    }

    // 用自增 seq 编码请求帧(CMD_MASTER), 记录本次请求 seq
    seq = duplex_tx_seq++;
    Board_Float8_Frame_Encode(DUPLEX_SELF_CMD, seq, downlink, frame);
    duplex_last_seq = seq;

    // RS485 发送时序: 发送态 -> 写帧 -> 保持(等移位寄存器搬空) -> 接收态
    Duplex_Rs485_Tx_Mode();
    uart_write_buffer(DUPLEX_UART, frame, BOARD_FLOAT8_FRAME_SIZE);
    system_delay_us(DUPLEX_TX_HOLD_US);
    Duplex_Rs485_Rx_Mode();

    // 进入等待应答态; 清更新标志, 使本请求窗口内的应答可被干净检出
    duplex_request_count++;
    duplex_uplink_update_flag = 0;   // 清旧标志: 新等待窗口从"未收到应答"开始
    duplex_state = DUPLEX_WAIT_REPLY;
}

/**
 * @brief  [保留接口兼容] 喂入毫秒时基
 * @param  ms  当前毫秒计数
 * @note   现以"下一次视觉事件"为判丢包截止点, 不再使用固定毫秒超时; 本函数保留为空操作,
 *         避免改动 main 的调用点。如需恢复固定超时再启用。
 */
void Duplex_Comm_Set_Now_Ms(uint32_t ms)
{
    (void)ms;
}

/**
 * @brief  主循环周期性调用: 排空 FIFO 解析应答; 收到有效应答即计成功
 *         流程:
 *           1) 记录解析前的更新标志旧值;
 *           2) Duplex_Parse_Fifo() 排空解析, 收到有效应答会置 update_flag;
 *           3) WAIT_REPLY 态下本次 Poll 新置了 update_flag(旧0->新1) => 成功, 回空闲态。
 *         不在此处做超时判定 —— 丢包改由下一次 Duplex_Comm_Trigger 判定(见上)。
 * @note   以"本次 Poll 是否新置 update_flag"判定, 配合 Trigger 在发起时清标志,
 *         避免把上一请求的残留标志误判为本次应答, 且不重复计数。
 */
void Duplex_Comm_Poll(void)
{
    uint8_t flag_before = duplex_uplink_update_flag;   // 解析前快照, 用于检出"本次新置"

    // 排空接收 FIFO 并解析
    Duplex_Parse_Fifo();

    if (duplex_state == DUPLEX_WAIT_REPLY
        && duplex_uplink_update_flag != 0 && flag_before == 0) {
        // 本次 Poll 新收到有效应答帧 => 计成功应答, 回空闲态
        duplex_reply_ok_count++;
        duplex_state = DUPLEX_IDLE;
    }
}

/**
 * @brief  读取小车上传(占位)数据缓冲
 * @return 指向 duplex_uplink_data[8] 的常量指针
 */
const float* Duplex_Comm_Get_Uplink(void)
{
    return duplex_uplink_data;
}

/**
 * @brief  清零通讯收发统计计数 (供启动稳定后清零, 以统计稳态丢包率)
 * @note   只清统计量, 不影响当前请求状态机与上传数据缓冲。
 */
void Duplex_Comm_Reset_Stats(void)
{
    duplex_request_count      = 0;
    duplex_reply_ok_count     = 0;
    duplex_timeout_count      = 0;
    duplex_decode_fail_count  = 0;
    duplex_cmd_mismatch_count = 0;
    duplex_rx_fifo_drop_count = 0;
    duplex_last_err           = DUPLEX_ERR_NONE;
}
