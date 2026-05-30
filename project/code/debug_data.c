#include "debug_data.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "data_complex.h"
#include "zf_device_wireless_uart.h"

#if DEBUG_DATA_ENABLE

// ===== 存储缓冲 (绝对地址放置，MAP 已确认无冲突) =====
#pragma location = 0x28060000
static debug_sample_t debug_buffer[DEBUG_DATA_MAX_SAMPLES];

// ===== 状态变量 =====
volatile uint8_t  debug_recording       = 0;
volatile uint16_t debug_sample_count    = 0;
volatile uint8_t  debug_data_ready      = 0;
volatile uint8_t  debug_send_active     = 0;
volatile uint8_t  debug_notify_pending  = DEBUG_NOTIFY_NONE;

static uint16_t send_cursor = 0;

// ===== 1ms ISR 调用: 采集一帧 =====
void debug_data_get(void)
{
    if (!debug_recording) return;

    if (debug_sample_count >= DEBUG_DATA_MAX_SAMPLES) {
        debug_recording = 0;
        debug_data_ready = 1;
        debug_notify_pending = DEBUG_NOTIFY_DONE;
        return;
    }

    debug_sample_t *s = &debug_buffer[debug_sample_count];
    s->timestamp = dataC.pit0_cnt;

    s->floats[0] = imu_data.roll;
    s->floats[1] = imu_data.pitch;
    s->floats[2] = imu_data.yaw;

    s->int16s[0] = motor_value.receive_speed_data[0];
    s->int16s[1] = motor_value.receive_speed_data[1];
    s->int16s[2] = motor_value.receive_speed_data[2];
    s->int16s[3] = motor_value.receive_speed_data[3];

    debug_sample_count++;
}

// ===== 编码单帧到 buffer, 返回帧长 =====
static uint16_t encode_frame(uint8_t *frame, uint16_t cursor)
{
    debug_sample_t *s = &debug_buffer[cursor];
    uint16_t pos = 0;

    frame[pos++] = 0xAA;  // 同步头
    frame[pos++] = 0x55;

    // timestamp (uint32 LE) — 解码端用单调性做二级验证
    uint32_t ts = s->timestamp;
    frame[pos++] = (uint8_t)(ts & 0xFF);
    frame[pos++] = (uint8_t)((ts >> 8) & 0xFF);
    frame[pos++] = (uint8_t)((ts >> 16) & 0xFF);
    frame[pos++] = (uint8_t)((ts >> 24) & 0xFF);

    // floats (LE)
    for (int i = 0; i < DEBUG_DATA_FLOAT_COUNT; i++) {
        uint32_t raw;
        memcpy(&raw, &s->floats[i], 4);
        frame[pos++] = (uint8_t)(raw & 0xFF);
        frame[pos++] = (uint8_t)((raw >> 8) & 0xFF);
        frame[pos++] = (uint8_t)((raw >> 16) & 0xFF);
        frame[pos++] = (uint8_t)((raw >> 24) & 0xFF);
    }

    // int16s (LE)
    for (int i = 0; i < DEBUG_DATA_INT16_COUNT; i++) {
        frame[pos++] = (uint8_t)(s->int16s[i] & 0xFF);
        frame[pos++] = (uint8_t)((s->int16s[i] >> 8) & 0xFF);
    }

    // checksum (XOR bytes 2..pos-1, 即 timestamp + floats + int16s)
    uint8_t checksum = 0;
    for (uint16_t i = 2; i < pos; i++) {
        checksum ^= frame[i];
    }
    frame[pos++] = checksum;
    frame[pos++] = 0x7F;  // 帧尾

    return pos;
}

// ===== 主循环调用: 非阻塞批量发送 =====
void debug_data_send_handler(void)
{
    if (!debug_send_active) return;

    uint8_t frame[DEBUG_FRAME_SIZE];
    uint16_t batch = 0;

    while (batch < DEBUG_DATA_SEND_BATCH && send_cursor < debug_sample_count) {
        uint16_t len = encode_frame(frame, send_cursor);
        if (wireless_uart_send_buffer(frame, len) != 0) {
            break;  // 模块忙超时, 帧未发出, 下次主循环重试
        }
        send_cursor++;
        batch++;
    }

    if (send_cursor >= debug_sample_count) {
        debug_send_active = 0;
        debug_notify_pending = DEBUG_NOTIFY_SEND_DONE;
    }
}

// ===== 开始采集 =====
void debug_data_start(void)
{
    debug_recording    = 1;
    debug_sample_count = 0;
    debug_data_ready   = 0;
    debug_send_active  = 0;
    send_cursor        = 0;
    debug_notify_pending = DEBUG_NOTIFY_START;
}

// ===== 请求发送 =====
void debug_data_request_send(void)
{
    if (debug_data_ready && !debug_send_active) {
        send_cursor = 0;
        debug_send_active = 1;
    }
}

// ===== 主循环调用: 发送状态通知 (非 ISR 上下文) =====
void debug_data_notify_handler(void)
{
    switch (debug_notify_pending) {
        case DEBUG_NOTIFY_START:
            wireless_uart_send_string("LOG:START\r\n");
            break;
        case DEBUG_NOTIFY_DONE:
            wireless_uart_send_string("LOG:DONE\r\n");
            break;
        case DEBUG_NOTIFY_SEND_DONE:
            wireless_uart_send_string("LOG:SEND_DONE\r\n");
            break;
        default:
            return;
    }
    debug_notify_pending = DEBUG_NOTIFY_NONE;
}

#else  // DEBUG_DATA_ENABLE == 0: 仅提供空桩, 不分配缓冲区

volatile uint8_t  debug_recording       = 0;
volatile uint16_t debug_sample_count    = 0;
volatile uint8_t  debug_data_ready      = 0;
volatile uint8_t  debug_send_active     = 0;
volatile uint8_t  debug_notify_pending  = DEBUG_NOTIFY_NONE;

void debug_data_get(void)              {}
void debug_data_send_handler(void)     {}
void debug_data_start(void)            {}
void debug_data_request_send(void)     {}
void debug_data_notify_handler(void)   {}

#endif
