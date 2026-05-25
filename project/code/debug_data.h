#ifndef _DEBUG_DATA_H_
#define _DEBUG_DATA_H_

#include "zf_common_headfile.h"

// ===== 功能开关 (置 1 启用, 0 关闭以释放 ~24KB RAM) =====
#define DEBUG_DATA_ENABLE        0

// ===== 可配置参数 =====
#define DEBUG_DATA_FLOAT_COUNT   3     // 每帧 float 参数个数
#define DEBUG_DATA_INT16_COUNT   4     // 每帧 int16 参数个数
#define DEBUG_DATA_DURATION_MS   1024  // 采集时长 (ms)
#define DEBUG_DATA_MAX_SAMPLES   1024  // 采集帧数
#define DEBUG_DATA_SEND_BATCH    2     // 主循环每次发送帧数 (非 ISR, 安全)

// 帧长: 2(sync) + 4(timestamp) + N*4 + M*2 + 1(checksum) + 1(0x7F)
#define DEBUG_FRAME_SIZE  (8 + DEBUG_DATA_FLOAT_COUNT * 4 + DEBUG_DATA_INT16_COUNT * 2)

// ===== 采样结构体 =====
typedef struct {
    uint32_t timestamp;                          // pit0_cnt (编入帧中做二级验证)
    float    floats[DEBUG_DATA_FLOAT_COUNT];     // 浮点参数
    int16_t  int16s[DEBUG_DATA_INT16_COUNT];     // 整型参数
} debug_sample_t;

// ===== 编译期校验 =====
// buffer 置于 0x28060000 独立区域, 大小上限 48KB (见 linker .icf)
#if (DEBUG_DATA_MAX_SAMPLES * sizeof(debug_sample_t)) > (48 * 1024)
#error "debug_buffer exceeds 48KB reserved region at 0x28060000"
#endif

// ===== 存储占用 =====
// sizeof(debug_sample_t) = 4 + 3*4 + 4*2 = 24 bytes
// 总缓冲: 1024 × 24 = 24576 bytes ≈ 24 KB

// ===== 状态变量 =====
extern volatile uint8_t  debug_recording;       // 1=采集中
extern volatile uint16_t debug_sample_count;    // 已采集帧数
extern volatile uint8_t  debug_data_ready;      // 1=采集完成,数据待发送
extern volatile uint8_t  debug_send_active;     // 1=发送中

// ===== 通知标志 (ISR 置位, 主循环消费) =====
#define DEBUG_NOTIFY_NONE       0
#define DEBUG_NOTIFY_START      1
#define DEBUG_NOTIFY_DONE       2
#define DEBUG_NOTIFY_SEND_DONE  3
extern volatile uint8_t debug_notify_pending;

// ===== API =====
void debug_data_get(void);              // 1ms ISR 调用: 采集一帧
void debug_data_send_handler(void);     // 主循环调用: 非阻塞批量发送
void debug_data_start(void);            // 开始采集
void debug_data_request_send(void);     // 请求发送已采集数据
void debug_data_notify_handler(void);   // 主循环调用: 发送状态通知字符串

// ===== 调参函数声明 =====
void Fly_Param_Update_Debug(uint8_t ch, float val);

#endif
