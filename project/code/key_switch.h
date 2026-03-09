#ifndef _KEY_SWITCH_H
#define _KEY_SWITCH_H
#include "zf_common_headfile.h"

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

// 时间基准与判定阈值宏 (使用 KS_ 前缀防�??与官方库冲突)
#define KEY_DT                  10    // Key_Switch_Update的周期，单位：ms
#define KS_MAX_SHOCK_PERIOD     20    // 消抖时间(ms)
#define KS_LONG_PRESS_PERIOD    1000  // 长按判定时间(ms)
#define PARAM_COUNT 1 // �������ε�����
// 定义状态枚�?
typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_DOWN,       // 刚刚按下的瞬�?
    KEY_EVT_UP,         // 刚刚松开的瞬�?
    KEY_EVT_SHORT,      // 成功触发�?�?
    KEY_EVT_LONG        // 成功触发长按
} Key_Event_e;

// 按键对象结构�?
typedef struct {
    gpio_pin_enum pin;           // 绑定的物理引�?
    uint8_t  active_level;       // 触发有效电平 (按下�?0，松开�?1)
    uint8_t  raw_state;          // �?时状�?
    uint8_t  stable_state;       // 消抖后的当前状�?
    uint8_t  last_stable_state;  // 上一帧的稳定状�?
    uint16_t debounce_cnt;       // 消抖计数�?
    uint16_t press_time;         // 已按下持�?时间
    uint8_t  is_pressed;         // 稳定长效保持状�? (1=按住, 0=松开)
    Key_Event_e event;           // 当前周期的瞬间动作事�?
} Key_Switch_t;

// 声明全局按键对象，供外部�? main �? isr 调用
extern Key_Switch_t dev_key1;
extern Key_Switch_t dev_key2;
extern Key_Switch_t dev_key3;
extern Key_Switch_t dev_key4;
extern Key_Switch_t dev_switch1;
extern Key_Switch_t dev_switch2;
extern float debug_params[PARAM_COUNT];
extern uint8_t current_param_idx;

// 外部�?调用的接口函�?
void key_switch_init(void);
void Key_Switch_Update(Key_Switch_t *key);
void Key_Switch_Update_All(void);
void debug_key1_test(void);
void Key_Switch_Param_Edit(void);

#endif