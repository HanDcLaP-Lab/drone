#include "key_switch.h"
#include "zf_common_headfile.h"

// 实际分配内存空间
Key_Switch_t dev_key1;
Key_Switch_t dev_key2;
Key_Switch_t dev_key3;
Key_Switch_t dev_key4;
Key_Switch_t dev_switch1;
Key_Switch_t dev_switch2;

float debug_params[PARAM_COUNT] = {(float)THRESHOLD};
uint8_t current_param_idx = 0;             // 当前选中的参数索引
uint8_t display_page_idx = 0;              // [新增] 屏幕显示页面索引

// 创建一个指针数组，把所有要扫描的按键/拨码开关集中管理
static Key_Switch_t* const ALL_KEYS[] = { 
    &dev_key1, 
    &dev_key2, 
    &dev_key3, 
    &dev_key4, 
    &dev_switch1, 
    &dev_switch2
};
#define KEY_COUNT (sizeof(ALL_KEYS) / sizeof(ALL_KEYS[0]))

// 内部辅助函数，快速初始化对象的属性
static void Key_Switch_Object_Init(Key_Switch_t *key_obj, gpio_pin_enum pin, uint8_t active_level) { 
    key_obj->pin = pin;
    key_obj->active_level = active_level; // 按下时引脚为0 
    key_obj->raw_state = 0;
    key_obj->stable_state = 0;
    key_obj->last_stable_state = 0;
    key_obj->debounce_cnt = 0;
    key_obj->press_time = 0;
    key_obj->is_pressed = 0;
    key_obj->event = KEY_EVT_NONE;
}

// 按键与按键对象初始化
void key_switch_init(void){ 
    // 1. 初始化底层 GPIO 引脚模式
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);               

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);            
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);            

    // 2. 绑定按键对象与引脚 (注释表明按下时电平为0)
    Key_Switch_Object_Init(&dev_key1, KEY1, 0); 
    Key_Switch_Object_Init(&dev_key2, KEY2, 0);
    Key_Switch_Object_Init(&dev_key3, KEY3, 0);
    Key_Switch_Object_Init(&dev_key4, KEY4, 0);
    
    // Switch 也是拨向 ON 的一侧为 0
    Key_Switch_Object_Init(&dev_switch1, SWITCH1, 0);
    Key_Switch_Object_Init(&dev_switch2, SWITCH2, 0);
}

/**
 * @brief  按键状态更新与防抖处理核心
 * @param  key: 指向需要更新的按键对象的指针
 */
void Key_Switch_Update(Key_Switch_t *key) {
    // 1. 读取按键引脚瞬时状态，并转换为逻辑状态 (1=正在按, 0=没按)
    uint8_t current_raw = (gpio_get_level(key->pin) == key->active_level) ? 1 : 0;

    // 2. 核心消抖逻辑：只有状态持续不变达到设定阈值，才认可该状态
    if (current_raw != key->raw_state) {
        key->raw_state = current_raw;
        key->debounce_cnt = 0; // 只要有电平跳变（杂讯），立刻重新计时
    } else {
        key->debounce_cnt += KEY_DT;
        // 当电平稳定时间达到消抖阈值 (KS_MAX_SHOCK_PERIOD)
        if (key->debounce_cnt >= KS_MAX_SHOCK_PERIOD) {
            key->stable_state = key->raw_state;
            key->debounce_cnt = KS_MAX_SHOCK_PERIOD; // 防止计数器溢出
        }
    }

    // 3. 清理上一帧的触发事件
    key->event = KEY_EVT_NONE;
    key->is_pressed = key->stable_state;

    // 4. 边沿检测与长短按业务逻辑
    if (key->stable_state == 1 && key->last_stable_state == 0) {
        // [下降沿] 刚刚按下的瞬间
        key->event = KEY_EVT_DOWN;
        key->press_time = 0;
    } 
    else if (key->stable_state == 0 && key->last_stable_state == 1) {
        // [上升沿] 刚刚松开的瞬间
        key->event = KEY_EVT_UP;
        
        // 如果松开时，按下的时间满足短按条件
        if (key->press_time >= KS_MAX_SHOCK_PERIOD && key->press_time < KS_LONG_PRESS_PERIOD) {
            key->event = KEY_EVT_SHORT; 
        }
    } 
    else if (key->stable_state == 1 && key->last_stable_state == 1) {
        // [电平保持] 持续按住
        key->press_time += KEY_DT;
        
        // 达到长按阈值的时间触发一次长按事件
        if (key->press_time == KS_LONG_PRESS_PERIOD) {
            key->event = KEY_EVT_LONG;
        }
    }

    // 5. 更新历史状态供下一帧使用
    key->last_stable_state = key->stable_state;
}

// 聚合更新所有按键状态 (可直接放在 10ms 定时器中调用)
void Key_Switch_Update_All(void) {
    for (int i = 0; i < KEY_COUNT; i++) {
        Key_Switch_Update(ALL_KEYS[i]);
    }
}

// 单独测试 dev_key1 打印状态的测试函数
void debug_key1_test(void) {
    // 1. 聚合更新所有按键和拨码开关的状态
    Key_Switch_Update_All();

    // 2. 边沿与动作事件判断 (触发事件，一帧只进一次)
    if (dev_key1.event == KEY_EVT_DOWN) {
        printf("KEY1 [事件]: 刚刚被按下 (DOWN) \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_UP) {
        printf("KEY1 [事件]: 刚刚被松开 (UP) \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_SHORT) {
        printf("KEY1 [事件]: 触发【短按】(SHORT)! \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_LONG) {
        printf("KEY1 [事件]: 触发【长按】(LONG)! \r\n");
    }
}

void Key_Switch_Param_Edit(void) {
    // 1. KEY1 按下切换当前选中的参数 (未写 LONG 事件，但仍然有效)
    if (dev_key1.event == KEY_EVT_SHORT) {
        current_param_idx++;
        if (current_param_idx >= PARAM_COUNT) {
            current_param_idx = 0; // 越界切换到第一个
        }
    }
    //  KEY2 按下增加参数 
    if (dev_key2.event == KEY_EVT_SHORT) {
        
        // 按 5% 的变化量，并确保最小变化 1.0 
        float delta = debug_params[current_param_idx] * 0.05f;
        if (delta < 1.0f) delta = 1.0f;

        // 按下增加参数
        debug_params[current_param_idx] += delta;

        // 3. 安全限制循环范围 (阈值在 0~255 之间)
        if (current_param_idx == 0) {
            if (debug_params[0] > 255.0f) {
                debug_params[0] = 255.0f; 
            }
        }
    }

    if (dev_key3.event == KEY_EVT_SHORT) {
        
        // 按 5% 的变化量，并确保最小变化 1.0 
        float delta = debug_params[current_param_idx] * 0.05f;
        if (delta < 1.0f) delta = 1.0f;

        // 按下减小参数
        debug_params[current_param_idx] -= delta;

        // 3. 安全限制循环范围 (阈值在 0~255 之间)
        if (current_param_idx == 0) {
            if (debug_params[0] < 0.0f) {
                debug_params[0] = 0.0f; 
            }
        }
    }

    // KEY4 短按，切换屏幕显示页面
    if (dev_key4.event == KEY_EVT_SHORT) {
        display_page_idx++;
        if (display_page_idx >= DISPLAY_PAGE_COUNT) {
            display_page_idx = 0;
        }
    }
}