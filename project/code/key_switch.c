#include "key_switch.h"
#include "zf_common_headfile.h"

// 实际分配内存空间
Key_Switch_t dev_key1;
Key_Switch_t dev_key2;
Key_Switch_t dev_key3;
Key_Switch_t dev_key4;
Key_Switch_t dev_switch1;
Key_Switch_t dev_switch2;

float debug_params[PARAM_COUNT] = {70.0f}; // ����0: ��ֵ����ֵ��Ĭ��70
uint8_t current_param_idx = 0;             // ��ǰѡ�еĲ�������

// 创建一个指针数组，把所有要扫描的按�?/拨码开关集中管�?
static Key_Switch_t* const ALL_KEYS[] = {
    &dev_key1, 
    &dev_key2, 
    &dev_key3, 
    &dev_key4, 
    &dev_switch1, 
    &dev_switch2
};
#define KEY_COUNT (sizeof(ALL_KEYS) / sizeof(ALL_KEYS[0]))

// 内部辅助函数，快速初始化对象的属�?
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

// 硬件与软件对象初始化
void key_switch_init(void){
    // 1. 初始化底�? GPIO 引脚模式
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);               
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);               

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);            
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);            

    // 2. 绑定软件对象与引�? (注释表明按下时电平为0)
    Key_Switch_Object_Init(&dev_key1, KEY1, 0);
    Key_Switch_Object_Init(&dev_key2, KEY2, 0);
    Key_Switch_Object_Init(&dev_key3, KEY3, 0);
    Key_Switch_Object_Init(&dev_key4, KEY4, 0);
    
    // Switch 也是�? ON 的一侧为 0
    Key_Switch_Object_Init(&dev_switch1, SWITCH1, 0);
    Key_Switch_Object_Init(&dev_switch2, SWITCH2, 0);
}

/**
 * @brief  按键状态更新与防抖处理核心
 * @param  key: 指向需要更新的按键对象的指�?
 */
void Key_Switch_Update(Key_Switch_t *key) {
    // 1. 读取硬件引脚瞬时状态，转换为逻辑状�? (1=正在�?, 0=没按)
    uint8_t current_raw = (gpio_get_level(key->pin) == key->active_level) ? 1 : 0;

    // 2. 核心消抖逻辑：只有状态持续不变达到设定阈值，才认可该状�?
    if (current_raw != key->raw_state) {
        key->raw_state = current_raw;
        key->debounce_cnt = 0; // 只要有电平跳变（杂波），立刻重新计时
    } else {
        key->debounce_cnt += KEY_DT;
        // 当电平稳定时间达到消抖阈�? (KS_MAX_SHOCK_PERIOD)
        if (key->debounce_cnt >= KS_MAX_SHOCK_PERIOD) {
            key->stable_state = key->raw_state;
            key->debounce_cnt = KS_MAX_SHOCK_PERIOD; // 防止计数器溢�?
        }
    }

    // 3. 清理上一帧的瞬发事件
    key->event = KEY_EVT_NONE;
    key->is_pressed = key->stable_state;

    // 4. 边沿检测与长短按业务逻辑
    if (key->stable_state == 1 && key->last_stable_state == 0) {
        // [下降沿] 刚刚按下的瞬�?
        key->event = KEY_EVT_DOWN;
        key->press_time = 0;
    } 
    else if (key->stable_state == 0 && key->last_stable_state == 1) {
        // [上升沿] 刚刚松开的瞬�?
        key->event = KEY_EVT_UP;
        
        // 如果松开时，按下的时间满足短按条�?
        if (key->press_time >= KS_MAX_SHOCK_PERIOD && key->press_time < KS_LONG_PRESS_PERIOD) {
            key->event = KEY_EVT_SHORT; 
        }
    } 
    else if (key->stable_state == 1 && key->last_stable_state == 1) {
        // [电平保持] 持续按住
        key->press_time += KEY_DT;
        
        // 达到长按阈值的瞬间触发一次长按事�?
        if (key->press_time == KS_LONG_PRESS_PERIOD) {
            key->event = KEY_EVT_LONG;
        }
    }

    // 5. 更新历史状态供下一帧对�?
    key->last_stable_state = key->stable_state;
}

// 聚合更新所有设备状�? (可直接放�? 10ms 定时器中断中)
void Key_Switch_Update_All(void) {
    for (int i = 0; i < KEY_COUNT; i++) {
        Key_Switch_Update(ALL_KEYS[i]);
    }
}

// 单独测试 dev_key1 打印状态的测试函数
void debug_key1_test(void) {
    // 1. 聚合更新所有按键和拨码开关的状�?
    Key_Switch_Update_All();

    // 2. 边沿与动作事件判�? (瞬发事件，一帧只进一�?)
    if (dev_key1.event == KEY_EVT_DOWN) {
        printf("KEY1 [事件]: 刚刚被按�? (DOWN) \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_UP) {
        printf("KEY1 [事件]: 刚刚被松开 (UP) \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_SHORT) {
        printf("KEY1 [事件]: 触发【短按�?(SHORT)! \r\n");
    } 
    else if (dev_key1.event == KEY_EVT_LONG) {
        printf("KEY1 [事件]: 触发【长按�?(LONG)! \r\n");
    }
}

void Key_Switch_Param_Edit(void) {
    // 1. KEY1 �̰��������л���ǰѡ�еĲ���
    if (dev_key1.event == KEY_EVT_SHORT) {
        current_param_idx++;
        if (current_param_idx >= PARAM_COUNT) {
            current_param_idx = 0; // Խ����лص�һ��
        }
    }

    // 2. KEY2 ���Σ��̰���С����������
    if (dev_key2.event == KEY_EVT_SHORT || dev_key2.event == KEY_EVT_LONG) {
        
        // ���� 5% �ı仯��������֤���ٱ仯 1.0 
        float delta = debug_params[current_param_idx] * 0.05f;
        if (delta < 1.0f) delta = 1.0f;

        if (dev_key2.event == KEY_EVT_SHORT) {
            debug_params[current_param_idx] -= delta;
        } 
        else if (dev_key2.event == KEY_EVT_LONG) {
            debug_params[current_param_idx] += delta;
        }

        // 3. ��ȫ�޷����� (��ֵֻ���� 0~255 ֮��)
        if (current_param_idx == 0) {
            if (debug_params[0] > 255.0f) debug_params[0] = 255.0f;
            if (debug_params[0] < 0.0f)   debug_params[0] = 0.0f;
        }
    }
}