#include "calibration.h"

#if CALIBRATION_ENABLE

// =================== 校准状态管理结构体 ===================
typedef struct {
    Calib_State_e state;
    uint32_t      state_start_ms;
    uint32_t      sample_cnt;
    float         sum_roll;
    float         sum_pitch;
    int32_t       sum_motor[4];
    
    float         roll_offset;
    float         pitch_offset;
    int16_t       hover_pwm[4];
    
    uint8_t       buzzer_on;
    uint32_t      buzzer_until_ms;
} Calibration_Ctrl_t;

static Calibration_Ctrl_t calib_ctrl = {
    .state = CALIB_STATE_WAIT_TRIGGER,
    .state_start_ms = 0,
    .sample_cnt = 0,
    .sum_roll = 0.0f,
    .sum_pitch = 0.0f,
    .sum_motor = {0, 0, 0, 0},
    .roll_offset = INIT_ROLL_OFFSET_DEG,
    .pitch_offset = INIT_PITCH_OFFSET_DEG,
    .hover_pwm = {INIT_HOVER_PWM_LF, INIT_HOVER_PWM_RF, INIT_HOVER_PWM_LB, INIT_HOVER_PWM_RB},
    .buzzer_on = 0,
    .buzzer_until_ms = 0
};

static void Buzzer_Start(void) {
    gpio_high(BUZZER_PIN);
    calib_ctrl.buzzer_on = 1;
    calib_ctrl.buzzer_until_ms = dataC.pit0_cnt + CALIB_BUZZER_DURATION_MS;
}

#endif // CALIBRATION_ENABLE

void Calibration_Init(void) {
#if CALIBRATION_ENABLE
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    calib_ctrl.state = CALIB_STATE_WAIT_TRIGGER;
    calib_ctrl.state_start_ms = 0;
    calib_ctrl.sum_roll = 0.0f;
    calib_ctrl.sum_pitch = 0.0f;
    calib_ctrl.sum_motor[0] = 0; calib_ctrl.sum_motor[1] = 0; calib_ctrl.sum_motor[2] = 0; calib_ctrl.sum_motor[3] = 0;
    calib_ctrl.sample_cnt = 0;
    calib_ctrl.roll_offset = INIT_ROLL_OFFSET_DEG;
    calib_ctrl.pitch_offset = INIT_PITCH_OFFSET_DEG;
    calib_ctrl.hover_pwm[0] = INIT_HOVER_PWM_LF;
    calib_ctrl.hover_pwm[1] = INIT_HOVER_PWM_RF;
    calib_ctrl.hover_pwm[2] = INIT_HOVER_PWM_LB;
    calib_ctrl.hover_pwm[3] = INIT_HOVER_PWM_RB;
    calib_ctrl.buzzer_on = 0;
    calib_ctrl.buzzer_until_ms = 0;

    // 校准完成前放大最外环积分限幅 (抗物理偏置)
    pid_image_x.max_i = IMAGE_PID_MAX_I_CALIB;
    pid_image_y.max_i = IMAGE_PID_MAX_I_CALIB;
#else
    pid_image_x.max_i = IMAGE_PID_MAX_I_NORMAL;
    pid_image_y.max_i = IMAGE_PID_MAX_I_NORMAL;
#endif
}

/**
 * @brief 在无人机重新解锁/起飞时重置校准状态机
 */
void Calibration_Reset(void) {
#if CALIBRATION_ENABLE
    calib_ctrl.state = CALIB_STATE_WAIT_TRIGGER;
    calib_ctrl.state_start_ms = 0;
    calib_ctrl.sum_roll = 0.0f;
    calib_ctrl.sum_pitch = 0.0f;
    calib_ctrl.sum_motor[0] = 0; calib_ctrl.sum_motor[1] = 0; calib_ctrl.sum_motor[2] = 0; calib_ctrl.sum_motor[3] = 0;
    calib_ctrl.sample_cnt = 0;
    calib_ctrl.roll_offset = INIT_ROLL_OFFSET_DEG;
    calib_ctrl.pitch_offset = INIT_PITCH_OFFSET_DEG;
    calib_ctrl.hover_pwm[0] = INIT_HOVER_PWM_LF;
    calib_ctrl.hover_pwm[1] = INIT_HOVER_PWM_RF;
    calib_ctrl.hover_pwm[2] = INIT_HOVER_PWM_LB;
    calib_ctrl.hover_pwm[3] = INIT_HOVER_PWM_RB;
    calib_ctrl.buzzer_on = 0;
    calib_ctrl.buzzer_until_ms = 0;
    gpio_low(BUZZER_PIN);

    // 重新进入校准流程: 放大最外环积分限幅
    pid_image_x.max_i = IMAGE_PID_MAX_I_CALIB;
    pid_image_y.max_i = IMAGE_PID_MAX_I_CALIB;
#endif
}

void Calibration_Update(void) {
#if CALIBRATION_ENABLE
    // 蜂鸣器 100ms 非阻塞关闭
    if (calib_ctrl.buzzer_on && dataC.pit0_cnt >= calib_ctrl.buzzer_until_ms) {
        gpio_low(BUZZER_PIN);
        calib_ctrl.buzzer_on = 0;
    }

    switch (calib_ctrl.state) {
        case CALIB_STATE_WAIT_TRIGGER: // 等待起飞到达校准高度
            if (current_drone_state == DRONE_STATE_NORMAL_FLIGHT &&
                flight_target.is_armed == ARM_STATE_ARMED &&
                imu_data.z >= TARGET_HEIGHT_CM - CALIB_START_HEIGHT_OFFSET_CM) {
                calib_ctrl.state = CALIB_STATE_STABILIZING;
                calib_ctrl.state_start_ms = dataC.pit0_cnt;
            }
            break;

        case CALIB_STATE_STABILIZING: // 非阻塞等待 3s, 让无人机稳定平飞定点
            if (dataC.pit0_cnt - calib_ctrl.state_start_ms >= CALIB_STABLE_WAIT_MS) {
                calib_ctrl.state = CALIB_STATE_SAMPLING;
                calib_ctrl.state_start_ms = dataC.pit0_cnt;
                calib_ctrl.sum_roll = 0.0f;
                calib_ctrl.sum_pitch = 0.0f;
                calib_ctrl.sum_motor[0] = 0; calib_ctrl.sum_motor[1] = 0; calib_ctrl.sum_motor[2] = 0; calib_ctrl.sum_motor[3] = 0;
                calib_ctrl.sample_cnt = 0;
                Buzzer_Start(); // 进入校准提示音
            }
            break;

        case CALIB_STATE_SAMPLING: // 1s 采样, 期间其它功能正常
            calib_ctrl.sum_roll += imu_data.roll;
            calib_ctrl.sum_pitch += imu_data.pitch;
            calib_ctrl.sum_motor[0] += motor_out.lf;
            calib_ctrl.sum_motor[1] += motor_out.rf;
            calib_ctrl.sum_motor[2] += motor_out.lb;
            calib_ctrl.sum_motor[3] += motor_out.rb;
            calib_ctrl.sample_cnt++;

            if (dataC.pit0_cnt - calib_ctrl.state_start_ms >= CALIB_SAMPLE_MS && calib_ctrl.sample_cnt > 0U) {
                calib_ctrl.roll_offset = calib_ctrl.sum_roll / (float)calib_ctrl.sample_cnt;
                calib_ctrl.pitch_offset = calib_ctrl.sum_pitch / (float)calib_ctrl.sample_cnt;
                calib_ctrl.hover_pwm[0] = (int16_t)(calib_ctrl.sum_motor[0] / (int32_t)calib_ctrl.sample_cnt);
                calib_ctrl.hover_pwm[1] = (int16_t)(calib_ctrl.sum_motor[1] / (int32_t)calib_ctrl.sample_cnt);
                calib_ctrl.hover_pwm[2] = (int16_t)(calib_ctrl.sum_motor[2] / (int32_t)calib_ctrl.sample_cnt);
                calib_ctrl.hover_pwm[3] = (int16_t)(calib_ctrl.sum_motor[3] / (int32_t)calib_ctrl.sample_cnt);

                // 零点变化后清除角度环与视觉位置环积分, 恢复最外环积分限幅为正常值
                Nonline_PID_Reset(&pid_roll);
                Nonline_PID_Reset(&pid_pitch);
                pid_image_x.max_i = IMAGE_PID_MAX_I_NORMAL;
                pid_image_y.max_i = IMAGE_PID_MAX_I_NORMAL;
                Nonline_PID_Reset(&pid_image_x);
                Nonline_PID_Reset(&pid_image_y);

                calib_ctrl.state = CALIB_STATE_DONE;
                Buzzer_Start(); // 校准完成提示音

#if defined(CY_CORE_CM7_0)
                char calib_msg[128];
                sprintf(calib_msg, "[CALIB] roll_offset=%.2f, pitch_offset=%.2f, hover=[%d,%d,%d,%d]\r\n",
                        calib_ctrl.roll_offset, calib_ctrl.pitch_offset,
                        calib_ctrl.hover_pwm[0], calib_ctrl.hover_pwm[1],
                        calib_ctrl.hover_pwm[2], calib_ctrl.hover_pwm[3]);
                wireless_uart_send_string(calib_msg);
                printf("%s", calib_msg);
#endif
            }
            break;

        case CALIB_STATE_DONE:
        default: // 已完成, 不再动作
            break;
    }
#endif
}

float Calibration_Get_Roll_Offset(void) {
#if CALIBRATION_ENABLE
    return calib_ctrl.roll_offset;
#else
    return INIT_ROLL_OFFSET_DEG;
#endif
}

float Calibration_Get_Pitch_Offset(void) {
#if CALIBRATION_ENABLE
    return calib_ctrl.pitch_offset;
#else
    return INIT_PITCH_OFFSET_DEG;
#endif
}

float Calibration_Get_Corrected_Roll(void) {
    return imu_data.roll - Calibration_Get_Roll_Offset();
}

float Calibration_Get_Corrected_Pitch(void) {
    return imu_data.pitch - Calibration_Get_Pitch_Offset();
}

int16_t Calibration_Get_Hover_PWM(uint8_t motor_index) {
#if CALIBRATION_ENABLE
    if (motor_index < 4U) {
        return calib_ctrl.hover_pwm[motor_index];
    }
    return INIT_HOVER_PWM_LF;
#else
    switch (motor_index) {
        case 0: return INIT_HOVER_PWM_LF;
        case 1: return INIT_HOVER_PWM_RF;
        case 2: return INIT_HOVER_PWM_LB;
        case 3: return INIT_HOVER_PWM_RB;
        default: return INIT_HOVER_PWM_LF;
    }
#endif
}

uint8_t Calibration_Is_Complete(void) {
#if CALIBRATION_ENABLE
    return (calib_ctrl.state == CALIB_STATE_DONE) ? 1U : 0U;
#else
    return 1U;
#endif
}