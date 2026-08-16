#include "calibration.h"

#if CALIBRATION_ENABLE

// =================== 校准状态 ===================
// 0: 等待触发, 1: 到达高度后稳定等待, 2: 采样中, 3: 校准完成
static uint8_t  calib_state = 0;
static uint32_t state_start_ms = 0;

// 采样累加
static float    sum_roll = 0.0f;
static float    sum_pitch = 0.0f;
static int32_t  sum_motor[4] = {0, 0, 0, 0};
static uint32_t sample_cnt = 0;

// 校准结果
static float    roll_offset = 0.0f;
static float    pitch_offset = 0.0f;
static int16_t  hover_pwm[4] = {
    HOVER_THROTTLE, HOVER_THROTTLE, HOVER_THROTTLE, HOVER_THROTTLE
};

// 蜂鸣器非阻塞控制
static uint8_t  buzzer_on = 0;
static uint32_t buzzer_until_ms = 0;

static void Buzzer_Start(void) {
    gpio_high(BUZZER_PIN);
    buzzer_on = 1;
    buzzer_until_ms = dataC.pit0_cnt + 100U;
}

#endif // CALIBRATION_ENABLE

void Calibration_Init(void) {
#if CALIBRATION_ENABLE
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    calib_state = 0;
    state_start_ms = 0;
    sum_roll = 0.0f;
    sum_pitch = 0.0f;
    sum_motor[0] = 0; sum_motor[1] = 0; sum_motor[2] = 0; sum_motor[3] = 0;
    sample_cnt = 0;
    roll_offset = 0.0f;
    pitch_offset = 0.0f;
    hover_pwm[0] = HOVER_THROTTLE;
    hover_pwm[1] = HOVER_THROTTLE;
    hover_pwm[2] = HOVER_THROTTLE;
    hover_pwm[3] = HOVER_THROTTLE;
    buzzer_on = 0;
    buzzer_until_ms = 0;
#endif
}

void Calibration_Update(void) {
#if CALIBRATION_ENABLE
    // 蜂鸣器 100ms 非阻塞关闭
    if (buzzer_on && dataC.pit0_cnt >= buzzer_until_ms) {
        gpio_low(BUZZER_PIN);
        buzzer_on = 0;
    }

    switch (calib_state) {
        case 0: // 等待起飞到达校准高度
            if (current_drone_state == DRONE_STATE_NORMAL_FLIGHT &&
                flight_target.is_armed == 1 &&
                imu_data.z >= TARGET_HEIGHT_CM - CALIB_START_HEIGHT_OFFSET_CM) {
                calib_state = 1;
                state_start_ms = dataC.pit0_cnt;
            }
            break;

        case 1: // 非阻塞等待 3s, 让无人机稳定平飞定点
            if (dataC.pit0_cnt - state_start_ms >= CALIB_STABLE_WAIT_MS) {
                calib_state = 2;
                state_start_ms = dataC.pit0_cnt;
                sum_roll = 0.0f;
                sum_pitch = 0.0f;
                sum_motor[0] = 0; sum_motor[1] = 0; sum_motor[2] = 0; sum_motor[3] = 0;
                sample_cnt = 0;
                Buzzer_Start(); // 进入校准
            }
            break;

        case 2: // 1s 采样, 期间其它功能正常
            sum_roll += imu_data.roll;
            sum_pitch += imu_data.pitch;
            sum_motor[0] += motor_out.lf;
            sum_motor[1] += motor_out.rf;
            sum_motor[2] += motor_out.lb;
            sum_motor[3] += motor_out.rb;
            sample_cnt++;

            if (dataC.pit0_cnt - state_start_ms >= CALIB_SAMPLE_MS && sample_cnt > 0U) {
                roll_offset = sum_roll / (float)sample_cnt;
                pitch_offset = sum_pitch / (float)sample_cnt;
                hover_pwm[0] = (int16_t)(sum_motor[0] / (int32_t)sample_cnt);
                hover_pwm[1] = (int16_t)(sum_motor[1] / (int32_t)sample_cnt);
                hover_pwm[2] = (int16_t)(sum_motor[2] / (int32_t)sample_cnt);
                hover_pwm[3] = (int16_t)(sum_motor[3] / (int32_t)sample_cnt);

                // 零点变化后清除角度环积分, 避免旧积分在新零点下造成偏置
                Nonline_PID_Reset(&pid_roll);
                Nonline_PID_Reset(&pid_pitch);
                calib_state = 3;
                Buzzer_Start(); // 校准完成
            }
            break;

        default: // 3: 已完成, 不再动作
            break;
    }
#endif
}

float Calibration_Get_Roll_Offset(void) {
#if CALIBRATION_ENABLE
    return roll_offset;
#else
    return 0.0f;
#endif
}

float Calibration_Get_Pitch_Offset(void) {
#if CALIBRATION_ENABLE
    return pitch_offset;
#else
    return 0.0f;
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
        return hover_pwm[motor_index];
    }
    return HOVER_THROTTLE;
#else
    return HOVER_THROTTLE;
#endif
}

uint8_t Calibration_Is_Complete(void) {
#if CALIBRATION_ENABLE
    return (calib_state == 3U) ? 1U : 0U;
#else
    return 0U;
#endif
}