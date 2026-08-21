#include "fly_ctrl.h"

// **************************** 三串级PID飞控架构 ****************************
//
//   Flight_Control_Loop() (1.25ms ISR调用)
//        │
//   ┌────┴────────────────────────────────────────────┐
//   │ 1. Flight_State_Update()    状态机/自动解锁/降落 │
//   │ 2. Flight_Control_Angle()   角度环 → 目标角速度  │  ← 外环: Nonline_PID
//   │ 3. Flight_Control_Rate()    角速度环 → 姿态修正  │  ← 内环: PID
//   │ 4. Flight_Control_Height()  高度环 → 基础油门    │  ← 串级: 位置→速度
//   │ 5. Flight_Motor_Mix()       混控 → 4路电机PWM   │  ← X型四旋翼混控
//   └─────────────────────────────────────────────────┘
//                               ↓
//                    motor_pwm_set() → UART驱动 → ESC
//
// 坐标系: 机体X=前, Y=右, Z=下 (NED)
// 电机映射(5.12a): LF=CW, RF=CCW, LB=CCW, RB=CW
// ******************************************************************************

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
// 定义 PID 对象
PID_t pid_height_vel;
PID_t pid_height_pos;
Nonline_PID_t pid_roll;
Nonline_PID_t pid_pitch;
Nonline_PID_t pid_yaw;

Nonline_PID_t pid_image_x;
Nonline_PID_t pid_image_y;
PID_t pid_g_roll;
PID_t pid_g_pitch;
PID_t pid_g_yaw;

PID_t pid_opt_vel_x;
PID_t pid_opt_vel_y;
static volatile uint32_t landing_start_ms = 0;
static volatile uint32_t landing_tof_seq = 0;
static volatile float landing_start_height = 0.0f;
static volatile uint32_t landing_cutoff_start_ms = 0; // [新增] 触地关停开始时刻 (ms)
static uint32_t flight_start_ms = 0; // [新增] 解锁时刻 (pit0_cnt)，用于全局飞行超时
static float mode_smooth_roll = 0.0f;      // 模式切换平滑中的目标横滚
static float mode_smooth_pitch = 0.0f;     // 模式切换平滑中的目标俯仰
static uint32_t mode_switch_start_ms = 0;  // 模式切换平滑起始时刻 (ms)
static uint32_t last_smooth_ms = 0;        // 模式切换平滑上次执行时刻 (ms)
static uint8_t mode_switch_pending = 0;    // 模式切换平滑进行中标志

// =================== 内部辅助函数 ===================
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = FLIGHT_STATE_NORMAL;
    flight_target.is_armed = ARM_STATE_WAITING_IMU_CALIB;  // 0: 锁定, 1: 解锁, 2: 等待校准后解锁
    flight_target.height = 0;
    flight_target.start_up_scale = 0.0f;
    flight_target.output_scale = 1.0f; // 默认缩放比例设为1

    // ----------- 初始化 PID 参数 -----------
    // 高度环
    PID_Init(&pid_height_pos, 0.7f, 0.4f, 0.0f, 30, 20, 40.0f);
    PID_Init(&pid_height_vel, 16.031f, 0.0f, 0.15f, 80, 2500, 15.0f);
    // 角度环a
    Nonline_PID_Init(&pid_roll, 9.328f, 0.239f, 0.0f, 0.05f, 20, 300, 40.0f);
    Nonline_PID_Init(&pid_pitch, 9.328f, 0.239f, 0.0f, 0.05f, 20, 300, 40.0f);
    Nonline_PID_Init(&pid_yaw, 1.5f, 0.33f, 0.0f, 0.0228f, 6, 45, 40.0f);

    //Nonline_PID_Init(&pid_image_yaw, 1.0f, 0.00f, 0.0f, 0.0f, 0, 60.0f, 4.0f);
    // 角速度环g
    PID_Init(&pid_g_roll, 2.764f, 3.327f, 0.115f, 120, 3500, 60.0f);
    PID_Init(&pid_g_pitch, 2.764f, 3.327f, 0.115f, 120, 3500, 60.0f);
    PID_Init(&pid_g_yaw, 6.1f, 1.32f, 0.00f, 150, 3500, 60.0f);
    // 视觉部分 (初始/校准完成前使用放大的积分限幅)
    Nonline_PID_Init(&pid_image_x, 0.094f, 0.018f, 0.073f, 0.0003f, IMAGE_PID_MAX_I_CALIB, MAX_TILT_ANGLE, 7.0f);
    Nonline_PID_Init(&pid_image_y, 0.094f, 0.018f, 0.073f, 0.0003f, IMAGE_PID_MAX_I_CALIB, MAX_TILT_ANGLE, 7.0f);

    // 光流速度环 (低高度定点外环, 输出目标倾角给角度环/角速度环)
    PID_Init(&pid_opt_vel_x, 0.10f, 0.02f, 0.0f, 100, MAX_TILT_ANGLE, 7.0f);
    PID_Init(&pid_opt_vel_y, 0.10f, 0.02f, 0.0f, 100, MAX_TILT_ANGLE, 7.0f);

    Flight_Attitude_Smoother_Reset();
}

void Flight_Unlock(void) {
    flight_target.is_armed = ARM_STATE_ARMED;
    flight_target.cur_state = FLIGHT_STATE_NORMAL;
    car_en = 1;
    flight_target.start_up_scale = 0.0f;
    flight_target.output_scale = 1.0f;
    dataC.camera_offset_y = CAM_OFFSET_Y;

    // 解锁瞬间重置积分，防止暴冲
    PID_Reset(&pid_height_vel);
    PID_Reset(&pid_height_pos);
    Nonline_PID_Reset(&pid_roll);
    Nonline_PID_Reset(&pid_pitch);
    Nonline_PID_Reset(&pid_yaw);
    PID_Reset(&pid_g_roll);
    PID_Reset(&pid_g_pitch);
    PID_Reset(&pid_g_yaw);
    Nonline_PID_Reset(&pid_image_x); // [新增] 重置视觉PID，防止积分累积
    Nonline_PID_Reset(&pid_image_y);
    PID_Reset(&pid_opt_vel_x); // [新增] 重置光流速度环PID
    PID_Reset(&pid_opt_vel_y);

    Calibration_Reset(); // [优化] 解锁/起飞时重置悬停校准状态机，允许执行新一轮校准
    Flight_Attitude_Smoother_Reset(); // 重置姿态平滑器

    // 锁定当前航向为目标航向，防止解锁即转圈
    flight_target.target_yaw = imu_data.yaw;
    flight_target.height = imu_data.z;
    flight_start_ms = dataC.pit0_cnt; // [新增] 记录解锁时刻，用于全局飞行超时
}

void Flight_Request_Landing(void) {
    if (flight_target.is_armed != ARM_STATE_ARMED || flight_target.cur_state != FLIGHT_STATE_NORMAL) return;

    car_en = 0;
    dataC.camera_offset_y = CAM_OFFSET_Y + LANDING_CAM_OFFSET_Y_DELTA;
    landing_start_height = flight_target.height;
    landing_start_ms = dataC.pit0_cnt;
    landing_tof_seq = tof_update_seq;
    flight_target.target_yaw = 360.0f; // [新增] 降落前设置目标偏航角为360度
    flight_target.cur_state = FLIGHT_STATE_PRE_LANDING;
}

void Flight_Lock(void) {
    flight_target.is_armed = ARM_STATE_DISARMED;
    motor_out.rf = 0;
    motor_out.rb = 0;
    motor_out.lb = 0;
    motor_out.lf = 0;
}

// 直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw) {
    flight_target.target_roll = Constrain_Float(roll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_pitch = Constrain_Float(pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_yaw = yaw;
}

// =================== 导航外环模式管理 (单点仲裁) ===================
static Nav_Mode_e current_nav_mode = NAV_MODE_ATTITUDE_HOLD;

Nav_Mode_e Flight_Nav_Mode_Update(float height_cm) {
    if (current_nav_mode == NAV_MODE_OPTICAL_FLOW) {
        // 已处于光流模式: 只有明显高于滞回上限才切回视觉
        if (height_cm >= (VISION_POSITION_MIN_HEIGHT_CM + VISION_POSITION_HYSTERESIS_CM)) {
            current_nav_mode = NAV_MODE_VISION_HOVER;
            OpticalFlow_Mode_Exit();
        }
    } else {
        // 未处于光流模式: 低于阈值立即切入光流
        if (height_cm < VISION_POSITION_MIN_HEIGHT_CM) {
            current_nav_mode = NAV_MODE_OPTICAL_FLOW;
            Image_Hover_Reset_For_OpticalFlow();
            OpticalFlow_Mode_Enter();
        } else {
            current_nav_mode = NAV_MODE_VISION_HOVER;
        }
    }
    return current_nav_mode;
}

Nav_Mode_e Flight_Get_Nav_Mode(void) {
    return current_nav_mode;
}

/**
 * @brief 重置模式切换姿态平滑器
 */
void Flight_Attitude_Smoother_Reset(void) {
    mode_smooth_roll = flight_target.target_roll;
    mode_smooth_pitch = flight_target.target_pitch;
    mode_switch_start_ms = dataC.pit0_cnt;
    last_smooth_ms = 0;
    mode_switch_pending = 1;
}

/**
 * @brief 带模式切换平滑的目标姿态设置
 */
void Flight_Set_Target_Attitude_Smoothed(float roll, float pitch, float yaw, float dt_sec) {
    (void)dt_sec; // 平滑步长按实际时钟间隔计算
    if (mode_switch_pending) {
        uint32_t now_ms = dataC.pit0_cnt;
        uint32_t step_ms = (last_smooth_ms == 0) ? 10U : (now_ms - last_smooth_ms);
        last_smooth_ms = now_ms;
        float step_dt = step_ms * 0.001f;
        if (step_dt < 0.001f) step_dt = 0.001f;
        if (step_dt > 0.05f) step_dt = 0.05f;
        uint32_t elapsed = now_ms - mode_switch_start_ms;
        float diff_roll = roll - mode_smooth_roll;
        float diff_pitch = pitch - mode_smooth_pitch;
        // 已平滑到位且超过最短平滑时间后退出平滑
        if (elapsed >= MODE_SWITCH_SMOOTH_MS && fabsf(diff_roll) < 0.5f && fabsf(diff_pitch) < 0.5f) {
            mode_switch_pending = 0;
            last_smooth_ms = 0;
        } else {
            // 速率限制: 全量程倾角在 MODE_SWITCH_SMOOTH_MS 内线性过渡, 避免切换阶跃
            float max_step = (MAX_TILT_ANGLE * step_dt) / ((float)MODE_SWITCH_SMOOTH_MS * 0.001f);
            if (diff_roll > max_step) diff_roll = max_step;
            else if (diff_roll < -max_step) diff_roll = -max_step;
            if (diff_pitch > max_step) diff_pitch = max_step;
            else if (diff_pitch < -max_step) diff_pitch = -max_step;
            mode_smooth_roll += diff_roll;
            mode_smooth_pitch += diff_pitch;
            Set_Target_Attitude(mode_smooth_roll, mode_smooth_pitch, yaw);
            return;
        }
    }
    Set_Target_Attitude(roll, pitch, yaw);
}

// =================== 内部功能模块 (Static) ===================

/**
 * @brief 飞行状态机更新
 * 处理自动解锁、飞行模式切换、目标高度设定
 */
static void Flight_State_Update(void) {
    // 1. 自动解锁逻辑 (等待IMU校准完成后自动解锁)
    if (flight_target.is_armed == ARM_STATE_WAITING_IMU_CALIB) {
        if (imu_data.is_calibrated) {
            Flight_Unlock();
        }
        // 若未校准，保持 is_armed=ARM_STATE_WAITING_IMU_CALIB，电机输出为0
    }

    // 2. 仅用降落请求之后的新ToF数据判断触地关停
    if (flight_target.cur_state == FLIGHT_STATE_PRE_LANDING && tof_update_seq != landing_tof_seq) {
        landing_tof_seq = tof_update_seq;
        if (imu_data.z < LANDING_CUTOFF_HEIGHT_CM) {
            flight_target.cur_state = FLIGHT_STATE_LANDING;
            landing_cutoff_start_ms = dataC.pit0_cnt;
            car_en = 0;
        }
    }

    // 3. 根据状态设定目标高度及特殊行为
    switch (flight_target.cur_state) {
        case FLIGHT_STATE_NORMAL:
            // 目标高度随 start_up_scale 缓升，保证高度环闭环无阶跃
            flight_target.target_height = TARGET_HEIGHT_CM * flight_target.start_up_scale;
            if (flight_target.is_armed == ARM_STATE_ARMED) {
                if (flight_target.start_up_scale < 1.0f) {
                    flight_target.start_up_scale += CTRL_DT_CTLOOP * 0.4f;  // 约2.5秒加满
                    if (flight_target.start_up_scale > 1.0f) {
                        flight_target.start_up_scale = 1.0f;
                    }
                }
                // [新增] 全局飞行超时检查
                if (flight_start_ms > 0 && dataC.pit0_cnt - flight_start_ms >= FLIGHT_TIMEOUT_MS) {
                    // Flight_Request_Landing(); // 暂时关闭70秒自动降落
                }
            }
            break;
        case FLIGHT_STATE_PRE_LANDING:
        {
            uint32_t elapsed_ms = dataC.pit0_cnt - landing_start_ms;
            flight_target.target_height = 0.0f;
            if (elapsed_ms < LANDING_ROTATE_WAIT_MS) {
                // 等待旋转360度完成，保持原高度
                flight_target.height = landing_start_height;
            } else if (elapsed_ms < LANDING_ROTATE_WAIT_MS + LANDING_DESCENT_TIME_MS) {
                // 旋转完成后线性下降至0
                uint32_t descent_elapsed = elapsed_ms - LANDING_ROTATE_WAIT_MS;
                flight_target.height = landing_start_height *
                    (1.0f - (float)descent_elapsed / (float)LANDING_DESCENT_TIME_MS);
            } else {
                flight_target.height = 0.0f;
            }
            break;
        }
        case FLIGHT_STATE_LANDING:
        {
            flight_target.target_height = 0.0f;
            flight_target.height = 0.0f;
            flight_target.start_up_scale = 0.0f;
            car_en = 0;

            // 触地关停阶段: 在 LANDING_CUTOFF_RAMP_MS 内将 PWM output_scale 线性平滑减小到 0
            uint32_t elapsed_ms = dataC.pit0_cnt - landing_cutoff_start_ms;
            if (elapsed_ms < LANDING_CUTOFF_RAMP_MS) {
                flight_target.output_scale = 1.0f - (float)elapsed_ms / (float)LANDING_CUTOFF_RAMP_MS;
            } else {
                flight_target.output_scale = 0.0f;
                Flight_Lock();
            }
            break;
        }
        default:
            break;
    }

    // 高度目标平滑 (时间常数约 1s, 独立于 TOF 数据速率)
    if (flight_target.cur_state == FLIGHT_STATE_NORMAL) {
        const float height_alpha = CTRL_DT_CTLOOP;
        flight_target.height = flight_target.height * (1.0f - height_alpha) + flight_target.target_height * height_alpha;
    }
}

/**
 * @brief 高度环修正量输出 (倾角补偿)
 * @return 修正量 (tof_base_throttle × 倾角补偿); 悬停基准由各电机校准 PWM 在混控中叠加
 * @note  高度 PID 已移至 tof_update() 内部, 响应数据就绪 + 实测 dt
 *        此处仅每 1.25ms 更新倾角补偿以匹配当前姿态
 */
static int16_t Flight_Control_Height(void) {
    float roll_rad = Calibration_Get_Corrected_Roll() * (PI / 180.0f);
    float pitch_rad = Calibration_Get_Corrected_Pitch() * (PI / 180.0f);
    float cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
    float compensation_factor = 1.0f;
    if (cos_tilt > 0.1f) {
        compensation_factor = 1.0f / cos_tilt;
    }
    compensation_factor = Constrain_Float(compensation_factor, 1.0f, 1.5f);
    return (int16_t)(tof_base_throttle * compensation_factor);
}

void Flight_Control_Angle(void) {
    // 1. 计算误差 (绝对系); 校准完成后以记录的 roll/pitch 平均值为新零点
    float roll_error = flight_target.target_roll - Calibration_Get_Corrected_Roll();
    float pitch_error = flight_target.target_pitch - Calibration_Get_Corrected_Pitch();
    float yaw_error = flight_target.target_yaw - imu_data.yaw;

    // 2. PID 计算 (输出即视为机体角速度目标，基于小角度假设)
    flight_target.target_g_roll = Nonline_PID_Calculate(&pid_roll, roll_error, CTRL_DT_CTLOOP);
    flight_target.target_g_pitch = Nonline_PID_Calculate(&pid_pitch, pitch_error, CTRL_DT_CTLOOP);
    flight_target.target_g_yaw = Nonline_PID_Calculate(&pid_yaw, yaw_error, CTRL_DT_CTLOOP);
}

/**
 * @brief 角速度环控制 (PID)
 * @param out_roll/pitch/yaw 输出的控制量指针
 */
static void Flight_Control_Rate(float *out_roll, float *out_pitch, float *out_yaw) {
    // Roll PID
    float roll_err = flight_target.target_g_roll - imu_data.groll;
    *out_roll = PID_Calculate(&pid_g_roll, roll_err, CTRL_DT_CTLOOP);
    
    // Pitch PID
    float pitch_err = flight_target.target_g_pitch - imu_data.gpitch;
    *out_pitch = PID_Calculate(&pid_g_pitch, pitch_err, CTRL_DT_CTLOOP);

    // Yaw PID
    float yaw_err = flight_target.target_g_yaw - imu_data.gyaw;
   
    *out_yaw = PID_Calculate(&pid_g_yaw, yaw_err, CTRL_DT_CTLOOP);
}

/**
 * @brief 电机混控与输出
 * @param base_throttle 基础油门
 * @param out_roll/pitch/yaw 三轴控制量
 */
static void Flight_Motor_Mix(int16_t base_throttle, float out_roll, float out_pitch, float out_yaw) {
    // 基础悬停油门随 start_up_scale 从怠速 (MIN_PWM) 平滑上升至悬停油门，消除电调起转阶跃
    float startup = flight_target.start_up_scale;
    int16_t hover_lf = Calibration_Get_Hover_PWM(0);
    int16_t hover_rf = Calibration_Get_Hover_PWM(1);
    int16_t hover_lb = Calibration_Get_Hover_PWM(2);
    int16_t hover_rb = Calibration_Get_Hover_PWM(3);

    int16_t base_lf = (int16_t)(MIN_PWM + (hover_lf - MIN_PWM) * startup + base_throttle);
    int16_t base_rf = (int16_t)(MIN_PWM + (hover_rf - MIN_PWM) * startup + base_throttle);
    int16_t base_lb = (int16_t)(MIN_PWM + (hover_lb - MIN_PWM) * startup + base_throttle);
    int16_t base_rb = (int16_t)(MIN_PWM + (hover_rb - MIN_PWM) * startup + base_throttle);

    float total_scale = flight_target.output_scale;

    // 混控算法 (X型四旋翼)
    // LF (左前, CW): Base + Pitch + Roll - Yaw
    motor_out.lf = (int16_t)((base_lf + out_pitch + out_roll + out_yaw) * total_scale);

    // RF (右前, CCW): Base + Pitch - Roll + Yaw
    motor_out.rf = (int16_t)((base_rf + out_pitch - out_roll - out_yaw) * total_scale);

    // LB (左后, CCW): Base - Pitch + Roll + Yaw
    motor_out.lb = (int16_t)((base_lb - out_pitch + out_roll - out_yaw) * total_scale);

    // RB (右后, CW): Base - Pitch - Roll - Yaw
    motor_out.rb = (int16_t)((base_rb - out_pitch - out_roll + out_yaw) * total_scale);

    // 输出限幅与映射: 查找最大值与最小值，若越界则映射到 [current_min_pwm, MAX_PWM]，保持推力矢量方向大致不变
    int16_t current_min_pwm = (flight_target.cur_state == FLIGHT_STATE_LANDING) ?
                              (int16_t)(MIN_PWM * flight_target.output_scale) : MIN_PWM;

    int16_t max_motor = motor_out.lf;
    int16_t min_motor = motor_out.lf;
    if (motor_out.rf > max_motor) max_motor = motor_out.rf;
    if (motor_out.lb > max_motor) max_motor = motor_out.lb;
    if (motor_out.rb > max_motor) max_motor = motor_out.rb;

    if (motor_out.rf < min_motor) min_motor = motor_out.rf;
    if (motor_out.lb < min_motor) min_motor = motor_out.lb;
    if (motor_out.rb < min_motor) min_motor = motor_out.rb;

    if (max_motor > MAX_PWM || min_motor < current_min_pwm) {
        if (max_motor > min_motor) {
            float span = (float)(max_motor - min_motor);
            float target_range = (float)(MAX_PWM - current_min_pwm);
            if (span > target_range) {
                // 差值跨度超过可用区间: 线性归一化压缩到 [current_min_pwm, MAX_PWM]
                float scale = target_range / span;
                motor_out.lf = (int16_t)(current_min_pwm + (motor_out.lf - min_motor) * scale);
                motor_out.rf = (int16_t)(current_min_pwm + (motor_out.rf - min_motor) * scale);
                motor_out.lb = (int16_t)(current_min_pwm + (motor_out.lb - min_motor) * scale);
                motor_out.rb = (int16_t)(current_min_pwm + (motor_out.rb - min_motor) * scale);
            } else if (max_motor > MAX_PWM) {
                // 上限超限但跨度可容纳: 整体平移下调, 保持差分力矩完全不变
                int16_t shift = max_motor - MAX_PWM;
                motor_out.lf -= shift;
                motor_out.rf -= shift;
                motor_out.lb -= shift;
                motor_out.rb -= shift;
            } else if (min_motor < current_min_pwm) {
                // 下限超限但跨度可容纳: 整体平移上调, 保持差分力矩完全不变
                int16_t shift = current_min_pwm - min_motor;
                motor_out.lf += shift;
                motor_out.rf += shift;
                motor_out.lb += shift;
                motor_out.rb += shift;
            }
        } else {
            // 四个电机数值完全相同
            int16_t val = (max_motor > MAX_PWM) ? MAX_PWM : ((min_motor < current_min_pwm) ? current_min_pwm : max_motor);
            motor_out.lf = val;
            motor_out.rf = val;
            motor_out.lb = val;
            motor_out.rb = val;
        }
    }

    // 最终边界保护 (确保绝对处于 [current_min_pwm, MAX_PWM])
    if (motor_out.lf > MAX_PWM) motor_out.lf = MAX_PWM;
    if (motor_out.lf < current_min_pwm) motor_out.lf = current_min_pwm;
    if (motor_out.rf > MAX_PWM) motor_out.rf = MAX_PWM;
    if (motor_out.rf < current_min_pwm) motor_out.rf = current_min_pwm;
    if (motor_out.lb > MAX_PWM) motor_out.lb = MAX_PWM;
    if (motor_out.lb < current_min_pwm) motor_out.lb = current_min_pwm;
    if (motor_out.rb > MAX_PWM) motor_out.rb = MAX_PWM;
    if (motor_out.rb < current_min_pwm) motor_out.rb = current_min_pwm;
}

// =================== 主控制循环 ===================
void Flight_Control_Loop(void) {
    // 1. 状态机更新 (包含自动解锁、模式切换、目标高度设定)
    Flight_State_Update();

    // 2. 角度环控制 (计算期望角速度)
    Flight_Control_Angle();

    // 3. 角速度环控制 (计算姿态修正量)
    Flight_Control_Rate(&motor_out.roll, &motor_out.pitch, &motor_out.yaw);

    // 4. 解锁状态检查 (未解锁或等待校准时清零电机输出并退出，不破坏 WAITING_IMU_CALIB 状态)
    if (flight_target.is_armed != ARM_STATE_ARMED) {
        motor_out.lf = 0;
        motor_out.rf = 0;
        motor_out.lb = 0;
        motor_out.rb = 0;
        return;
    }

    // 5. 高度环控制 (计算基础油门)
    int16_t base_throttle = Flight_Control_Height();

    // 6. 电机混控与输出
    Flight_Motor_Mix(base_throttle, motor_out.roll, motor_out.pitch, motor_out.yaw);
}

void motor_pwm_set() {
    if (current_drone_state == DRONE_STATE_DEBUG) {
        small_driver_set_duty(0, 0, 0, 0);
        return;
    }

    if (flight_target.is_armed == ARM_STATE_ARMED) {
        // 注意：UART 驱动通常直接接受逻辑占空比（如 0-10000），不再需要 PWM 的 4000 偏置
        small_driver_set_duty(motor_out.lf, motor_out.lb, motor_out.rb, motor_out.rf);
        // small_driver_set_duty(2000, 2000, 2000, 2000);
    } else {
        motor_out.rf = 0;
        motor_out.rb = 0;
        motor_out.lf = 0;
        motor_out.lb = 0;
        small_driver_set_duty(0, 0, 0, 0);
    }
}
