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
Motor_Offsset_t motor_offset = {0};
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
static uint32_t flight_start_ms = 0; // [新增] 解锁时刻 (pit0_cnt)，用于全局飞行超时
// =================== 内部辅助函数 ===================
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = normal;
    flight_target.is_armed = 2;  // 0: 锁定, 1: 解锁, 2: 等待校准后解锁
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
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.094f, 0.018f, 0.073f, 0.0003f, 50, MAX_TILT_ANGLE , 7.0f);
    Nonline_PID_Init(&pid_image_y, 0.094f, 0.018f, 0.073f, 0.0003f, 50, MAX_TILT_ANGLE , 7.0f);

    // 光流速度环 (低高度定点外环, 输出目标倾角给角度环/角速度环)
    PID_Init(&pid_opt_vel_x, 0.10f, 0.02f, 0.0f, 100, MAX_TILT_ANGLE, 7.0f);
    PID_Init(&pid_opt_vel_y, 0.10f, 0.02f, 0.0f, 100, MAX_TILT_ANGLE, 7.0f);

}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    flight_target.cur_state = normal;
    car_en = 1;
    flight_target.start_up_scale = 0.0f;
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

    // 锁定当前航向为目标航向，防止解锁即转圈
    flight_target.target_yaw = imu_data.yaw;
    flight_target.height = imu_data.z;
    flight_start_ms = dataC.pit0_cnt; // [新增] 记录解锁时刻，用于全局飞行超时
}

void Flight_Request_Landing(void) {
    if (flight_target.is_armed != 1 || flight_target.cur_state != normal) return;

    car_en = 0;
    dataC.camera_offset_y = CAM_OFFSET_Y + LANDING_CAM_OFFSET_Y_DELTA;
    landing_start_height = flight_target.height;
    landing_start_ms = dataC.pit0_cnt;
    landing_tof_seq = tof_update_seq;
    flight_target.cur_state = pre_landing;
}

void Flight_Lock(void) {
    flight_target.is_armed = 0;
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

// =================== 内部功能模块 (Static) ===================

/**
 * @brief 飞行状态机更新
 * 处理自动解锁、飞行模式切换、目标高度设定
 */
static void Flight_State_Update(void) {
    // 1. 自动解锁逻辑 (等待IMU校准完成后自动解锁)
    if (flight_target.is_armed == 2) {
        if (imu_data.is_calibrated) {
            Flight_Unlock();
        }
        // 若未校准，保持 is_armed=2，电机输出为0
    }

    // 2. 仅用降落请求之后的新ToF数据判断触地关停
    if (flight_target.cur_state == pre_landing && tof_update_seq != landing_tof_seq) {
        landing_tof_seq = tof_update_seq;
        if (imu_data.z < LANDING_CUTOFF_HEIGHT_CM) {
            flight_target.cur_state = landing;
        }
    }

    // 3. 根据状态设定目标高度及特殊行为
    switch (flight_target.cur_state) {
        case normal:
            // 起飞缓启动改为对目标高度缩放, 不再直接缩放 PWM 输出
            flight_target.target_height = TARGET_HEIGHT_CM * flight_target.start_up_scale;
            if (flight_target.is_armed == 1) {
                if (flight_target.start_up_scale < 1.0f) {
                    flight_target.start_up_scale += CTRL_DT_CTLOOP * 0.4f;  // 约5秒加满
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
        case pre_landing:
        {
            uint32_t elapsed_ms = dataC.pit0_cnt - landing_start_ms;
            flight_target.target_height = 0.0f;
            if (elapsed_ms < LANDING_DESCENT_TIME_MS) {
                flight_target.height = landing_start_height *
                    (1.0f - (float)elapsed_ms / (float)LANDING_DESCENT_TIME_MS);
            } else {
                flight_target.height = 0.0f;
            }
            break;
        }
        case landing:
            flight_target.target_height = 0.0f;
            flight_target.height = 0.0f;
            flight_target.start_up_scale = 0.0f;
            car_en = 0;
            Flight_Lock();
            break;
        default:
            break;
    }

    // 高度目标平滑 (时间常数约 1s, 独立于 TOF 数据速率)
    if (flight_target.cur_state == normal) {
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

    // 根据机架前后/左右静态偏差叠加电机补偿。
    motor_offset.lf = (int16_t)( PITCH_OFFSET + ROLL_OFFSET);
    motor_offset.rf = (int16_t)( PITCH_OFFSET - ROLL_OFFSET);
    motor_offset.lb = (int16_t)(-PITCH_OFFSET + ROLL_OFFSET);
    motor_offset.rb = (int16_t)(-PITCH_OFFSET - ROLL_OFFSET);

    // 校准完成后直接用各电机平均 PWM 作为基准, 再叠加高度环修正量;
    // 未完成时 Calibration_Get_Hover_PWM()==HOVER_THROTTLE, 故与当前逻辑完全一致
    int16_t base_lf = (int16_t)(Calibration_Get_Hover_PWM(0) + base_throttle);
    int16_t base_rf = (int16_t)(Calibration_Get_Hover_PWM(1) + base_throttle);
    int16_t base_lb = (int16_t)(Calibration_Get_Hover_PWM(2) + base_throttle);
    int16_t base_rb = (int16_t)(Calibration_Get_Hover_PWM(3) + base_throttle);

    // 混控算法 (X型四旋翼)
    // LF (左前, CW): Base + Pitch + Roll - Yaw
    motor_out.lf = (int16_t)((base_lf + out_pitch + out_roll + out_yaw + motor_offset.lf) * flight_target.output_scale);

    // RF (右前, CCW): Base + Pitch - Roll + Yaw
    motor_out.rf = (int16_t)((base_rf + out_pitch - out_roll - out_yaw + motor_offset.rf) * flight_target.output_scale);

    // LB (左后, CCW): Base - Pitch + Roll + Yaw
    motor_out.lb = (int16_t)((base_lb - out_pitch + out_roll - out_yaw + motor_offset.lb) * flight_target.output_scale);

    // RB (右后, CW): Base - Pitch - Roll - Yaw
    motor_out.rb = (int16_t)((base_rb - out_pitch - out_roll + out_yaw + motor_offset.rb) * flight_target.output_scale);

    // 输出限幅
    // 查找最大值，若超限则四颗电机等比例缩放，保持推力矢量方向不变
    int16_t max_motor = motor_out.lf;
    if (motor_out.rf > max_motor) max_motor = motor_out.rf;
    if (motor_out.lb > max_motor) max_motor = motor_out.lb;
    if (motor_out.rb > max_motor) max_motor = motor_out.rb;

    if (max_motor > MAX_PWM) {
        float scale = (float)MAX_PWM / max_motor;
        motor_out.lf = (int16_t)(motor_out.lf * scale);
        motor_out.rf = (int16_t)(motor_out.rf * scale);
        motor_out.lb = (int16_t)(motor_out.lb * scale);
        motor_out.rb = (int16_t)(motor_out.rb * scale);
    }

    // 下限保护 (电机不能反转)
    if (motor_out.lf < MIN_PWM) motor_out.lf = MIN_PWM;
    if (motor_out.rf < MIN_PWM) motor_out.rf = MIN_PWM;
    if (motor_out.lb < MIN_PWM) motor_out.lb = MIN_PWM;
    if (motor_out.rb < MIN_PWM) motor_out.rb = MIN_PWM;
}

// =================== 主控制循环 ===================
void Flight_Control_Loop(void) {
    // 1. 状态机更新 (包含自动解锁、模式切换、目标高度设定)
    Flight_State_Update();

    // 2. 角度环控制 (计算期望角速度)
    Flight_Control_Angle();

    // 3. 角速度环控制 (计算姿态修正量)
    Flight_Control_Rate(&motor_out.roll, &motor_out.pitch, &motor_out.yaw);

    // 4. 锁定检查
    if (flight_target.is_armed == 0) {
        Flight_Lock();
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

    if (flight_target.is_armed == 1) {
        // 注意：UART 驱动通常直接接受逻辑占空比（如 0-10000），不再需要 PWM 的 4000 偏置
        small_driver_set_duty(motor_out.lf, motor_out.lb, motor_out.rb,motor_out.rf);
        // small_driver_set_duty(2000, 2000, 2000, 2000);
    } else {
        motor_out.rf = 0;
        motor_out.rb = 0;
        motor_out.lf = 0;
        motor_out.lb = 0;
        small_driver_set_duty(0, 0, 0, 0);
    }
}
