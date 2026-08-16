#include "opticalflow_ctrl.h"

// =================== 光流速度环内部状态 ===================
static uint8_t opt_flow_mode_active = 0;   // 低高度光流速度环激活标志
static uint32_t last_opt_flow_ms = 0;      // 光流速度环上次计算时刻 (ms)
static uint32_t mode_switch_start_ms = 0;  // 模式切换平滑起始时刻 (ms)
static uint32_t last_smooth_ms = 0;        // 模式切换平滑上次执行时刻 (ms)
static float mode_smooth_roll = 0.0f;      // 模式切换平滑中的目标横滚
static float mode_smooth_pitch = 0.0f;     // 模式切换平滑中的目标俯仰
static uint8_t mode_switch_pending = 0;    // 模式切换平滑进行中标志

// =================== 内部辅助函数 ===================

/**
 * @brief 判断当前应使用光流速度环还是视觉环 (带滞回)
 */
static uint8_t OpticalFlow_Mode_Should_Be_Active_Internal(void) {
    if (opt_flow_mode_active) {
        // 已处于光流模式: 只有明显高于阈值后才切回视觉, 避免阈值附近抖动
        return imu_data.z < VISION_POSITION_MIN_HEIGHT_CM + VISION_POSITION_HYSTERESIS_CM;
    }
    // 未处于光流模式: 低于阈值立即切入, 与视觉失效高度保持一致
    return imu_data.z < VISION_POSITION_MIN_HEIGHT_CM;
}

/**
 * @brief 开始模式切换平滑: 记录当前目标倾角作为平滑起点
 */
static void Start_Mode_Switch_Smoothing(void) {
    mode_smooth_roll = flight_target.target_roll;
    mode_smooth_pitch = flight_target.target_pitch;
    mode_switch_start_ms = dataC.pit0_cnt;
    last_smooth_ms = 0;
    mode_switch_pending = 1;
}

/**
 * @brief 带模式切换平滑的目标姿态设置
 */
static void Set_Target_Attitude_Smoothed(float roll, float pitch, float yaw, float dt_sec) {
    (void)dt_sec; // 平滑步长按实际主循环/事件间隔计算, 不依赖调用方传入的标称 dt
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

/**
 * @brief 进入低高度光流速度环: 复位视觉/速度环状态并锁定当前航向
 */
void OpticalFlow_Mode_Enter(void) {
    if (opt_flow_mode_active) return;
    opt_flow_mode_active = 1;
    PID_Reset(&pid_opt_vel_x);
    PID_Reset(&pid_opt_vel_y);
    Image_Hover_Reset_For_OpticalFlow();
    last_opt_flow_ms = 0;
    flight_target.target_yaw = imu_data.yaw; // 进入低高度光流模式时锁定当前航向, 避免搜索残留
    Start_Mode_Switch_Smoothing();
}

/**
 * @brief 退出低高度光流速度环: 复位速度环积分
 */
void OpticalFlow_Mode_Exit(void) {
    if (!opt_flow_mode_active) return;
    opt_flow_mode_active = 0;
    PID_Reset(&pid_opt_vel_x);
    PID_Reset(&pid_opt_vel_y);
    last_opt_flow_ms = 0;
    flight_target.target_yaw = imu_data.yaw; // 退出光流模式时也锁定当前航向, 避免切换瞬间 yaw 跳变
    Start_Mode_Switch_Smoothing();
}

/**
 * @brief 供 image_ctrl 判断当前应使用哪个外环
 */
uint8_t OpticalFlow_Mode_Should_Be_Active(void) {
    return OpticalFlow_Mode_Should_Be_Active_Internal();
}

/**
 * @brief 供 image_ctrl 设置带模式切换平滑的目标姿态
 */
void OpticalFlow_Set_Target_Attitude_Smoothed(float roll, float pitch, float yaw, float dt_sec) {
    Set_Target_Attitude_Smoothed(roll, pitch, yaw, dt_sec);
}

/**
 * @brief 光流速度环: 机体系速度 -> 地面系速度 -> PID(目标0速) -> 机体系目标倾角
 * @note 低高度定点使用; 角度环/角速度环继续复用 fly_ctrl 现有链路
 */
static void Flight_OpticalFlow_Velocity_Control(float dt_sec, float *out_roll, float *out_pitch) {
#if defined(CY_CORE_CM7_0)
    float yaw_rad = VISION_EARTH_YAW_DEG(imu_data.yaw) * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    // 机体系速度 -> 地面系速度
    float earth_vel_x = upixels_data.filt_vel_x * cos_yaw - upixels_data.filt_vel_y * sin_yaw;
    float earth_vel_y = upixels_data.filt_vel_x * sin_yaw + upixels_data.filt_vel_y * cos_yaw;

    // 目标速度为 0, 误差 = 0 - 当前速度
    float target_earth_accel_x = PID_Calculate(&pid_opt_vel_x, -earth_vel_x, dt_sec);
    float target_earth_accel_y = PID_Calculate(&pid_opt_vel_y, -earth_vel_y, dt_sec);

    // 地面系目标 -> 机体系目标倾角 (与视觉位置环相同约定)
    float cur_yaw_rad = VISION_EARTH_YAW_DEG(imu_data.yaw) * 3.14159265f / 180.0f;
    float cur_cos_yaw = cosf(cur_yaw_rad);
    float cur_sin_yaw = sinf(cur_yaw_rad);
    float target_body_accel_x = target_earth_accel_x * cur_cos_yaw + target_earth_accel_y * cur_sin_yaw;
    float target_body_accel_y = -target_earth_accel_x * cur_sin_yaw + target_earth_accel_y * cur_cos_yaw;

    *out_pitch = -target_body_accel_x;
    *out_roll  = target_body_accel_y;
#else
    (void)dt_sec; (void)out_roll; (void)out_pitch;
#endif
}

/**
 * @brief 低高度光流速度环任务 (独立于视觉帧调度, 由主循环在光流新帧时驱动)
 * @param flow_frame_new 本次调用前是否有新的光流帧被消费; 无新帧时只做模式切换/复位, 不重复计算
 */
void Flight_OpticalFlow_Control_Task(uint8_t flow_frame_new) {
    // 带滞回的模式判断: 当前模式决定进入/退出阈值
    if (!OpticalFlow_Mode_Should_Be_Active_Internal()) {
        OpticalFlow_Mode_Exit();
        return;
    }

    OpticalFlow_Mode_Enter();

    // 光流数据未就绪或超时: 复位速度环并回平, 避免沿用陈旧速度/目标
    if (!flow_frame_new) {
        if (last_opt_flow_ms == 0 || (dataC.pit0_cnt - last_opt_flow_ms) > OPT_FLOW_TIMEOUT_MS) {
            PID_Reset(&pid_opt_vel_x);
            PID_Reset(&pid_opt_vel_y);
            Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, 0.01f);
        }
        return;
    }

#if defined(CY_CORE_CM7_0)
    // 光流帧无效或积分时间为 0 时视为数据未就绪, 回平等待有效帧
    if (upixels_data.valid == 0 || upixels_data.integration_timespan == 0) {
        PID_Reset(&pid_opt_vel_x);
        PID_Reset(&pid_opt_vel_y);
        Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, 0.01f);
        return;
    }
#endif

    uint32_t now_ms = dataC.pit0_cnt;
    uint32_t dt_ms = (last_opt_flow_ms == 0) ? 10U : (now_ms - last_opt_flow_ms);
    last_opt_flow_ms = now_ms;
    float dt_sec = dt_ms * 0.001f;
    if (dt_sec < 0.001f) dt_sec = 0.001f;
    if (dt_sec > 0.05f) dt_sec = 0.05f;

    float target_roll_val = 0.0f;
    float target_pitch_val = 0.0f;
    Flight_OpticalFlow_Velocity_Control(dt_sec, &target_roll_val, &target_pitch_val);
    Set_Target_Attitude_Smoothed(target_roll_val, target_pitch_val, flight_target.target_yaw, dt_sec);
}