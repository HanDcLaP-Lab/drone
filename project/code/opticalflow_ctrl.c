#include "opticalflow_ctrl.h"

// =================== 光流速度环内部状态 ===================
static uint8_t  opt_flow_mode_active = 0;   // 低高度光流速度环激活标志
static uint32_t last_opt_flow_ms = 0;      // 光流速度环上次计算时刻 (ms)

/**
 * @brief 进入低高度光流速度环: 复位速度环状态并锁定当前航向，启动姿态平滑
 */
void OpticalFlow_Mode_Enter(void) {
    if (opt_flow_mode_active) return;
    opt_flow_mode_active = 1;
    PID_Reset(&pid_opt_vel_x);
    PID_Reset(&pid_opt_vel_y);
    last_opt_flow_ms = 0;
    flight_target.target_yaw = imu_data.yaw; // 进入低高度光流模式时锁定当前航向
    Flight_Attitude_Smoother_Reset();
}

/**
 * @brief 退出低高度光流速度环: 复位速度环积分，启动姿态平滑
 */
void OpticalFlow_Mode_Exit(void) {
    if (!opt_flow_mode_active) return;
    opt_flow_mode_active = 0;
    PID_Reset(&pid_opt_vel_x);
    PID_Reset(&pid_opt_vel_y);
    last_opt_flow_ms = 0;
    flight_target.target_yaw = imu_data.yaw; // 退出光流模式时锁定当前航向, 避免切换瞬间 yaw 跳变
    Flight_Attitude_Smoother_Reset();
}

/**
 * @brief 查询光流模式是否激活
 */
uint8_t OpticalFlow_Mode_Is_Active(void) {
    return opt_flow_mode_active;
}

/**
 * @brief 光流速度环: 机体系速度 -> 地面系速度 -> PID(目标0速) -> 机体系目标倾角
 * @note 低高度定点使用; 角度环/角速度环继续复用 fly_ctrl 现有链路
 */
static void Flight_OpticalFlow_Velocity_Control(float dt_sec, float *out_roll, float *out_pitch) {
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
}

/**
 * @brief 低高度光流速度环任务 (由主循环在处于光流模式时驱动)
 * @param flow_frame_new 本次调用前是否有新的光流帧被消费; 无新帧时只做超时判定, 不重复计算
 */
void Flight_OpticalFlow_Control_Task(uint8_t flow_frame_new) {
    // 光流数据未就绪或超时: 复位速度环并回平, 避免沿用陈旧速度/目标
    if (!flow_frame_new) {
        if (last_opt_flow_ms == 0 || (dataC.pit0_cnt - last_opt_flow_ms) > OPT_FLOW_TIMEOUT_MS) {
            PID_Reset(&pid_opt_vel_x);
            PID_Reset(&pid_opt_vel_y);
            Flight_Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, 0.01f);
        }
        return;
    }

    // 光流帧无效或积分时间为 0 时视为数据未就绪, 回平等待有效帧
    if (upixels_data.valid == 0 || upixels_data.integration_timespan == 0) {
        PID_Reset(&pid_opt_vel_x);
        PID_Reset(&pid_opt_vel_y);
        Flight_Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, 0.01f);
        return;
    }

    uint32_t now_ms = dataC.pit0_cnt;
    uint32_t dt_ms = (last_opt_flow_ms == 0) ? 10U : (now_ms - last_opt_flow_ms);
    last_opt_flow_ms = now_ms;
    float dt_sec = dt_ms * 0.001f;
    if (dt_sec < 0.001f) dt_sec = 0.001f;
    if (dt_sec > 0.05f) dt_sec = 0.05f;

    float target_roll_val = 0.0f;
    float target_pitch_val = 0.0f;
    Flight_OpticalFlow_Velocity_Control(dt_sec, &target_roll_val, &target_pitch_val);
    Flight_Set_Target_Attitude_Smoothed(target_roll_val, target_pitch_val, flight_target.target_yaw, dt_sec);
}