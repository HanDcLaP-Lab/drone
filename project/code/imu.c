#include "imu.h"
#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

// ******************************************************************************
// 传感器融合架构 (Mahony姿态 + 世界加速度观测)
//
//   IMU_Update_Loop() (1.25ms ISR调用)
//        │
//   ┌────┴─────────────────────────────────────────────┐
//   │ 1. 读取原始传感器 (IMU660RA)                          │
//   │ 2. tof_update()  → 获取高度/速度 + 运行高度PID        │
//   │    (tof.c 内部处理传感器差异, 仅当数据就绪时计算)       │
//   │ 3. 电机振动陷波滤波 (自适应跟踪平均转速基频)        │
//   │ 4. 卡尔曼滤波 (6轴加速度+陀螺仪)                   │
//   │ 5. 约2.5s采样校准 → 初始姿态四元数 + 陀螺零偏       │
//   │ 6. Mahony_Update()   姿态融合 (自适应加速度权重)    │
//   │    ├─ 连续误差补偿: 运动剧烈→降权, 静止→全信        │
//   │    ├─ 积分修正: 仅静止时累积 (防止Yaw漂移)          │
//   │    └─ Z轴仅靠陀螺仪积分 (不接受加速度计修正)         │
//   │ 7. 四元数→欧拉角 (roll/pitch/yaw)                  │
//   │ 8. Yaw增量累加 (支持连续旋转, 不受±180°跳变影响)     │
//   │ 9. Navigation_Update() → 仅计算世界加速度 (观测用)   │
//   └──────────────────────────────────────────────────┘
//
// 输出: imu_data (roll, pitch, yaw, groll, gpitch, gyaw, z, vz, world_a*)
// 注意: z/vz 由 tof_update() 写入, Navigation_Update 不再参与 Z 轴融合
// ******************************************************************************

// ================= 全局变量定义 =================
IMU_Data_t imu_data = {0}; 
volatile uint16_t imu_gyro_new_sample_count = 0;
volatile uint16_t imu_acc_new_sample_count = 0;

#define IMU_CALIB_VALID_SAMPLES 1000u   // 有效静止样本目标数 (~1.25s @ 800Hz)
#define IMU_CALIB_GYRO_TOLERANCE 5.0f   // 静止角速度上限 (°/s)
#define IMU_CALIB_MAX_ATTEMPTS 4000u    // 超时保底 (5s @ 800Hz)

// 内部算法变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
// 陀螺仪校准相关
static double offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float  offset_ax = 0.0f, offset_ay = 0.0f, offset_az = 0.0f; // 加速度计三轴零偏 (m/s^2)
static double sum_gx = 0, sum_gy = 0, sum_gz = 0;
static double sum_ax = 0, sum_ay = 0, sum_az = 0;
static uint16_t calib_cnt = 0;
static uint16_t calib_valid_cnt = 0; // 通过静止检测的有效样本数

typedef struct {
    float ax, ay, az;
    float acc_norm;
    float acc_weight;
} IMU_Motion_Debug_t;

static volatile IMU_Motion_Debug_t imu_motion_debug;

#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// ================= 陷波滤波器 (电机振动抑制, IMU 特化) =================
#if NOTCH_ENABLE

// 谐波倍率: 基频/二倍频/六倍频(叶片通过频率)
static const float notch_harmonic_mult[NOTCH_HARMONIC_COUNT] = {1.0f, 2.0f, 6.0f};

#define NOTCH_SLICE_COUNT (NOTCH_MOTOR_COUNT * NOTCH_HARMONIC_COUNT) // 4×3=12

// IMU 专用陷波配置 (通用 biquad 实现见 filters.c)
const NotchConfig_t notch_cfg = {
    .fs       = NOTCH_FS,
    .q        = NOTCH_Q,
    .min_freq = NOTCH_MIN_FREQ,
    .max_freq = NOTCH_MAX_FREQ,
};

// 每通道级联 NOTCH_SLICE_COUNT 个 biquad, 按 [电机0·1×, 电机0·2×, 电机0·6×, 电机1·1×, ...] 排列
static NotchFilter_t notch_gx[NOTCH_SLICE_COUNT];
static NotchFilter_t notch_gy[NOTCH_SLICE_COUNT];
static NotchFilter_t notch_gz[NOTCH_SLICE_COUNT];
static NotchFilter_t notch_ax[NOTCH_SLICE_COUNT];
static NotchFilter_t notch_ay[NOTCH_SLICE_COUNT];
static NotchFilter_t notch_az[NOTCH_SLICE_COUNT];

static uint16_t notch_timeout_cnt;   // 距上次转速更新的飞控周期数
static uint8_t  notch_active;        // 0=旁通(超时/未收到数据), 1=工作中
uint8_t         notch_active_count;  // 当前生效的陷波切片数 0~12 (调试接口)

// ---- 批量操作 ----
static void Notch_InitAll(void) {
    for (int i = 0; i < NOTCH_SLICE_COUNT; i++) {
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GX) Notch_Init(&notch_gx[i]);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GY) Notch_Init(&notch_gy[i]);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GZ) Notch_Init(&notch_gz[i]);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AX) Notch_Init(&notch_ax[i]);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AY) Notch_Init(&notch_ay[i]);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AZ) Notch_Init(&notch_az[i]);
    }
    notch_timeout_cnt = NOTCH_TIMEOUT_TICKS + 1;
    notch_active = 0;
}

// UART 回调触发: 每个电机独立计算 1×/2×/6× 频率, 各谐波独立判定边界
static void Notch_UpdateAllFreqs(void) {
    uint8_t active_cnt = 0;
    for (int m = 0; m < NOTCH_MOTOR_COUNT; m++) {
        float rpm = (float)motor_value.receive_speed_data[m];
        if (rpm < 0) rpm = 0;
        float base_freq = rpm * 0.016666667f;  // RPM / 60

        for (int h = 0; h < NOTCH_HARMONIC_COUNT; h++) {
            int idx = m * NOTCH_HARMONIC_COUNT + h;
            float freq = base_freq * notch_harmonic_mult[h];

            if (freq >= NOTCH_MIN_FREQ && freq <= NOTCH_MAX_FREQ) active_cnt++;

            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GX) Notch_SetFreq(&notch_gx[idx], freq, &notch_cfg);
            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GY) Notch_SetFreq(&notch_gy[idx], freq, &notch_cfg);
            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GZ) Notch_SetFreq(&notch_gz[idx], freq, &notch_cfg);
            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AX) Notch_SetFreq(&notch_ax[idx], freq, &notch_cfg);
            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AY) Notch_SetFreq(&notch_ay[idx], freq, &notch_cfg);
            if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AZ) Notch_SetFreq(&notch_az[idx], freq, &notch_cfg);
        }
    }
    notch_active_count = active_cnt;
}

// 每 1.25ms 调用: 检查转速数据有效期, 更新激活状态
static void Notch_CheckTimeout(void) {
    if (motor_value.speed_data_updated) {
        motor_value.speed_data_updated = 0;
        notch_timeout_cnt = 0;

        if (!notch_active) {
            Notch_InitAll();                  // 旁通→激活: 清零延迟线防阶跃
            notch_active = 1;
        }
        Notch_UpdateAllFreqs();
    } else if (notch_active) {
        notch_timeout_cnt++;
        if (notch_timeout_cnt > NOTCH_TIMEOUT_TICKS) {
            notch_active = 0;
            notch_active_count = 0;
        }
    }
}

// 对单个通道级联全部 NOTCH_SLICE_COUNT 个 biquad
static float Notch_ApplyChannel(NotchFilter_t *nf_array, float value) {
    for (int i = 0; i < NOTCH_SLICE_COUNT; i++) {
        value = Notch_Update(&nf_array[i], value);
    }
    return value;
}
#endif

// ================= 内部辅助函数 =================
static float invSqrt(float x) {
    if (x < 1e-10f) return 1.0f;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void imu_init(void){
    while(1)
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");   
        }
        else
        {
           break;
        } 
        system_delay_ms(1000);                                       
    }
}

static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 计算加速度模长 (用于计算偏差)
    // 假设传入的 ax,ay,az 单位是 m/s^2 (根据您代码中的 GRAVITY_MSS 宏)
    // 如果传入的是归一化值(1.0g)，请将 9.8f 改为 1.0f
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    imu_motion_debug.acc_norm = acc_norm;

    // 2. 角度转弧度
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // 先按模长偏差降低加速度可信度，再叠加下方的方向创新门控。
    float acc_weight = 1.0f;
    float error_magnitude = fabsf(acc_norm - GRAVITY_MSS); // 计算与重力(9.8)的偏差绝对值

    // 补偿曲线设计：
    // 偏差 <= 0.4m/s^2: 权重 1.0；偏差 >= 0.8m/s^2: 权重 0.0
    // 中间区域 : 使用线性插值进行平滑补偿，绝非简单的死区跳变
    if (error_magnitude > 0.8f) {
        acc_weight = 0.0f; 
    } else if (error_magnitude > 0.4f) {
        // 线性衰减公式：随着误差变大，权重线性下降
        acc_weight = 1.0f - (error_magnitude - 0.4f) / (0.8f - 0.4f);
    }
    // 结果：acc_weight 是一个 0.0 ~ 1.0 之间的连续系数

    // 3. 加速度归一化 (Mahony 必须步骤)
    if (acc_norm < 0.1f || acc_norm != acc_norm) return;  // NaN也会通过<比较, 加isnan检查
    float inv_norm = 1.0f / acc_norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    // 4. 估计重力方向 (基于当前四元数推算)
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 误差计算 (叉积: 测量向量 x 估计向量)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 运动加速度的模长可能仍接近重力；方向创新用于阻止其污染 roll/pitch。
    float direction_error = sqrtf(ex * ex + ey * ey + ez * ez);
    float direction_weight = 1.0f;
    if (direction_error >= IMU_ACC_DIR_REJECT_ERROR) {
        direction_weight = 0.0f;
    } else if (direction_error > IMU_ACC_DIR_FULL_TRUST_ERROR) {
        direction_weight = (IMU_ACC_DIR_REJECT_ERROR - direction_error)
                         / (IMU_ACC_DIR_REJECT_ERROR - IMU_ACC_DIR_FULL_TRUST_ERROR);
    }
    if (direction_weight < acc_weight) acc_weight = direction_weight;
    imu_motion_debug.acc_weight = acc_weight;

    // 6. 积分误差 (Integral Feedback)
    // 【关键补偿】：当运动剧烈(权重低)时，必须停止积分！
    // 否则错误的加速度会被"记忆"到陀螺仪零偏中，导致停下来后Yaw还在飘
    if (acc_weight > 0.1f) { 
        exInt += ex * KI * DT * acc_weight;
        eyInt += ey * KI * DT * acc_weight;
        // ezInt += ez * KI * DT; // Z轴(Yaw)本身就不该有加速度积分修正
    }
    
    // 7. 修正角速度 (Proportional Feedback)
    // 将计算出的 acc_weight 乘入 KP
    // 这样做的物理含义是：当存在非重力加速度时，我们"相应地"降低对加速度计的信任
    // 这是一个动态调节过程，不是死区。
    gx += KP * acc_weight * ex + exInt;
    gy += KP * acc_weight * ey + eyInt;
    
    // Z轴处理：Yaw 轴绝对不能接受加速度计的直接修正 (会引入严重的离心力漂移)
    // 之前的 gz += 0 是完全正确的。Yaw 只能靠陀螺仪积分。
    gz += 0; 

    // 8. 四元数更新 (毕卡算法)
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * DT);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * DT);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * DT);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * DT);

    // 9. 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

static void Navigation_Update(float ax, float ay, float az) {
    // 1. 预计算四元数乘积
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    // 2. 将机体加速度旋转到世界坐标系
    float w_ax = (1 - 2*(q2q2 + q3q3))*ax + 2*(q1q2 - q0q3)*ay + 2*(q1q3 + q0q2)*az;
    float w_ay = 2*(q1q2 + q0q3)*ax + (1 - 2*(q1q1 + q3q3))*ay + 2*(q2q3 - q0q1)*az;
    float w_az = 2*(q1q3 - q0q2)*ax + 2*(q2q3 + q0q1)*ay + (1 - 2*(q1q1 + q2q2))*az;

    // 3. 去除重力 (加速度计零偏已在上游 IMU_Update_Loop 中扣除)
    w_az = w_az - GRAVITY_MSS;

    // 4. 滤波与死区 (Z轴死区稍大，防止静态积分漂移)
    if(fabsf(w_ax) < 0.01f) w_ax = 0; 
    if(fabsf(w_ay) < 0.01f) w_ay = 0;
    if(fabsf(w_az) < 0.12f) w_az = 0; // [修复] 0.05→0.12: 覆盖校准后的残余噪声 (~12mg)
    
    // 更新到结构体 (仅用于观察方向，不用于位置控制)
    imu_data.world_ax = w_ax;
    imu_data.world_ay = w_ay;
    imu_data.world_az = w_az;

    // Z 轴位置/速度由 tof_update() 独立处理 (TOF-only, 不依赖加速度估计)
    // Navigation_Update 仅负责世界加速度观测

    // ================== 水平通道清零 ==================
    // 强制清零，避免数据漂移干扰判断
    imu_data.vx = 0;
    imu_data.vy = 0;
    imu_data.x = 0;
    imu_data.y = 0;
}

// ================= 对外接口函数 =================

void IMU_Update_Loop(void) {
  
    imu660ra_get_acc();
    imu660ra_get_gyro();
    // tof_update() 已移至 ISR 层: VL53L8CX→gpio_2_exti, DL1B→pit0_ch0

    // [新增] 统计原始寄存器值变化次数, 用于估算 IMU 实际数据更新率
    static int16 last_raw_acc_x = 0, last_raw_acc_y = 0, last_raw_acc_z = 0;
    static int16 last_raw_gyro_x = 0, last_raw_gyro_y = 0, last_raw_gyro_z = 0;
    static uint8_t imu_sample_counter_inited = 0;
    if (!imu_sample_counter_inited) {
        last_raw_acc_x = imu660ra_acc_x;
        last_raw_acc_y = imu660ra_acc_y;
        last_raw_acc_z = imu660ra_acc_z;
        last_raw_gyro_x = imu660ra_gyro_x;
        last_raw_gyro_y = imu660ra_gyro_y;
        last_raw_gyro_z = imu660ra_gyro_z;
        imu_sample_counter_inited = 1;
    } else {
        if (imu660ra_acc_x != last_raw_acc_x || imu660ra_acc_y != last_raw_acc_y || imu660ra_acc_z != last_raw_acc_z) {
            imu_acc_new_sample_count++;
            last_raw_acc_x = imu660ra_acc_x;
            last_raw_acc_y = imu660ra_acc_y;
            last_raw_acc_z = imu660ra_acc_z;
        }
        if (imu660ra_gyro_x != last_raw_gyro_x || imu660ra_gyro_y != last_raw_gyro_y || imu660ra_gyro_z != last_raw_gyro_z) {
            imu_gyro_new_sample_count++;
            last_raw_gyro_x = imu660ra_gyro_x;
            last_raw_gyro_y = imu660ra_gyro_y;
            last_raw_gyro_z = imu660ra_gyro_z;
        }
    }

    float raw_gx = imu660ra_gyro_transition(imu660ra_gyro_x);
    float raw_gy = imu660ra_gyro_transition(imu660ra_gyro_y);
    float raw_gz = imu660ra_gyro_transition(imu660ra_gyro_z);

    float raw_ax = imu660ra_acc_transition(imu660ra_acc_x) * GRAVITY_MSS;
    float raw_ay = imu660ra_acc_transition(imu660ra_acc_y) * GRAVITY_MSS;
    float raw_az = imu660ra_acc_transition(imu660ra_acc_z) * GRAVITY_MSS;

    // ================= 电机振动陷波滤波 =================
#if NOTCH_ENABLE
    Notch_CheckTimeout();
    if (notch_active) {
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GX) raw_gx = Notch_ApplyChannel(notch_gx, raw_gx);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GY) raw_gy = Notch_ApplyChannel(notch_gy, raw_gy);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_GZ) raw_gz = Notch_ApplyChannel(notch_gz, raw_gz);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AX) raw_ax = Notch_ApplyChannel(notch_ax, raw_ax);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AY) raw_ay = Notch_ApplyChannel(notch_ay, raw_ay);
        if (NOTCH_CHANNEL_MASK & NOTCH_CHANNEL_AZ) raw_az = Notch_ApplyChannel(notch_az, raw_az);
    }
#endif

    // ================= 加速度计卡尔曼滤波 =================
    raw_ax = Kalman_Update(&K_ax, raw_ax);
    raw_ay = Kalman_Update(&K_ay, raw_ay);
    raw_az = Kalman_Update(&K_az, raw_az);

    raw_gx = Kalman_Update(&K_groll, raw_gx);
    raw_gy = Kalman_Update(&K_gpitch, raw_gy);
    raw_gz = Kalman_Update(&K_gyaw, raw_gz);

    // ================= 校准逻辑 =================
    if (imu_data.is_calibrated == 0) {
        calib_cnt++;

        // 静止检测仅看角速度（加速度计可能有偏置，其模长本来就不等于重力，不可用于静止判断）
        float gyro_mag = sqrtf(raw_gx * raw_gx + raw_gy * raw_gy + raw_gz * raw_gz);
        if (gyro_mag < IMU_CALIB_GYRO_TOLERANCE) {
            sum_gx += raw_gx;
            sum_gy += raw_gy;
            sum_gz += raw_gz;
            sum_ax += raw_ax;
            sum_ay += raw_ay;
            sum_az += raw_az;
            calib_valid_cnt++;
        }

        uint8_t enough_valid = (calib_valid_cnt >= IMU_CALIB_VALID_SAMPLES);
        uint8_t timed_out    = (calib_cnt >= IMU_CALIB_MAX_ATTEMPTS && calib_valid_cnt >= 100);
        if (enough_valid || timed_out) {
            uint16_t n = calib_valid_cnt;

            // 陀螺零偏
            offset_gx = (float)(sum_gx / (double)n);
            offset_gy = (float)(sum_gy / (double)n);
            offset_gz = (float)(sum_gz / (double)n);

            // 加速度计均值 → 映射到机体
            float avg_ax = (float)(sum_ax / (double)n);
            float avg_ay = (float)(sum_ay / (double)n);
            float avg_az = (float)(sum_az / (double)n);
            float rax = IMU_MAP_AX(avg_ax, avg_ay, avg_az);
            float ray = IMU_MAP_AY(avg_ax, avg_ay, avg_az);
            float raz = IMU_MAP_AZ(avg_ax, avg_ay, avg_az);

            // 统一矢量分解：方向取加速度计矢量归一化 → 大小取重力标量 → 差值即三轴偏置
            //   静止时: avg = true_gravity + bias
            //   方向正确（偏置不显著改变矢量方向），模长已知 = GRAVITY_MSS
            float ra_mag  = sqrtf(rax * rax + ray * ray + raz * raz);
            float ra_inv  = (ra_mag > 0.01f) ? (1.0f / ra_mag) : 1.0f;
            float grav_x  = rax * ra_inv * GRAVITY_MSS; // 真重力在机体三轴的分量
            float grav_y  = ray * ra_inv * GRAVITY_MSS;
            float grav_z  = raz * ra_inv * GRAVITY_MSS;
            offset_ax     = rax - grav_x;
            offset_ay     = ray - grav_y;
            offset_az     = raz - grav_z;

            // 初始姿态（从重力方向分量反算 roll/pitch，yaw 恒为 0）
            float init_roll  = atan2f(grav_y, grav_z);
            float init_pitch = atan2f(-grav_x, sqrtf(grav_y * grav_y + grav_z * grav_z));

            float c1 = cosf(0.0f);          float s1 = sinf(0.0f);
            float c2 = cosf(init_pitch / 2); float s2 = sinf(init_pitch / 2);
            float c3 = cosf(init_roll / 2);  float s3 = sinf(init_roll / 2);

            q0 = c1*c2*c3 + s1*s2*s3;
            q1 = c1*c2*s3 - s1*s2*c3;
            q2 = c1*s2*c3 + s1*c2*s3;
            q3 = s1*c2*c3 - c1*s2*s3;

            float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;

            imu_data.is_calibrated = 1;
            imu_data.z = 0.0f;
            imu_data.vz = 0.0f;
            imu_data.yaw = 0.0f;
        }
        return;
    }

    // ================= 1. 去除陀螺零偏 =================
    raw_gx -= offset_gx;
    raw_gy -= offset_gy;
    raw_gz -= offset_gz;

    // ================= 2. 轴向映射 =================
    float map_ax = IMU_MAP_AX(raw_ax, raw_ay, raw_az);
    float map_ay = IMU_MAP_AY(raw_ax, raw_ay, raw_az);
    float map_az = IMU_MAP_AZ(raw_ax, raw_ay, raw_az);

    // 去除加速度计三轴零偏（校准阶段测得，统一在此处扣除）
    map_ax -= offset_ax;
    map_ay -= offset_ay;
    map_az -= offset_az;

    imu_motion_debug.ax = map_ax;
    imu_motion_debug.ay = map_ay;
    imu_motion_debug.az = map_az;

    float map_gx = IMU_MAP_GX(raw_gx, raw_gy, raw_gz);
    float map_gy = IMU_MAP_GY(raw_gx, raw_gy, raw_gz);
    float map_gz = IMU_MAP_GZ(raw_gx, raw_gy, raw_gz);

    // 死区处理 (仅针对陀螺仪，防止 Yaw 漂移)
    //if (fabsf(map_gx) < 0.1f) map_gx = 0; 
    //if (fabsf(map_gy) < 0.1f) map_gy = 0;
    if (fabsf(map_gz) < VALID_G_MIN) map_gz = 0;

    imu_data.groll  = map_gx;
    imu_data.gpitch = map_gy;
    imu_data.gyaw   = map_gz;

    Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    
    // 欧拉角转换
    // 修正为标准欧拉角公式 (NED坐标系)
    imu_data.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1) imu_data.pitch = copysignf(90.0f, sinp);
    else imu_data.pitch = asinf(sinp) * 180.0f / PI;

    // ================= 4. 应用安装误差补偿 =================
    imu_data.roll  -= IMU_MOUNT_ADJUST_ROLL;
    imu_data.pitch -= IMU_MOUNT_ADJUST_PITCH;
    
    // Yaw 不再从四元数欧拉角差分得到，避免 roll/pitch 加速度修正耦合进 yaw。
    // 这里使用机体系角速度换算出的欧拉 yaw rate 做连续积分。
    float yaw_roll_rad = imu_data.roll * (PI / 180.0f);
    float yaw_pitch_rad = imu_data.pitch * (PI / 180.0f);
    float cos_pitch = cosf(yaw_pitch_rad);
    if (fabsf(cos_pitch) < 0.1f) {
        cos_pitch = (cos_pitch >= 0.0f) ? 0.1f : -0.1f;
    }
    float yaw_rate = (map_gy * sinf(yaw_roll_rad) + map_gz * cosf(yaw_roll_rad)) / cos_pitch;
    imu_data.yaw += yaw_rate * DT;

    Navigation_Update(map_ax, map_ay, map_az);
}

void IMU_Check_Data_Print(void) {
#if IMU_MOTION_DEBUG_ENABLE
    static uint32_t last_print_ms = 0;
    uint32_t now_ms = dataC.pit0_cnt;

    if (!imu_data.is_calibrated ||
        (uint32_t)(now_ms - last_print_ms) < IMU_MOTION_DEBUG_PERIOD_MS) return;
    last_print_ms = now_ms;

    float ax = imu_motion_debug.ax;
    float ay = imu_motion_debug.ay;
    float az = imu_motion_debug.az;
    float acc_roll = atan2f(ay, az) * 180.0f / PI;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           (unsigned long)now_ms,
           imu_data.roll, imu_data.pitch,
           imu_data.groll, imu_data.gpitch,
           ax, ay, az,
           imu_motion_debug.acc_norm, imu_motion_debug.acc_weight,
           acc_roll, acc_pitch);
#endif
}
