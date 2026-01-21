#include "imu.h"
#include "zf_common_headfile.h"

// ================= 全局变量定义 =================
IMU_Data_t imu_data = {0}; 

// 内部算法变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差

// 陀螺仪校准相关
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;

static uint16_t calib_cnt = 0;
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// 融合参数 (建议调试时可微调)
#define K_POS  0.4f   // 位置修正系数 (高度权重)
#define K_VEL  0.8f   // 速度修正系数 (速度收敛快慢)

// ================= 内部辅助函数 =================
static float invSqrt(float x) {
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

void tof_init(void){
    while(1)
    {
        if(dl1b_init())
            printf("tof_init_error");
        else
            {
                printf("tof_init_done");
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

    // 2. 角度转弧度
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // ==============================================================================
    // 【核心逻辑：连续误差补偿】 (Adaptive Gain)
    // 我们无法算出干扰向量的方向，但能算出干扰的"烈度"。
    // 利用这个烈度，动态调整修正力度。
    // ==============================================================================
    float acc_weight = 1.0f;
    float error_magnitude = fabsf(acc_norm - GRAVITY_MSS); // 计算与重力(9.8)的偏差绝对值

    // 补偿曲线设计：
    // 偏差 < 0.5 (约0.05g): 认为是噪声，权重 1.0 (完全信任)
    // 偏差 > 2.5 (约0.25g): 认为是显著运动干扰，权重 0.0 (完全屏蔽)
    // 中间区域 : 使用线性插值进行平滑补偿，绝非简单的死区跳变
    if (error_magnitude > 0.8f) {
        acc_weight = 0.0f; 
    } else if (error_magnitude > 0.4f) {
        // 线性衰减公式：随着误差变大，权重线性下降
        acc_weight = 1.0f - (error_magnitude - 0.4f) / (0.8f - 0.4f);
    }
    // 结果：acc_weight 是一个 0.0 ~ 1.0 之间的连续系数

    // 3. 加速度归一化 (Mahony 必须步骤)
    if (acc_norm < 0.1f) return; 
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

    // 3. 去除重力 
    w_az = w_az - GRAVITY_MSS;

    // 4. 滤波与死区 (Z轴死区稍大，防止静态积分漂移)
    if(fabsf(w_ax) < 0.01f) w_ax = 0; 
    if(fabsf(w_ay) < 0.01f) w_ay = 0;
    if(fabsf(w_az) < 0.1f) w_az = 0;
    
    // 更新到结构体 (仅用于观察方向，不用于位置控制)
    imu_data.world_ax = w_ax;
    imu_data.world_ay = w_ay;
    imu_data.world_az = w_az;

    // ================== Z轴二阶观测器融合 (核心修改) ==================
    float acc_up_cms2 = w_az * 100.0f; // m/s^2 -> cm/s^2
    
    // 1. 惯性导航预测 (先只靠加速度计推算)
    // 速度 += 加速度 * dt
    imu_data.vz += acc_up_cms2 * DT;
    // 位置 += 速度 * dt
    imu_data.z += imu_data.vz * DT + 0.5f * acc_up_cms2 * DT * DT;
    
    // 2. ToF 观测修正
    if (dl1b_finsh_flag == 1) {
        dl1b_finsh_flag = 0;
        
        uint16_t tof_z_mm = dl1b_distance_mm;
        // 物理限幅
        if (tof_z_mm > 1500) tof_z_mm = 1500;

        // 有效范围判断
        if (tof_z_mm > 10) {
            
            // 倾角补偿 (将斜边距离换算为垂直高度)
            float rad_roll = imu_data.roll * (PI / 180.0f);
            float rad_pitch = imu_data.pitch * (PI / 180.0f);
            float kc = fabsf(cosf(rad_roll) * cosf(rad_pitch));
            
            float tof_height_cm = (tof_z_mm / 10.0f) * kc;

            // --- 核心算法：二阶互补/观测器 ---
            // 计算 "测量值" 与 "估计值" 的偏差
            float z_error = tof_height_cm - imu_data.z;

            // 修正位置 (Proportional term)
            imu_data.z += z_error * K_POS;

            // 修正速度 (Integral term / Velocity correction)
            // 逻辑：如果位置一直偏低，说明速度估算偏小，需要补偿速度
            imu_data.vz += z_error * K_VEL;
        }
    } else {
        // [新增] 阻尼逻辑：如果 ToF 丢失，让垂直速度缓慢归零，防止漂飞
        imu_data.vz *= 0.999f; 
    }

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
    dl1b_get_distance();
    
    float raw_gx = imu660ra_gyro_transition(imu660ra_gyro_x);
    float raw_gy = imu660ra_gyro_transition(imu660ra_gyro_y);
    float raw_gz = imu660ra_gyro_transition(imu660ra_gyro_z);

    float raw_ax = imu660ra_acc_transition(imu660ra_acc_x) * GRAVITY_MSS;
    float raw_ay = imu660ra_acc_transition(imu660ra_acc_y) * GRAVITY_MSS;
    float raw_az = imu660ra_acc_transition(imu660ra_acc_z) * GRAVITY_MSS;

    // ================= 校准逻辑 (包含加速度计) =================
    if (imu_data.is_calibrated == 0) {
        calib_cnt++;
        
        // 累加陀螺仪
        offset_gx += raw_gx;
        offset_gy += raw_gy;
        offset_gz += raw_gz;
        
        if (calib_cnt >= 2500) {
            // 计算平均值
            offset_gx /= 2500.0f;
            offset_gy /= 2500.0f;
            offset_gz /= 2500.0f;

            imu_data.is_calibrated = 1;
            imu_data.z = 0.0f;
            imu_data.vz = 0.0f; // 校准完成，速度清零
        }
        return; 
    }

    // ================= 1. 去除零偏 =================
    raw_gx -= offset_gx;
    raw_gy -= offset_gy;
    raw_gz -= offset_gz;
    // 注意：垂直轴(raw_ax)不要减，它的基准(gravity_ref)在 Navigation_Update 里用

    // ================= 2. 轴向映射 (这里补全了缺失的代码) =================
    float map_ax = raw_az;  // 机头
    float map_ay = -raw_ay;  // 机身右侧
    float map_az = -raw_ax;  // 垂直方向

    //  之前漏掉了下面这三行陀螺仪映射定义
    float map_gx = raw_gz;  // Roll (横滚)
    float map_gy = -raw_gy;  // Pitch (俯仰)
    float map_gz = raw_gx;  // Yaw (航向)

    // 死区处理 (仅针对陀螺仪，防止 Yaw 漂移)
    //if (fabsf(map_gx) < 0.1f) map_gx = 0; 
    //if (fabsf(map_gy) < 0.1f) map_gy = 0;
    if (fabsf(map_gz) < VALID_G_MIN) map_gz = 0;

    // ================= 3. 滤波与解算 =================
    imu_data.groll = -Kalman_Update(&K_groll, -map_gx);
    imu_data.gpitch = Kalman_Update(&K_gpitch, map_gy);
    imu_data.gyaw = Kalman_Update(&K_gyaw, -map_gz);
    
    Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    
    // 欧拉角转换
    imu_data.roll = -atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1) imu_data.pitch = copysignf(90.0f, sinp);
    else imu_data.pitch =  - asinf(sinp) * 180.0f / PI;
    
    imu_data.yaw = - atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    Navigation_Update(map_ax, map_ay, map_az);
}