#include "imu.h"
#include "zf_common_headfile.h"

// ================= 全局变量定义 =================
IMU_Data_t imu_data = {0}; 

// 内部算法变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差

// 陀螺仪校准相关
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
// [新增] 加速度计校准相关 (Sensor坐标系)
static float offset_raw_ay = 0; // Sensor Y轴零偏 (对应机身侧向)
static float offset_raw_az = 0; // Sensor Z轴零偏 (对应机身前后)
static float gravity_ref = 9.8f; // 真实的重力基准 (来自 Sensor X轴)

static uint16_t calib_cnt = 0;

// 融合参数 (建议调试时可微调)
#define K_POS  0.3f   // 位置修正系数 (高度权重)
#define K_VEL  0.7f   // 速度修正系数 (速度收敛快慢)

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

    // 1. 计算加速度模长
    float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);

    // 2. 角度转弧度
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // 3. 加速度归一化
    if (accel_magnitude < 0.1f) return; 
    float inv_norm = 1.0f / accel_magnitude;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    // 4. 估计重力方向
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 误差计算 (叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 6. 积分误差
    exInt += ex * KI * DT;
    eyInt += ey * KI * DT;
    ezInt += ez * KI * DT;

    // 7. 修正角速度
    gx += KP * ex + exInt;
    gy += KP * ey + eyInt;
    gz += KP * ez + ezInt;

    // 8. 四元数更新
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

    // 3. 去除重力 (使用校准得到的真实基准值，而非宏定义)
    w_az = w_az - gravity_ref;

    // 4. 滤波与死区 (Z轴死区稍大，防止静态积分漂移)
    if(fabsf(w_ax) < 0.1f) w_ax = 0; 
    if(fabsf(w_ay) < 0.1f) w_ay = 0;
    if(fabsf(w_az) < 0.25f) w_az = 0;
    
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
// [code/imu.c]

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
        
        // 1. 累加陀螺仪
        offset_gx += raw_gx;
        offset_gy += raw_gy;
        offset_gz += raw_gz;
        
        // 2. 累加加速度计 (假设静止平放)
        offset_raw_ay += raw_ay; // 应该为0
        offset_raw_az += raw_az; // 应该为0
        
        // 垂直轴不应为0，应为当地重力，我们累加它的绝对值或相反数
        // 因为 map_az = -raw_ax，且 map_az 向上为正(9.8)，所以 raw_ax 应该约为 -9.8
        // 我们记录这个"1G"的模长
        gravity_ref += -raw_ax; 
        
        if (calib_cnt >= 2500) {
            // 计算平均值
            offset_gx /= 2500.0f;
            offset_gy /= 2500.0f;
            offset_gz /= 2500.0f;
            
            offset_raw_ay /= 2500.0f;
            offset_raw_az /= 2500.0f;
            gravity_ref   /= 2500.0f; // 得到实测的重力值

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

    // 加速度计去水平零偏
    raw_ay -= offset_raw_ay;
    raw_az -= offset_raw_az;
    // 注意：垂直轴(raw_ax)不要减，它的基准(gravity_ref)在 Navigation_Update 里用

    // ================= 2. 轴向映射 (这里补全了缺失的代码) =================
    float map_ax = -raw_az;  // 机头
    float map_ay = -raw_ay;  // 机身右侧
    float map_az = -raw_ax;  // 垂直方向

    //  之前漏掉了下面这三行陀螺仪映射定义
    float map_gx = -raw_gz;  // Roll (横滚)
    float map_gy = -raw_gy;  // Pitch (俯仰)
    float map_gz = -raw_gx;  // Yaw (航向)

    // 死区处理 (仅针对陀螺仪，防止 Yaw 漂移)
    if (fabsf(map_gx) < 0.1f) map_gx = 0; 
    if (fabsf(map_gy) < 0.1f) map_gy = 0;
    if (map_gz > -0.1f && map_gz < 0.1f) map_gz = 0.0f; // Yaw 轴强力死区

    // ================= 3. 滤波与解算 =================
    imu_data.groll = -Kalman_Update(&K_groll, map_gx);
    imu_data.gpitch = Kalman_Update(&K_gpitch, map_gy);
    imu_data.gyaw = Kalman_Update(&K_gyaw, map_gz);
    
    Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    
    // 欧拉角转换
    imu_data.roll = -atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f + 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1) imu_data.pitch = copysignf(90.0f, sinp);
    else imu_data.pitch = asinf(sinp) * 180.0f / PI;
    
    imu_data.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    Navigation_Update(map_ax, map_ay, map_az);
}