#include "image_process.h"
#include "image.h"
#include <math.h>

// ******************************************************************************
// 地面投影流水线 (像素坐标 → 地面物理坐标)
//
//   calculate_ground_positions(height, pitch, roll, yaw)
//        │
//   ┌────┴──────────────────────────────────────────────┐
//   │ 1. pixelTo3DRay(u,v)    像素→3D射线 (相机畸变校正) │
//   │ 2. cameraToBody()        安装偏转(CAM_TOP_YAW_DEG) │
//   │                           + 姿态旋转 (pitch/roll补偿)│
//   │ 3. projectToGround()     相似三角形投影到水平地面   │
//   │ 4. 上电航向固定系旋转 + 卡尔曼滤波 + 转回机体系       │
//   └──────────────────────────────────────────────────┘
//
// 输出: pos.raw_car/raw_target[] (原始位置), pos.k_car/k_target (主目标Kalman滤波后)
// 单位: cm (取决于传入的 height 参数单位)
// 关键: 在无人机上电航向固定系下做卡尔曼滤波，避免机体旋转引起的滞后
// ******************************************************************************

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 定义 3D 空间向量
typedef struct { double x, y, z; } Vector3D;

// 全局变量定义
GroundPos pos = {0};
static uint8_t car_position_filter_initialized = 0;
static uint8_t target_position_filter_initialized = 0;

void ground_position_history_reset(void) {
    car_position_filter_initialized = 0;
    target_position_filter_initialized = 0;
    memset(&pos, 0, sizeof(pos));
    dataC.car_target_dist = 0.0f;
}

// ==========================================
// 2. 核心算法：像素坐标 -> 3D 空间射线 (指向地面)
// ==========================================
// 输出坐标系定义 (Camera Frame):
// X: 相机右侧 (Right)
// Y: 相机前方 (Forward, 图像上方)
// Z: 相机上方 (Up, 实际指向相机内部，地面点此处为负值)
static Vector3D pixelTo3DRay(double u, double v) {
    Vector3D ray;
    
    double u_prime = u - CAM_CX;
    double v_prime = v - CAM_CY;

    // 转换为相机物理坐标 (X向右, Y向前)
    double x = INV_S11 * u_prime + INV_S12 * v_prime;
    double y = -(INV_S21 * u_prime + INV_S22 * v_prime); // 取负，使得 Y 正向为前方

    double rho = sqrt(x * x + y * y);
    double rho2 = rho * rho;
    double z_poly = CAM_A0 + CAM_A2 * rho2 + CAM_A3 * rho2*rho + CAM_A4 * rho2*rho2;

    // 构建射线：X为右，Y为前。由于 CAM_A0 是正的，z_poly 指向相机内部(上)。
    // 物理世界的光线从地面射向相机，所以我们要找的“指向地面的射线”是向下(Z为负)
    ray.x = x;
    ray.y = y;
    ray.z = -z_poly; 
    
    return ray;
}

// ==========================================
// 3. 无人机姿态旋转 (Pitch, Roll)
// ==========================================
// 将相机坐标系下的射线旋转回水平坐标系 (Body/World Frame)
// 输入 cam: x=Right, y=Forward, z=Up(Negative)
// 输出 body: x=World_Right, y=World_Forward, z=World_Up(Negative)
static Vector3D cameraToBody(const Vector3D *cam, double sinp, double cosp, double sinr, double cosr) {
    Vector3D body;

    // 相机安装偏转: 图像上方相对机头方向的旋转角 (CAM_TOP_YAW_DEG, 见image.h)
    // 先绕光轴把相机系 (x=Right, y=Forward) 转到机体系, 再做倾角补偿。
    // (F', R') = R_z(φ)·(cam.y, cam.x);  φ=0 时与旧代码完全一致
    double sinq = sin(CAM_TOP_YAW_DEG * M_PI / 180.0);
    double cosq = cos(CAM_TOP_YAW_DEG * M_PI / 180.0);

    // 映射输入向量到中间物理坐标系 (Forward, Right, Down) 以便使用标准旋转公式
    // pixelTo3DRay 输出: x=Right (Body Y), y=Forward (Body X), z=Up
    // 物理坐标: xb=Forward, yb=Right, zb=Down
    double xb = cam->y * cosq - cam->x * sinq;  // Forward
    double yb = cam->y * sinq + cam->x * cosq;  // Right
    double zb = -cam->z;     // Down (取反，因为cam->z是负的)

    // 1. 应用 Roll (绕 Forward/X 轴旋转)
    // 右翼下压(Roll>0) -> 相机右侧下沉 -> 需将向量逆向旋转回水平
    double xt = xb;
    double yt = yb * cosr - zb * sinr;
    double zt = yb * sinr + zb * cosr;
    
    // 2. 应用 Pitch (绕 Right/Y 轴旋转) -> 影响 Forward(X) 和 Down(Z)
    // 机头抬起(Pitch>0) -> 相机前视上扬 -> 需将向量压回水平
    double xw = xt * cosp + zt * sinp;
    double zw = -xt * sinp + zt * cosp;
    double yw = yt;
    
    // 映射回输出向量 (保持 pixelTo3DRay 的格式: x=Right, y=Forward, z=Up/NegDown)
    body.x = yw;
    body.y = xw;
    body.z = -zw;

    return body;
}

// ==========================================
// 4. 将射线投影到真实水平地面
// ==========================================
// 基于相似三角形原理计算地面坐标
static GroundPoint projectToGround(Vector3D ray, double height) {
    GroundPoint ground_pt = {0.0, 0.0};
    
    // 防止射线指向水平线以上或完全水平
    if (ray.z > -0.01) {
        ray.z = -0.01;
    }

    double scale = -height / ray.z;

    // ray.x 是 Right, ray.y 是 Forward
    // 目标: ground_pt.x 是 Forward, ground_pt.y 是 Right
    ground_pt.x = ray.y * scale; // X = Forward (cm)
    ground_pt.y = ray.x * scale; // Y = Right (cm)
    
    // 硬限幅最大有效地面距离
    double dist_sq = ground_pt.x * ground_pt.x + ground_pt.y * ground_pt.y;
    if (dist_sq > MAX_DIST * MAX_DIST) {
        double limit_scale = MAX_DIST / sqrt(dist_sq);
        ground_pt.x *= limit_scale;
        ground_pt.y *= limit_scale;
    }

    return ground_pt;
}


// ==========================================
// 5. 精确计算单点的物理距离 (供图像处理使用)
// ==========================================
void get_accurate_ground_distance(double u, double v, double height, double pitch_deg, double roll_deg, double *out_x, double *out_y, double *out_dist) {
    double p_rad = pitch_deg * M_PI / 180.0;
    double r_rad = roll_deg * M_PI / 180.0;
    double sinp = sin(p_rad), cosp = cos(p_rad);
    double sinr = sin(r_rad), cosr = cos(r_rad);

    Vector3D ray = pixelTo3DRay(u, v);
    Vector3D body = cameraToBody(&ray, sinp, cosp, sinr, cosr);
    GroundPoint pt = projectToGround(body, height);

    if (out_x) *out_x = pt.x;
    if (out_y) *out_y = pt.y;
    if (out_dist) *out_dist = sqrt(pt.x * pt.x + pt.y * pt.y);
}

// ==========================================
// 6. 计算地面坐标主函数
// ==========================================
// 输出坐标: x 向前、y 向右，单位 cm
void calculate_ground_positions(double height, double pitch_deg, double roll_deg, double yaw_deg) {
    // 提前计算本帧统一的正余弦，避免目标循环中重复计算耗时
    double p_rad = pitch_deg * M_PI / 180.0;
    double r_rad = roll_deg * M_PI / 180.0;
    double y_rad = yaw_deg * M_PI / 180.0;
    double sinp = sin(p_rad), cosp = cos(p_rad);
    double sinr = sin(r_rad), cosr = cos(r_rad);
    double siny = sin(y_rad), cosy = cos(y_rad);

    
    // ================== 小车 ==================
    if (cam_down.car_valid) { 
        // pixelTo3DRay 参数顺序为 (u, v) 即 (Col, Row)
        Vector3D ray_car = pixelTo3DRay((double)cam_down.car_center_x, (double)cam_down.car_center_y);
        Vector3D body_car = cameraToBody(&ray_car, sinp, cosp, sinr, cosr);
        double car_plane_height = height - CAR_LIGHT_HEIGHT_CM;
        if (car_plane_height < 0.0) car_plane_height = 0.0;
        pos.raw_car = projectToGround(body_car, car_plane_height);

        // 将相对于机头的XY旋转到无人机上电航向固定系后再滤波（不是绝对North/East）
        // 防止机体旋转时相对坐标波动导致卡尔曼滤波产生巨大滞后
        double raw_earth_x = pos.raw_car.x * cosy - pos.raw_car.y * siny;
        double raw_earth_y = pos.raw_car.x * siny + pos.raw_car.y * cosy;

        // if (fabs(raw_earth_x - K_car_x.x) > 50.0 || fabs(raw_earth_y - K_car_y.x) > 50.0) {
        //     Kalman_Reset_State(&K_car_x, raw_earth_x);
        //     Kalman_Reset_State(&K_car_y, raw_earth_y);
        // }

        double k_earth_x = raw_earth_x;
        double k_earth_y = raw_earth_y;
        if (!car_position_filter_initialized) {
            Kalman_Init(&K_car_x, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, (float)raw_earth_x);
            Kalman_Init(&K_car_y, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, (float)raw_earth_y);
            car_position_filter_initialized = 1;
        } else {
            k_earth_x = Kalman_Update(&K_car_x, raw_earth_x);
            k_earth_y = Kalman_Update(&K_car_y, raw_earth_y);
        }
        
        // 滤波结束后再转回相对于机头的坐标，保持与飞控代码的接口兼容
        pos.k_car.x = k_earth_x * cosy + k_earth_y * siny;
        pos.k_car.y = -k_earth_x * siny + k_earth_y * cosy;

    }
    // 注意：若未识别到，保持上一帧位置

    // ================== 信标 ==================
    if (cam_down.target_valid && cam_down.target_count > 0U) {
        uint8_t target_count = cam_down.target_count;
        if (target_count > TARGET_CANDIDATE_COUNT) target_count = TARGET_CANDIDATE_COUNT;

        for (uint8_t i = 0; i < target_count; i++) {
            Vector3D ray_target = pixelTo3DRay((double)cam_down.target_centers[i][1],
                                              (double)cam_down.target_centers[i][0]);
            Vector3D body_target = cameraToBody(&ray_target, sinp, cosp, sinr, cosr);
            pos.raw_target[i] = projectToGround(body_target, height);
        }
        for (uint8_t i = target_count; i < TARGET_CANDIDATE_COUNT; i++) {
            pos.raw_target[i] = pos.raw_target[target_count - 1U];
        }

        double raw_earth_x = pos.raw_target[0].x * cosy - pos.raw_target[0].y * siny;
        double raw_earth_y = pos.raw_target[0].x * siny + pos.raw_target[0].y * cosy;
        double k_earth_x = raw_earth_x;
        double k_earth_y = raw_earth_y;
        if (!target_position_filter_initialized) {
            Kalman_Init(&K_target_x, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, (float)raw_earth_x);
            Kalman_Init(&K_target_y, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, (float)raw_earth_y);
            target_position_filter_initialized = 1;
        } else {
            k_earth_x = Kalman_Update(&K_target_x, raw_earth_x);
            k_earth_y = Kalman_Update(&K_target_y, raw_earth_y);
        }

        pos.k_target.x = k_earth_x * cosy + k_earth_y * siny;
        pos.k_target.y = -k_earth_x * siny + k_earth_y * cosy;
    } else {
        memset(pos.raw_target, 0, sizeof(pos.raw_target));
    }

    // 车和信标使用相同参数的Kalman结果，避免不同滤波相位污染相对距离。
    if (cam_down.car_valid && cam_down.target_valid &&
        car_position_filter_initialized && target_position_filter_initialized) {
        double dx = pos.k_car.x - pos.k_target.x;
        double dy = pos.k_car.y - pos.k_target.y;
        dataC.car_target_dist = (float)sqrt(dx * dx + dy * dy);
    }
}
