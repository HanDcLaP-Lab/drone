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
//   │ 2. cameraToBody()        姿态旋转 (pitch/roll补偿)│
//   │ 3. projectToGround()     相似三角形投影到水平地面   │
//   │ 4. 地球系旋转 + 卡尔曼滤波 + 转回机体系             │
//   └──────────────────────────────────────────────────┘
//
// 输出: pos.car (小车位置), pos.target (信标位置), pos.k_car (滤波后)
// 单位: cm (取决于传入的 height 参数单位)
// 关键: 在大地坐标系下做卡尔曼滤波，避免机体旋转引起的滞后
// ******************************************************************************

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// 1. 常量参数
// ==========================================
// 畸变中心 (Distortion Center)
const double CX = 95.5754542924;
const double CY = 56.0345163934;

// 逆拉伸矩阵 (Inverse Stretch Matrix)
const double INV_S11 = 1.0000000000;
const double INV_S12 = 0.0000000000;
const double INV_S21 = 0.0000000000;
const double INV_S22 = 1.0000000000;

// 映射多项式系数 (Mapping Coefficients)
const double A0 = 53.5431129928;
const double A2 = -0.0157824974;
const double A3 = 0.0002786109;
const double A4 = -0.0000044018;

// 定义 3D 空间向量
typedef struct { double x, y, z; } Vector3D;

// 全局变量定义
GroundPos pos = {0};

// ==========================================
// 2. 核心算法：像素坐标 -> 3D 空间射线 (指向地面)
// ==========================================
// 输出坐标系定义 (Camera Frame):
// X: 相机右侧 (Right)
// Y: 相机前方 (Forward, 图像上方)
// Z: 相机上方 (Up, 实际指向相机内部，地面点此处为负值)
static Vector3D pixelTo3DRay(double u, double v) {
    Vector3D ray;
    
    double u_prime = u - CX;
    double v_prime = v - CY;

    // 转换为相机物理坐标 (X向右, Y向前)
    double x = INV_S11 * u_prime + INV_S12 * v_prime;
    double y = -(INV_S21 * u_prime + INV_S22 * v_prime); // 取负，使得 Y 正向为前方

    double rho = sqrt(x * x + y * y);
    double rho2 = rho * rho;
    double z_poly = A0 + A2 * rho2 + A3 * rho2*rho + A4 * rho2*rho2;

    // 构建射线：X为右，Y为前。由于 A0 是正的，z_poly 指向相机内部(上)。
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

    // 映射输入向量到中间物理坐标系 (Forward, Right, Down) 以便使用标准旋转公式
    // pixelTo3DRay 输出: x=Right (Body Y), y=Forward (Body X), z=Up
    // 物理坐标: xb=Forward, yb=Right, zb=Down
    double xb = cam->y;      // Forward
    double yb = cam->x;      // Right
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
// 输出: pos.car.x (前), pos.car.y (右) 单位: cm (取决于height单位)
void calculate_ground_positions(double height, double pitch_deg, double roll_deg, double yaw_deg) {
    const double k = 0.8; 
    
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
        pos.raw_car = projectToGround(body_car, height);

        // 将相对于机头的 XY 旋转为大地绝对坐标 North/East 后再滤波
        // 防止机体旋转时相对坐标波动导致卡尔曼滤波产生巨大滞后
        double raw_earth_x = pos.raw_car.x * cosy - pos.raw_car.y * siny;
        double raw_earth_y = pos.raw_car.x * siny + pos.raw_car.y * cosy;

        // if (fabs(raw_earth_x - K_car_x.x) > 50.0 || fabs(raw_earth_y - K_car_y.x) > 50.0) {
        //     Kalman_Reset_State(&K_car_x, raw_earth_x);
        //     Kalman_Reset_State(&K_car_y, raw_earth_y);
        // }

        double k_earth_x = Kalman_Update(&K_car_x, raw_earth_x);
        double k_earth_y = Kalman_Update(&K_car_y, raw_earth_y);
        
        // 滤波结束后再转回相对于机头的坐标，保持与飞控代码的接口兼容
        pos.k_car.x = k_earth_x * cosy + k_earth_y * siny;
        pos.k_car.y = -k_earth_x * siny + k_earth_y * cosy;

        pos.car.x = pos.car.x * (1.0 - k) + pos.raw_car.x * k;
        pos.car.y = pos.car.y * (1.0 - k) + pos.raw_car.y * k;

    }
    // 注意：若未识别到，保持上一帧位置

    // ================== 信标 ==================
    if (cam_down.target_valid) { 
        Vector3D ray_target = pixelTo3DRay((double)cam_down.target_center_x, (double)cam_down.target_center_y);
        Vector3D body_target = cameraToBody(&ray_target, sinp, cosp, sinr, cosr);
        pos.raw_target = projectToGround(body_target, height);
        
        pos.target.x = pos.target.x * (1.0 - k) + pos.raw_target.x * k;
        pos.target.y = pos.target.y * (1.0 - k) + pos.raw_target.y * k;
    }

    // 计算双目标直线距离
    if (cam_down.car_valid && cam_down.target_valid) {
        double dx = pos.car.x - pos.target.x;
        double dy = pos.car.y - pos.target.y;
        dataC.car_target_dist = (float)sqrt(dx * dx + dy * dy);
    }
}