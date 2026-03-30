#include "image_process.h"
#include "image.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// 1. 常量参数
// ==========================================
// 畸变中心 (Distortion Center)
const double CX = 94.0532845692;
const double CY = 59.6200034349;

// 逆拉伸矩阵 (Inverse Stretch Matrix)
const double INV_S11 = 1.0000000000;
const double INV_S12 = 0.0000000000;
const double INV_S21 = 0.0000000000;
const double INV_S22 = 1.0000000000;

// 映射多项式系数 (Mapping Coefficients)
const double A0 = 75.0109988547;
const double A2 = -0.0058158086;
const double A3 = 0.0000239113;
const double A4 = -0.0000003180;

// 定义 3D 空间向量
typedef struct { double x, y, z; } Vector3D;

// 全局变量定义
GroundPoint car_ground_pos = {0.0, 0.0};
GroundPoint target_ground_pos = {0.0, 0.0};

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
    if (ray.z >= 0) return ground_pt;

    double scale = -height / ray.z;

    // ray.x 是 Right, ray.y 是 Forward
    // 目标: ground_pt.x 是 Forward, ground_pt.y 是 Right
    ground_pt.x = ray.y * scale; // X = Forward (cm)
    ground_pt.y = ray.x * scale; // Y = Right (cm)
    return ground_pt;
}

// ==========================================
// 5. 计算地面坐标主函数
// ==========================================
// 输出: car_ground_pos.x (前), car_ground_pos.y (右) 单位: cm (取决于height单位)
void calculate_ground_positions(double height, double pitch_deg, double roll_deg) {
    const double k = 0.5; 
    extern volatile float share_data_from_1[]; 
    static float k_car_ground_pos_x,k_car_ground_pos_y = 0;
    
    // 提前计算本帧统一的正余弦，避免目标循环中重复计算耗时
    double p_rad = pitch_deg * M_PI / 180.0;
    double r_rad = roll_deg * M_PI / 180.0;
    double sinp = sin(p_rad), cosp = cos(p_rad);
    double sinr = sin(r_rad), cosr = cos(r_rad);

    // ================== 小车 ==================
    if (cam_down.car_valid) { 
        // pixelTo3DRay 参数顺序为 (u, v) 即 (Col, Row)
        Vector3D ray_car = pixelTo3DRay((double)cam_down.car_center_x, (double)cam_down.car_center_y);
        Vector3D body_car = cameraToBody(&ray_car, sinp, cosp, sinr, cosr);
        GroundPoint raw_car = projectToGround(body_car, height);

        k_car_ground_pos_x = Kalman_Update(&K_car_x, raw_car.x);
        k_car_ground_pos_y = Kalman_Update(&K_car_y, raw_car.y);
        
        car_ground_pos.x = car_ground_pos.x * (1.0 - k) + raw_car.x * k;
        car_ground_pos.y = car_ground_pos.y * (1.0 - k) + raw_car.y * k;

        share_data_from_1[7] = ray_car.x;
        share_data_from_1[8] = ray_car.y;
        share_data_from_1[9] = k_car_ground_pos_x;
        share_data_from_1[10] = body_car.x;
        share_data_from_1[11] = body_car.y;
        share_data_from_1[12] = k_car_ground_pos_y;
    }
    // 注意：若未识别到，保持上一帧位置

    // ================== 信标 ==================
    if (cam_down.target_valid) { 
        Vector3D ray_target = pixelTo3DRay((double)cam_down.target_center_x, (double)cam_down.target_center_y);
        Vector3D body_target = cameraToBody(&ray_target, sinp, cosp, sinr, cosr);
        GroundPoint raw_target = projectToGround(body_target, height);
        
        target_ground_pos.x = target_ground_pos.x * (1.0 - k) + raw_target.x * k;
        target_ground_pos.y = target_ground_pos.y * (1.0 - k) + raw_target.y * k;
    }

    // 计算双目标直线距离
    if (cam_down.car_valid && cam_down.target_valid) {
        double dx = car_ground_pos.x - target_ground_pos.x;
        double dy = car_ground_pos.y - target_ground_pos.y;
        share_data_from_1[13] = (float)sqrt(dx * dx + dy * dy);
    }
}