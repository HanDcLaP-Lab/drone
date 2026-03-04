#include "image_process.h"
#include "image.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// 1. 常量参数
// ==========================================
const double CX = 94.3964870095;
const double CY = 56.7202594062;

const double INV_S11 = 1.0000000000;
const double INV_S12 = 0.0000000000;
const double INV_S21 = 0.0000000000;
const double INV_S22 = 1.0000000000;

const double A0 = 68.5826350751;
const double A2 = -0.0051614074;
const double A3 = 0.0000196348;
const double A4 = -0.0000002898;

// 定义 3D 空间向量
typedef struct { double x, y, z; } Vector3D;

// 全局变量定义
GroundPoint car_ground_pos = {0.0, 0.0};
GroundPoint target_ground_pos = {0.0, 0.0};

// ==========================================
// 2. 核心算法：像素坐标 -> 3D 空间射线 (指向地面)
// ==========================================
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
static Vector3D cameraToBody(const Vector3D *cam, double pitch_deg, double roll_deg) {
    Vector3D body;
    double p = pitch_deg * M_PI / 180.0;
    double r = roll_deg * M_PI / 180.0;
    
    double sinp = sin(p), cosp = cos(p);
    double sinr = sin(r), cosr = cos(r);

    // 映射输入向量到物理坐标系 (Forward, Right, Down)
    // pixelTo3DRay 输出: x=Right (Body Y), y=Forward (Body X), z=Up
    // 物理坐标: xb=Forward, yb=Right, zb=Down
    double xb = cam->y;      // Forward
    double yb = cam->x;      // Right
    double zb = -cam->z;     // Down (取反，因为cam->z是负的)

    // 1. 应用 Roll (绕 Forward/X 轴旋转)
    // 右翼下压(Roll>0) -> Right向量向下偏(+Z)
    double xt = xb;
    double yt = yb * cosr - zb * sinr;
    double zt = yb * sinr + zb * cosr;
    
    // 2. 应用 Pitch (绕 Right/Y 轴旋转) -> 影响 Forward(X) 和 Down(Z)
    // 机头抬起(Pitch>0) -> Forward向量向上偏(-Z)
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
static GroundPoint projectToGround(Vector3D ray, double height) {
    GroundPoint ground_pt = {0.0, 0.0};
    if (ray.z >= 0) return ground_pt; 
    
    double scale = -height / ray.z;
    
    // 修正映射关系以符合 README (X前 Y右)
    // ray.y 是 Forward, ray.x 是 Right
    ground_pt.x = ray.y * scale; // X = Forward
    ground_pt.y = ray.x * scale; // Y = Right
    return ground_pt;
}

// ==========================================
// 5. 计算地面坐标主函数
// ==========================================
void calculate_ground_positions(double height, double pitch_deg, double roll_deg) {
    // 小车 (Index 0): image.h 中定义 centers[i][0] 为 row (v), centers[i][1] 为 col (u)
    Vector3D ray_car = pixelTo3DRay((double)cam_down.centers[0][1], (double)cam_down.centers[0][0]);
    Vector3D body_car = cameraToBody(&ray_car, pitch_deg, roll_deg);
    car_ground_pos = projectToGround(body_car, height);

    // 目标 (Index 1)
    Vector3D ray_target = pixelTo3DRay((double)cam_down.centers[1][1], (double)cam_down.centers[1][0]);
    Vector3D body_target = cameraToBody(&ray_target, pitch_deg, roll_deg);
    target_ground_pos = projectToGround(body_target, height);
}