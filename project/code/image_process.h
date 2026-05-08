#ifndef CODE_IMAGE_PROCESS_H_
#define CODE_IMAGE_PROCESS_H_

#include "zf_common_headfile.h"

#define MAX_DIST 1500.0


// 定义地面点结构体
typedef struct {
    double x;
    double y;
} GroundPoint;

typedef struct {
    GroundPoint raw_car;
    GroundPoint raw_target;
    GroundPoint k_car;
    GroundPoint k_target;
    GroundPoint target;
    GroundPoint car;
}GroundPos;
// 全局变量，存储计算后的地面坐标 (供 M7_1_data_send 使用)
extern GroundPos pos;

// 计算地面坐标主函数
void calculate_ground_positions(double height, double pitch_deg, double roll_deg, double yaw_deg);

#endif