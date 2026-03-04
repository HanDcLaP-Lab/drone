#ifndef CODE_IMAGE_PROCESS_H_
#define CODE_IMAGE_PROCESS_H_

#include "zf_common_headfile.h"

// 定义地面点结构体
typedef struct {
    double x;
    double y;
} GroundPoint;

// 全局变量，存储计算后的地面坐标 (供 M7_1_data_send 使用)
extern GroundPoint car_ground_pos;
extern GroundPoint target_ground_pos;

// 计算地面坐标主函数
void calculate_ground_positions(double height, double pitch_deg, double roll_deg);

#endif