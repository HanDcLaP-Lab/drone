#include "display.h"
#include "zf_common_headfile.h"
#include "key_switch.h"

#define MERGE_MARK_HOLD_MS 1000U

void display_init()
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}
void display_motor_output_display()
{
  //电机输出
    ips200_show_string(0,16*2,"lf:");
    ips200_show_int(40,16*2,motor_out.lf, 4);
    ips200_show_string(120,16*2,"rf:");
    ips200_show_int(160,16*2,motor_out.rf, 4);
    ips200_show_string(0,16*3,"lb:");
    ips200_show_int(40,16*3,motor_out.lb, 4);
    ips200_show_string(120,16*3,"rb:");
    ips200_show_int(160,16*3,motor_out.rb, 4);
    
  
    ips200_show_string(0,16*4,"y:");
    ips200_show_float(40,16*4 , cam_down.centers[0][0], 3, 2); // [修改] 显示浮点数坐标
    ips200_show_string(120,16*4,"x:");
    ips200_show_float(160,16*4 , cam_down.centers[0][1], 3, 2); // [修改] 显示浮点数坐标
    
    ips200_show_string(0,16*5,"t_p:");
    ips200_show_float(40,16*5 , flight_target.target_pitch, 3,2);
    ips200_show_string(120,16*5,"t_r:");
    ips200_show_float(160,16*5 , flight_target.target_roll, 3,2);

    // //imu数据
    ips200_show_string(0,16*7,"Ro:");
    ips200_show_float(40,16*7 , imu_data.roll, 3,2);
    ips200_show_string(120,16*7,"Pi:");
    ips200_show_float(160,16*7 , imu_data.pitch, 3,2);
    ips200_show_string(0,16*8,"Ya:");
    ips200_show_float(40,16*8 , imu_data.yaw, 3,2);
    ips200_show_string(120,16*8,"z:");
    ips200_show_float(160,16*8 , imu_data.z, 3,2);
    
    ips200_show_string(0,16*9,"GR:");
    ips200_show_int(40,16*9 , (int)imu_data.groll, 4);
    ips200_show_string(120,16*9,"GP:");
    ips200_show_int(160,16*9 , (int)imu_data.gpitch, 4);
    ips200_show_string(0,16*10,"GY:");
    ips200_show_int(40,16*10 , (int)imu_data.gyaw, 4);

    ips200_show_string(0,16*11,"a_kp:");
    ips200_show_float(40,16*11 , pid_roll.kp, 4 , 2);
    ips200_show_string(120,16*11,"a_ki:");
    ips200_show_float(160,16*11 , pid_roll.ki, 4 , 2);
    ips200_show_string(0,16*12,"a_kp2:");
    ips200_show_float(40,16*12 , pid_roll.kp2, 4 , 2);
    ips200_show_string(120,16*12,"g_kp:");
    ips200_show_float(160,16*12 , pid_g_roll.kp, 4 , 2);
    ips200_show_string(0,16*13,"g_ki:");
    ips200_show_float(40,16*13 , pid_g_roll.ki, 4 , 2);
    ips200_show_string(120,16*13,"g_kd:");
    ips200_show_float(160,16*13 , pid_g_roll.kd, 4 , 2);

    ips200_show_string(0,16*15,"v_kp:");
    ips200_show_float(40,16*15 , pid_image_x.kp, 4 , 2);
    ips200_show_string(120,16*15,"v_ki:");
    ips200_show_float(160,16*15 , pid_image_x.ki, 4 , 2);
    ips200_show_string(0,16*16,"v_kd:");
    ips200_show_float(40,16*16 , pid_image_x.kd, 4 , 2);
    ips200_show_string(120,16*16,"v_kp2:");
    ips200_show_float(160,16*16 , pid_image_x.kp2, 4 , 2);
    
    ips200_show_string(0,16*17,"x:");
    ips200_show_float(40,16*17, share_data_from_1[S1_K_CAR_X], 4, 2);
    ips200_show_string(120,16*17,"y:");
    ips200_show_float(160,16*17, share_data_from_1[S1_K_CAR_Y], 4, 2);
}

void display_image_display(void){
    ips200_show_string(0,16*1,"x:");
    ips200_show_float(40,16*1, share_data_from_1[S1_CAR_CENTER_X], 4, 2);
    ips200_show_string(120,16*1,"y:");
    ips200_show_float(160,16*1, share_data_from_1[S1_CAR_CENTER_Y], 4, 2);

    ips200_show_string(0,16*3,"cx:");
    ips200_show_float(40,16*3, share_data_from_1[S1_CAR_RAW_X], 4, 2);
    ips200_show_string(120,16*3,"cy:");
    ips200_show_float(160,16*3, share_data_from_1[S1_CAR_RAW_Y], 4, 2);
    ips200_show_string(0,16*4,"tx:");
    ips200_show_float(40,16*4, share_data_from_1[S1_TARGET_X], 4, 2);
    ips200_show_string(120,16*4,"ty:");
    ips200_show_float(160,16*4, share_data_from_1[S1_TARGET_Y], 4, 2);

    ips200_show_string(0,16*5,"x:");
    ips200_show_float(40,16*5, share_data_from_1[S1_RAW_CAR_X], 4, 2);
    ips200_show_string(120,16*5,"y:");
    ips200_show_float(160,16*5, share_data_from_1[S1_SNAPSHOT_YAW], 4, 2);
    ips200_show_string(0,16*6,"z:");
    ips200_show_float(40,16*6, share_data_from_1[S1_K_CAR_X], 4, 2);

    ips200_show_string(0,16*7,"x:");
    ips200_show_float(40,16*7, share_data_from_1[10], 4, 2); // reserved
    ips200_show_string(120,16*7,"y:");
    ips200_show_float(160,16*7, share_data_from_1[11], 4, 2); // reserved
    ips200_show_string(0,16*8,"z:");
    ips200_show_float(40,16*8, share_data_from_1[S1_K_CAR_Y], 4, 2);

    ips200_show_string(0,16*9,"Ro:");
    ips200_show_float(40,16*9 , imu_data.roll, 3,2);
    ips200_show_string(120,16*9,"Pi:");
    ips200_show_float(160,16*9 , imu_data.pitch, 3,2);
    ips200_show_string(0,16*10,"Ya:");
    ips200_show_float(40,16*10 , imu_data.yaw, 3,2);
    ips200_show_string(120,16*10,"z:");
    ips200_show_float(160,16*10 , imu_data.z, 3,2);

    ips200_show_string(0,16*12,"di:");
    ips200_show_float(40,16*12 , share_data_from_1[S1_CAR_TARGET_DIST], 4,2);
}

void display_image_debug_display(float car_x, float car_y, uint8_t car_valid, float target_x, float target_y, uint8_t target_valid){
    static uint8_t last_display_locked_state = 0;
    static uint8_t merge_mark_active = 0;
    static uint32_t merge_mark_start_ms = 0;
    uint8_t display_locked_state = (uint8_t)share_data_from_1[S1_LOCKED_COUNT];
    uint32_t now_ms = dataC.pit0_cnt;

    if (display_locked_state == 4 && last_display_locked_state != 4) {
        merge_mark_start_ms = now_ms;
        merge_mark_active = 1;
    }
    last_display_locked_state = display_locked_state;
    if (merge_mark_active && (uint32_t)(now_ms - merge_mark_start_ms) >= MERGE_MARK_HOLD_MS) {
        merge_mark_active = 0;
    }

    // 将底层的 0/1 放大为 0/255 以便屏幕显示
    for(int i = 0; i < MT9V03X_H * MT9V03X_W; i++) {
        image_copy[0][i] = cam_down.binarized_image[i] ? 255 : 0;
    }
    
    // 将要显示的数组刷入内存供 DMA 搬运
    SCB_CleanDCache_by_Addr((void*)image_copy, sizeof(image_copy));
    
    // 显示二值化图像
    ips200_displayimage03x((const uint8 *)image_copy , MT9V03X_W, MT9V03X_H);
    
    // [新增] 叠加显示小车和信标的十字准星
    // 检查小车是否有效
    if (car_valid) {
        int16_t cx = (int16_t)car_x;
        int16_t cy = (int16_t)car_y;
        
        int16_t radius = 4; // 长度扩大一倍
        for (int16_t offset = 0; offset <= 1; offset++) { // 粗细扩大一倍
            // 画水平线（带边缘截断）
            int16_t y = cy + offset;
            if (y >= 0 && y < MT9V03X_H) {
                int16_t x1 = cx - radius < 0 ? 0 : cx - radius;
                int16_t x2 = cx + radius >= MT9V03X_W ? MT9V03X_W - 1 : cx + radius;
                if (x1 <= x2) ips200_draw_line(x1, y, x2, y, RGB565_RED);
            }
            
            // 画垂直线（带边缘截断）
            int16_t x = cx + offset;
            if (x >= 0 && x < MT9V03X_W) {
                int16_t y1 = cy - radius < 0 ? 0 : cy - radius;
                int16_t y2 = cy + radius >= MT9V03X_H ? MT9V03X_H - 1 : cy + radius;
                if (y1 <= y2) ips200_draw_line(x, y1, x, y2, RGB565_RED);
            }
        }
    }

    // 检查信标是否有效
    if (target_valid) {
        int16_t tx = (int16_t)target_x;
        int16_t ty = (int16_t)target_y;

        int16_t radius = 4; // 长度扩大一倍
        for (int16_t offset = 0; offset <= 1; offset++) { // 粗细扩大一倍
            // 画水平线（带边缘截断）
            int16_t y = ty + offset;
            if (y >= 0 && y < MT9V03X_H) {
                int16_t x1 = tx - radius < 0 ? 0 : tx - radius;
                int16_t x2 = tx + radius >= MT9V03X_W ? MT9V03X_W - 1 : tx + radius;
                if (x1 <= x2) ips200_draw_line(x1, y, x2, y, RGB565_GREEN); // 改为绿色
            }
            
            // 画垂直线（带边缘截断）
            int16_t x = tx + offset;
            if (x >= 0 && x < MT9V03X_W) {
                int16_t y1 = ty - radius < 0 ? 0 : ty - radius;
                int16_t y2 = ty + radius >= MT9V03X_H ? MT9V03X_H - 1 : ty + radius;
                if (y1 <= y2) ips200_draw_line(x, y1, x, y2, RGB565_GREEN); // 改为绿色
            }
        }
    }
    
    // 显示页面号
    ips200_show_string(160, 16*10, "P:");
    ips200_show_int(176, 16*10, display_page_idx, 1);
    ips200_show_string(160, 16*18, "M:");
    ips200_show_int(176, 16*18, merge_mark_active, 1);

    // 页面0：核心状态
    if(display_page_idx == 0)
    {
        ips200_show_string(0, 16*10, "Thresh:");
        ips200_show_int(80, 16*10, cam_down.threshold, 3);

        ips200_show_string(0, 16*11, "Car A:");
        ips200_show_int(50, 16*11, cam_down.car_dot_num, 4);
        ips200_show_string(100, 16*11, "R:"); // Ratio
        ips200_show_float(120, 16*11, cam_down.car_ratio, 2, 2);

        ips200_show_string(0, 16*12, "Tgt A:");
        ips200_show_int(50, 16*12, cam_down.target_dot_num, 4);
        ips200_show_string(100, 16*12, "R:"); 
        ips200_show_float(120, 16*12, cam_down.target_ratio, 2, 2);

        ips200_show_string(0, 16*13, "MaxR:");
        ips200_show_float(40, 16*13, cam_down.debug.max_ratio, 3, 2);
        ips200_show_string(90, 16*13, "MinR:");
        ips200_show_float(130, 16*13, cam_down.debug.min_ratio, 3, 2);

        ips200_show_string(0, 16*14, "H:");
        ips200_show_int(40, 16*14, (int)share_data_from_0[S0_IMU_HEIGHT], 4);
        
        ips200_show_string(0, 16*15, "kX:");
        ips200_show_float(40, 16*15, pos.k_car.x, 4, 2);
        ips200_show_string(100, 16*15,"kY:");
        ips200_show_float(140, 16*15, pos.k_car.y, 4, 2);

        ips200_show_string(0, 16*16, "TR:");
        ips200_show_float(40, 16*16, share_data_from_0[S0_TARGET_ROLL], 4, 2);
        ips200_show_string(100, 16*16, "TP:");
        ips200_show_float(140, 16*16, share_data_from_0[S0_TARGET_PITCH], 4, 2);

        ips200_show_string(0, 16*17, "iR:");
        ips200_show_float(40, 16*17, share_data_from_0[S0_IMU_ROLL], 4, 2);
        ips200_show_string(100, 16*17, "iP:");
        ips200_show_float(140, 16*17, share_data_from_0[S0_IMU_PITCH], 4, 2);
        ips200_show_string(0, 16*18, "iY:");
        ips200_show_float(40, 16*18, share_data_from_0[S0_IMU_YAW], 4, 2);
    }
    // 页面1：亮度与电机
    else if(display_page_idx == 1)
    {

        ips200_show_string(0, 16*14, "LF:");
        ips200_show_int(40, 16*14, (int)share_data_from_0[S0_MOTOR_LF], 4);
        ips200_show_string(100, 16*14, "RF:");
        ips200_show_int(140, 16*14, (int)share_data_from_0[S0_MOTOR_RF], 4);
        ips200_show_string(0, 16*15, "LB:");
        ips200_show_int(40, 16*15, (int)share_data_from_0[S0_MOTOR_LB], 4);
        ips200_show_string(100, 16*15, "RB:");
        ips200_show_int(140, 16*15, (int)share_data_from_0[S0_MOTOR_RB], 4);
    }
}
