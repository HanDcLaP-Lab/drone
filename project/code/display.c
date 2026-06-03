#include "display.h"
#include "zf_common_headfile.h"

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
    //ips200_show_float(40,16*12 , share_data_from_1[S1_CAR_TARGET_DIST], 4,2);
}

void display_image_debug_display(void){
    // 将底层的 0/1 放大为 0/255 以便屏幕显示
    for(int i = 0; i < MT9V03X_H * MT9V03X_W; i++) {
        image_copy[0][i] = cam_down.binarized_image[i] ? 255 : 0;
    }
    
    // 将要显示的数组刷入内存供 DMA 搬运
    SCB_CleanDCache_by_Addr((void*)image_copy, sizeof(image_copy));
    
    // 显示二值化图像 (假设全屏大小为 188x120)
    ips200_displayimage03x((const uint8 *)image_copy , MT9V03X_W, MT9V03X_H);
    
    // 在屏幕下方显示状态与阈值
    // ips200_show_string(0, 16*9, "Mode: DEBUG");

    // if (current_param_idx == 0) {
    //     ips200_show_string(0, 16*10, "-> Thresh:"); // 带有指示箭头代表当前高亮选中
    // } else {
    //     ips200_show_string(0, 16*10, "   Thresh:"); // 未选中时用空格对齐
    // }
    ips200_show_int(80, 16*10, cam_down.threshold, 3);
    // ips200_show_float(0, 16*9, share_data_from_0[S0_DEBUG_ERR_X], 2, 2);
    // ips200_show_float(100, 16*9, share_data_from_0[S0_DEBUG_ERR_Y], 2, 2);
    // ips200_show_float(0, 16*10, share_data_from_0[S0_TARGET_ROLL], 2, 2);
    // ips200_show_float(60, 16*10, share_data_from_0[S0_TARGET_PITCH], 2, 2);
    // ips200_show_float(120, 16*10, share_data_from_0[S0_TARGET_YAW], 2, 2);


    ips200_show_string(0, 16*11, "L1 A:");
    ips200_show_int(40, 16*11, cam_down.car_dot_num, 4);
    ips200_show_string(80, 16*11, "R:"); // Ratio 长宽比
    ips200_show_float(100, 16*11, cam_down.car_ratio, 2, 2);

    // 打印 2 号灯 (面积次大的灯) 的数据
    ips200_show_string(0, 16*12, "L2 A:");
    ips200_show_int(40, 16*12, cam_down.target_dot_num, 4);
    ips200_show_string(80, 16*12, "R:"); 
    ips200_show_float(100, 16*12, cam_down.target_ratio, 2, 2);


    ips200_show_string(0, 16*13, "MaxR:");
    ips200_show_float(40, 16*13, cam_down.debug.max_ratio, 3, 2);
    ips200_show_string(90, 16*13, "MinR:");
    ips200_show_float(130, 16*13, cam_down.debug.min_ratio, 3, 2);

    
    ips200_show_string(0, 16*14, "z:");
    ips200_show_int(40, 16*14, (int)share_data_from_0[S0_IMU_HEIGHT], 4);
    ips200_show_string(80, 16*14, "RF:");
    ips200_show_int(120, 16*14, (int)share_data_from_0[S0_MOTOR_RF], 4);
    ips200_show_string(0, 16*15, "LB:");
    ips200_show_int(40, 16*15, (int)share_data_from_0[S0_MOTOR_LB], 4);
    ips200_show_string(80, 16*15, "RB:");
    ips200_show_int(120, 16*15, (int)share_data_from_0[S0_MOTOR_RB], 4);

    ips200_show_float(0, 16*16, pos.k_car.x, 4,2);
    ips200_show_float(80, 16*16,pos.k_car.y, 4,2);

    ips200_show_float(0, 16*17, share_data_from_0[S0_TARGET_ROLL], 4,2);
    ips200_show_float(80, 16*17, share_data_from_0[S0_TARGET_PITCH], 4,2);
    ips200_show_float(0, 16*18, share_data_from_0[S0_IMU_ROLL], 4,2);
    ips200_show_float(80, 16*18, share_data_from_0[S0_IMU_PITCH], 4 , 2);
    ips200_show_float(0, 16*19, share_data_from_0[S0_IMU_YAW], 4,2);
    // ips200_show_float(0, 16*19, cam_down.target_center_x, 4,2);
    // ips200_show_float(80, 16*19, cam_down.target_center_y, 4,2);
}