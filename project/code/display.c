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
    ips200_show_int(30,16*2,motor_out.lf, 4);
    ips200_show_string(80,16*2,"rf:");
    ips200_show_int(110,16*2,motor_out.rf, 4);
    ips200_show_string(0,16*3,"lb:");
    ips200_show_int(30,16*3,motor_out.lb, 4);
    ips200_show_string(80,16*3,"rb:");
    ips200_show_int(110,16*3,motor_out.rb, 4);
    
    //相机参数
    ips200_show_string(0,16*4,"x:");
    ips200_show_int(30,16*4 , cam_down.centers[0][0], 4);
    ips200_show_string(80,16*4,"y:");
    ips200_show_int(110,16*4 , cam_down.centers[0][1], 4);
    
    ips200_show_string(0,16*5,"t_p:");
    ips200_show_int(30,16*5 , (int)flight_target.target_pitch, 4);
    ips200_show_string(80,16*5,"t_r:");
    ips200_show_int(110,16*5 , (int)flight_target.target_roll, 4);
    
    //imu数据
    ips200_show_string(0,16*7,"Ro:");
    ips200_show_int(30,16*7 , (int)imu_data.roll, 4);
    ips200_show_string(80,16*7,"Pi:");
    ips200_show_int(110,16*7 , (int)imu_data.pitch, 4);
    ips200_show_string(0,16*8,"Ya:");
    ips200_show_int(30,16*8 , (int)imu_data.yaw, 4);
    ips200_show_string(80,16*8,"z:");
    ips200_show_int(110,16*8 , (int)imu_data.z, 4);

    ips200_show_string(0,16*9,"GR:");
    ips200_show_int(30,16*9 , (int)imu_data.groll, 4);
    ips200_show_string(80,16*9,"GP:");
    ips200_show_int(110,16*9 , (int)imu_data.gpitch, 4);
    ips200_show_string(0,16*10,"GY:");
    ips200_show_int(30,16*10 , (int)imu_data.gyaw, 4);
    
}/*
printf("IMU: R:%5.1f P:%5.1f Y:%5.1f | Ax:%4.2f Ay:%4.2f | H:%5.1f\r\n", 
           imu_data.roll, 
           imu_data.pitch, 
           imu_data.yaw,
           imu_data.world_ax, // 重点观察：向前推是否为正
           imu_data.world_ay, // 重点观察：向右推是否为正
           imu_data.z);       // 重点观察：上抬是否增加*/