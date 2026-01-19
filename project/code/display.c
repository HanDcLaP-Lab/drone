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
    ips200_show_int(40,16*4 , cam_down.centers[0][0], 4);
    ips200_show_string(120,16*4,"x:");
    ips200_show_int(160,16*4 , cam_down.centers[0][1], 4);
    
    ips200_show_string(0,16*5,"t_p:");
    ips200_show_float(40,16*5 , flight_target.target_pitch, 3,2);
    ips200_show_string(120,16*5,"t_r:");
    ips200_show_float(160,16*5 , flight_target.target_roll, 3,2);

    //imu数据
    ips200_show_string(0,16*7,"Ro:");
    ips200_show_float(40,16*7 , imu_data.roll, 3,2);
    ips200_show_string(120,16*7,"Pi:");
    ips200_show_float(160,16*7 , imu_data.pitch, 3,2);
    ips200_show_string(0,16*8,"Ya:");
    ips200_show_float(40,16*8 , imu_data.yaw, 3,2);
    ips200_show_string(120,16*8,"z:");
    ips200_show_float(160,16*8 , imu_data.z, 3,2);
    
    // ips200_show_string(120,16*10,"yaw:");
    // ips200_show_float(160,16*10 , imu_data.yaw, 3,2);
    // ips200_show_string(0,16*9,"GR:");
    // ips200_show_int(40,16*9 , (int)imu_data.groll, 4);
    // ips200_show_string(120,16*9,"GP:");
    // ips200_show_int(160,16*9 , (int)imu_data.gpitch, 4);
    // ips200_show_string(0,16*10,"GY:");
    // ips200_show_int(40,16*10 , (int)imu_data.gyaw, 4);
    
}
