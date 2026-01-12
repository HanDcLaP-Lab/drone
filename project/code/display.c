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
    ips200_show_string(0,16*2,"lf:");
    ips200_show_int(30,16*2,motor_out.lf, 4);
    ips200_show_string(80,16*2,"rf:");
    ips200_show_int(110,16*2,motor_out.rf, 4);
    ips200_show_string(0,16*3,"lb:");
    ips200_show_int(30,16*3,motor_out.lb, 4);
    ips200_show_string(80,16*3,"rb:");
    ips200_show_int(110,16*3,motor_out.rb, 4);
}