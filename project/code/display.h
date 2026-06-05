#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>

#define IPS200_TYPE     (IPS200_TYPE_SPI)       

void display_init();
void display_motor_output_display(); 
void display_image_display(void);
void display_image_debug_display(float car_x, float car_y, uint8_t car_valid, float target_x, float target_y, uint8_t target_valid);
#endif