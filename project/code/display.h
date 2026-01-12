#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>

#define IPS200_TYPE     (IPS200_TYPE_SPI)       

void display_init();
void display_motor_output_display();
#endif