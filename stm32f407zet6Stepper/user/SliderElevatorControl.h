#ifndef __SLIDERELEVATORCONTROL_H
#define __SLIDERELEVATORCONTROL_H

#include "main.h"

void Silder_TIM_Callback(void);
void set_solid_enable(uint8_t enable);
void set_Slider_position(float position, float speed);
void set_Slider_position_2(float position, float speed);
void Slider_position_init(void);
#endif
