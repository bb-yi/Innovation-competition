#ifndef __STEPPER_A4988_H
#define __STEPPER_A4988_H

#include "main.h"

/*
TIM1CH1主动pwm发射
DIE------>PA15
setp----->PE9

*/

// enable 低电平工作
#define DIR_PIN GPIO_PIN_14 // 低电平顺时针转动,高电平逆时针转动
#define DIR_PORT GPIOC

#define CW 0  // 顺时针
#define CCW 1 // 逆时针

void stepper_init(void);
void stepper_Callback(void);
void stepper_set_pwm_Frequency(uint32_t Frequency);
void stepperRotateAngle(float speed, float Angle);

#endif
