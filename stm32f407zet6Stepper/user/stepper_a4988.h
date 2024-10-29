#ifndef __STEPPER_A4988_H
#define __STEPPER_A4988_H

#include "main.h"

// enable 低电平工作
#define DIR_PIN GPIO_PIN_15 // 低电平顺时针转动,高电平逆时针转动
#define DIR_PORT GPIOD
#define ENABLE_PIN GPIO_PIN_5 // 使能信号
#define ENABLE_PORT GPIOC

#define Stepper_timer htim12

#define ENABLE 1
#define DISABLE 0

#define CW 0  // 顺时针
#define CCW 1 // 逆时针

#define microsteps 1.0f // 步进电机分辨率 000 1分频 100 2分频 010 4分频 110 8分频 111 16分频

#define Pwm_Out_Tim htim12

void stepper_pwm_Callback(void);
void stepper_enable(uint8_t enable);
void stepper_rotation(float angle, float speed, uint8_t Holding_angle);
void set_Slider_position(float position, float speed);
void Slider_position_init(void);
void Slider_position_init_Callback(void);
#endif
