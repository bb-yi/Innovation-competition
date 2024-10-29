#ifndef __MECANUMMOTIONCONTROL_H
#define __MECANUMMOTIONCONTROL_H

#include "main.h"
#include "tool.h"
#include "tim.h"
#include "oled.h"
#include "filtering.h"
#include "cmsis_os.h"
#include "pid.h"
#define MOTOR_PWM_FREQ 1000 // PWM波频率

/*

电机A   TIM10_CH1 PB8
U8    TIM11_CH1 PB9
编码器  PA15 TIM2CH1
        PB3  TIM2CH2

电机B   TIM9_CH1 PE5
U9    TIM9_CH2 PE6
编码器  PB4 TIM3CH1
        PB5 TIM3CH2

电机C   TIM1_CH1 PE9
U10    TIM1_CH2 PE11
编码器  PB6 TIM4CH1
        PB7 TIM4CH2

电机D   TIM1_CH3 PE13
U12    TIM1_CH4 PE14
编码器  PA0 TIM5CH1
        PA1 TIM5CH2

一圈1560个计数
*/

/*
        前
    +---------+
    |         |
↖  |         |   ↗
轮B |         |   轮C
    |         |
    |         |
    +---------+
    |         |
轮A |         |   轮D
↗  |         |   ↖
    |         |
    +---------+
        后
*/
/*
运动解算
https://blog.csdn.net/liuerin/article/details/104175981?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_utm_term~default-5-104175981-blog-120140997.235^v43^control&spm=1001.2101.3001.4242.4&utm_relevant_index=7
*/
#define MOTORA 0
#define MOTORB 1
#define MOTORC 2
#define MOTORD 3

#define Encoder_Frequency 520

#define MOTOR_A_IN2_TIM htim10
#define MOTOR_A_IN1_TIM htim11
#define MOTOR_A_IN2_CH TIM_CHANNEL_1
#define MOTOR_A_IN1_CH TIM_CHANNEL_1
#define MOTOR_A_ENCODER_TIM htim2
#define MOTOR_A_ENCODER_1 TIM_CHANNEL_1
#define MOTOR_A_ENCODER_2 TIM_CHANNEL_2

#define MOTOR_B_TIM htim9
#define MOTOR_B_IN2_CH TIM_CHANNEL_2
#define MOTOR_B_IN1_CH TIM_CHANNEL_1
#define MOTOR_B_ENCODER_TIM htim3
#define MOTOR_B_ENCODER_1 TIM_CHANNEL_1
#define MOTOR_B_ENCODER_2 TIM_CHANNEL_2

#define MOTOR_C_TIM htim1
#define MOTOR_C_IN2_CH TIM_CHANNEL_1
#define MOTOR_C_IN1_CH TIM_CHANNEL_2
#define MOTOR_C_ENCODER_TIM htim4
#define MOTOR_C_ENCODER_1 TIM_CHANNEL_1
#define MOTOR_C_ENCODER_2 TIM_CHANNEL_2

#define MOTOR_D_TIM htim1
#define MOTOR_D_IN2_CH TIM_CHANNEL_3
#define MOTOR_D_IN1_CH TIM_CHANNEL_4
#define MOTOR_D_ENCODER_TIM htim5
#define MOTOR_D_ENCODER_1 TIM_CHANNEL_1
#define MOTOR_D_ENCODER_2 TIM_CHANNEL_2

void motor_init(void);
void motor_stop_all(void);
void set_motor_Voltage(uint8_t motor_id, float speed);
void base_control(float x_speed, float y_speed, float rot_speed);
void PID_Init(void);
void encoders_read(void);
void calibrateDistanceToZero(void);
void set_motor_speed(uint8_t motor_id, float target_speed);
uint8_t base_rotation_control_world(float target_angle, float speed);
void base_run_distance(float distance, float speed);
void base_Horizontal_run_distance(float distance, float speed);
void base_run_distance2(float distance, float speed);
void base_Horizontal_run_distance2(float distance, float speed);
void base_rotation_world(float angle, float speed);

void base_run_angle(float angle, float speed);

void motor_test(void);
typedef struct
{
        uint32_t last_read_time;
        int motor_A_encoder_val;
        int motor_B_encoder_val;
        int motor_C_encoder_val;
        int motor_D_encoder_val;
        int last_motor_A_encoder_val;
        int last_motor_B_encoder_val;
        int last_motor_C_encoder_val;
        int last_motor_D_encoder_val;
        float motor_A_speed;
        float motor_B_speed;
        float motor_C_speed;
        float motor_D_speed;
        float last_motor_A_speed;
        float last_motor_B_speed;
        float last_motor_C_speed;
        float last_motor_D_speed;
        float wheel_A_distance;
        float wheel_B_distance;
        float wheel_C_distance;
        float wheel_D_distance;
} MOTOR_Encoder;

#endif