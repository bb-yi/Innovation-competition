#ifndef __MECANUMMOTIONCONTROL_H
#define __MECANUMMOTIONCONTROL_H

#include "main.h"
#include "tool.h"
#include "tim.h"
#include "oled.h"
#include "filtering.h"
#include "cmsis_os.h"
#include "pid.h"

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
void motor_init(void);
void motor_stop_all(void);
void base_control(float x_speed, float y_speed, float rot_speed);
void calibrateDistanceToZero(void);
void set_motor_speed(uint8_t motor_id, float target_speed);
uint8_t base_rotation_control_world(float target_angle, float speed);
void base_run_distance(float distance, float speed);
void base_Horizontal_run_distance(float distance, float speed);
void base_rotation_world(float angle, float speed);
void base_run_angle(float angle, float speed);
void motor_test(void);

#endif