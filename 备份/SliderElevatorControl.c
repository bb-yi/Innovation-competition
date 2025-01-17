﻿#include "SliderElevatorControl.h"
#include "tool.h"
#include "tim.h"
#include "cmsis_os.h"
#include "ZDT_Stepper.h"
#include "beep.h"
extern ZDTStepperData stepperdata_5;
float last_position = 0;
extern uint8_t command_success_flag;
void set_Slider_position_with_chack(uint8_t id, uint8_t dir, uint16_t accel_accel, uint16_t decel_accel, float max_speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
{
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_release_stall_protection(5);
        ZDT_Stepper_Set_T_position(id, dir, accel_accel, decel_accel, max_speed_f, position_angle_f, position_mode, sync_flag);
        osDelay(10);
        printf("T形运动,等待返回,%d\n", id);
        if (command_success_flag == 1)
        {
            printf("成功\n");
            break;
        }
    }
}

void set_Slider_position(float position, float speed)
{
    position = clamp(position, 4, 145);
    float d_position = position - last_position;
    uint8_t dir;
    dir = (d_position > 0) ? 1 : 0;
    d_position = Abs(d_position);
    float angle = d_position * 36.0f / 4.0f;
    printf("滑块位置：%f,角度：%f\r\n", position, angle);
    set_beep_short_flag();
    set_Slider_position_with_chack(5, dir, 1500, 1500, speed, angle, REL_POS_MODE, SYNC_DISABLE);
    last_position = position;
    uint32_t start_time = HAL_GetTick();
    float max_time = ((d_position) / 100.0f);
    for (;;)
    {
        ZDT_Stepper_Read_motor_status_flags(5);
        osDelay(1);
        uint32_t current_time = HAL_GetTick();
        if (stepperdata_5.motor_position_reached == 1)
        {
            set_beep_short_flag();

            printf("电机到位\r\n");
            break;
        }
        if (current_time - start_time > (uint32_t)(2 * 1000))
        {
            set_beep_long_flag();
            printf("设置滑台位置超时,max_time:%f\n", max_time);
            break;
        }
    }
}
void set_Slider_position_2(float position, float speed)
{
    position = clamp(position, 4, 145);
    float d_position = position - last_position;
    uint8_t dir;
    dir = (d_position > 0) ? 1 : 0;
    d_position = Abs(d_position);
    float angle = d_position * 36.0f / 4.0f;
    printf("滑块位置：%f,角度：%f\r\n", position, angle);
    ZDT_Stepper_Set_T_position(5, dir, 1500, 1500, speed, angle, REL_POS_MODE, SYNC_DISABLE);
    last_position = position;
}
void Slider_position_init(void)
{
    ZDT_Stepper_trigger_zero_return(5, 2, SYNC_DISABLE);
    uint32_t start_time = HAL_GetTick();

    for (;;)
    {
        ZDT_Stepper_Read_Zero_Status_flags(5);
        uint32_t current_time = HAL_GetTick();

        if (stepperdata_5.motor_zeroing_in_progress == 0)
        {
            printf("结束回零\r\n");
            last_position = 150;
            break;
        }
        if (current_time - start_time > 5000)
        {
            printf("回零超时\n");
            break;
        }
        osDelay(10);
    }
}