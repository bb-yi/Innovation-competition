#include "servo.h"
#include "tim.h"
#include "tool.h"
#include "SliderElevatorControl.h"

/*
20000个计数归零
周期50hz 一个周期 20ms
0.5ms~2.5ms
占空比：2.5%~12.5%
计数个数 500~2500
*/

void Set_Servo_angle(uint8_t servo, uint16_t angle)
{

    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET) // 电机使能开关
    {
        if (servo == 0)
        {
            uint16_t count = int_Map(angle, 0, 270, 500, 2500);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, count);
        }
        else if (servo == 1)
        {
            uint16_t count = int_Map(angle, 0, 270, 500, 2500);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, count);
        }
        else if (servo == 2)
        {
            uint16_t count = int_Map(angle, 0, 180, 500, 2500);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, count);
        }
    }
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    Set_Servo_angle(0, 54);
    Set_Servo_angle(1, 15);
    Set_Servo_angle(2, 30);
}
void Set_Sliding_table_Pos(uint8_t pos)
{
    switch (pos)
    {
    case 0:
        Set_Servo_angle(0, 51);
        break;
    case 1:
        Set_Servo_angle(0, 222);
        break;
    default:
        break;
    }
}

void Set_Table_Pos(uint8_t pos)
{
    switch (pos)
    {
    case 0:
        Set_Servo_angle(1, 20);
        break;
    case 1:
        Set_Servo_angle(1, 138);
        break;
    case 2:
        Set_Servo_angle(1, 258);
        break;
    default:
        break;
    }
}

void Catch_material(void)
{
    Set_Servo_angle(catch, 133);
}

void Release_material(void)
{
    Set_Servo_angle(catch, 30);
}
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    Set_Sliding_table_Pos(0); // 滑台舵机
    Set_Table_Pos(0);         // 物料盘舵机
    Catch_material();         // 爪子舵机
}
void Get_material(uint8_t pos)
{
    Release_material();
    Set_Sliding_table_Pos(0);
    osDelay(500);
    set_Slider_position(85, 7);
    osDelay(1000);
    Catch_material();
    osDelay(500);
    set_Slider_position(145, 7);
    Set_Table_Pos(pos);
    Set_Sliding_table_Pos(1);
    osDelay(1500);
    set_Slider_position(100, 7);
    osDelay(800);
    Release_material();
    osDelay(500);
    set_Slider_position(145, 7);
    osDelay(200);
    Set_Sliding_table_Pos(0);
}

void Get_material_floor(uint8_t pos)
{
    Release_material();
    Set_Sliding_table_Pos(0);
    osDelay(500);
    set_Slider_position(0, 7);
    osDelay(1000);
    Catch_material();
    osDelay(500);
    set_Slider_position(145, 7);
    Set_Table_Pos(pos);
    Set_Sliding_table_Pos(1);
    osDelay(1500);
    set_Slider_position(100, 7);
    osDelay(800);
    Release_material();
    osDelay(500);
    set_Slider_position(145, 7);
    osDelay(200);
    Set_Sliding_table_Pos(0);
}

void Put_material(uint8_t pos)
{
    set_Slider_position(150, 7);
    osDelay(1000);
    Set_Table_Pos(pos);
    Set_Sliding_table_Pos(1);
    osDelay(800);
    set_Slider_position(95, 7);
    osDelay(500);
    Catch_material();
    osDelay(500);
    set_Slider_position(150, 7);
    osDelay(800);
    Set_Sliding_table_Pos(0);
    osDelay(800);
    set_Slider_position(5, 7);
    osDelay(800);
    Release_material();
    osDelay(1000);
    set_Slider_position(150, 7);
    osDelay(1500);
}