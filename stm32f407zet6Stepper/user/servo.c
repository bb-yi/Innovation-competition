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

void Set_Sliding_table_Pos(uint8_t pos)
{
    switch (pos)
    {
    case 0:
        Set_Servo_angle(0, 53);
        break;
    case 1:
        Set_Servo_angle(0, 214);
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
        Set_Servo_angle(1, 135 - 5);
        break;
    case 1:
        Set_Servo_angle(1, 255 - 5);
        break;
    case 2:
        Set_Servo_angle(1, 15 - 5);
        break;
    case 3:
        Set_Servo_angle(1, 90);
        break;
    default:
        break;
    }
}

void Catch_material(void)
{
    Set_Servo_angle(catch, 133);
    printf("闭合爪子\n");
}

void Release_material(void)
{
    Set_Servo_angle(catch, 40);
    printf("张开爪子\n");
}
void catch_material_in_middle(void)
{
    Set_Servo_angle(catch, 116);
    printf("爪子中间态\n");
}
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    // Set_Sliding_table_Pos(1); // 滑台舵机
    if(0)
    {   
    Set_Servo_angle(0, 225); // 滑台1收起来
    }
    else
    {
    Set_Sliding_table_Pos(0); // 滑台舵机 0 在外面 1收起来
    }
    Set_Table_Pos(3);
    // Set_Table_Pos(0);   // 物料盘舵机
    Release_material(); // 爪子舵机
}
float solider_speed = 800.0f;

void Get_material(uint8_t pos)
{
    Release_material();
    Set_Sliding_table_Pos(0);
    Set_Table_Pos(pos);
    osDelay(500);
    set_Slider_position(100, solider_speed);
    osDelay(100);
    Catch_material();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    Set_Sliding_table_Pos(1);
    osDelay(1000);
    set_Slider_position(100, solider_speed);
    osDelay(200);
    catch_material_in_middle();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    osDelay(200);
    Set_Sliding_table_Pos(0);
    Release_material();
}

void Get_material_floor(uint8_t pos)
{
    Release_material();
    Set_Sliding_table_Pos(0);
    Set_Table_Pos(pos);

    osDelay(500);
    set_Slider_position(5, solider_speed);
    osDelay(100);
    Catch_material();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    Set_Sliding_table_Pos(1);
    osDelay(1000);
    set_Slider_position(95, solider_speed);
    osDelay(200);
    catch_material_in_middle();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    osDelay(200);
    Set_Sliding_table_Pos(0);
    Release_material();
}

void Put_material(uint8_t pos)
{

    set_Slider_position(150, solider_speed);
    Set_Table_Pos(pos);
    catch_material_in_middle();
    Set_Sliding_table_Pos(1);
    osDelay(800);
    set_Slider_position(90, solider_speed);
    osDelay(100);
    Catch_material();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    osDelay(100);
    Set_Sliding_table_Pos(0);
    osDelay(500);
    set_Slider_position(10, solider_speed - 200);
    osDelay(200);
    Release_material();
    osDelay(400);
    set_Slider_position(150, solider_speed);
    osDelay(100);
}
void Put_material_in_obj(uint8_t pos)
{
    set_Slider_position(150, solider_speed);
    osDelay(100);
    Set_Table_Pos(pos);
    catch_material_in_middle();
    Set_Sliding_table_Pos(1);
    osDelay(800);
    set_Slider_position(98, solider_speed);
    osDelay(100);
    Catch_material();
    osDelay(500);
    set_Slider_position(150, solider_speed);
    osDelay(100);
    Set_Sliding_table_Pos(0);
    osDelay(500);
    set_Slider_position(80, solider_speed - 300);
    osDelay(200);
    Release_material();
    osDelay(400);
    set_Slider_position(150, solider_speed);
    osDelay(100);
}