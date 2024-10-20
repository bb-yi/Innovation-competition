#include "stepper_a4988.h"
#include "tool.h"
#include "tim.h"
#include "cmsis_os.h"
/*
42步进电机步距角1.8
一圈200步 a4988最大16细分 200*16=3200脉冲一圈

一圈滑台40mm 一毫米40/360
*/

int stepper_count = 0;
int max_stepper_count;
uint8_t Slider_is_OK = 0;
uint8_t stepper_run_flag = 0; // 步进电机运行标志位 0 停止 1 运行
void stepper_pwm_Callback(void)
{
    stepper_count++;
    if (stepper_count >= max_stepper_count)
    {
        stepper_run_flag = 0;
        HAL_TIM_PWM_Stop_IT(&Stepper_timer, TIM_CHANNEL_1);
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

/**
 * @brief 步进电机使能函数
 *
 * @param enable 1 ENABLE 使能  0 DISABLE失能
 */
void stepper_enable(uint8_t enable)
{
    if (enable == ENABLE)
    {
        HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_RESET);
    }
    else if (enable == DISABLE)
    {
        HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief
 *
 * @param angle 转动角度 正为向上
 * @param speed 速度 单位转每秒
 * @param Holding_angle 保持力矩
 */
void stepper_rotation(float angle, float speed, uint8_t Holding_angle)
{
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET) // 电机使能开关
    {

        if (angle < 0)
        {
            HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
        }
        angle = Abs(angle);
        stepper_count = 0;
        max_stepper_count = (int)(angle * microsteps / 1.8f);
        stepper_enable(ENABLE);
        uint16_t prescaler = 83;
        stepper_run_flag = 1;
        float freq = speed * 200.0f * microsteps;
        float pwm_freq_arr = (float)CPU_MainFrequency / ((float)prescaler + 1.0f) / freq * 1.0f - 1.0f;
        __HAL_TIM_SET_PRESCALER(&Stepper_timer, prescaler);
        __HAL_TIM_SetAutoreload(&Stepper_timer, (uint16_t)pwm_freq_arr);
        __HAL_TIM_SetCompare(&Stepper_timer, TIM_CHANNEL_1, (uint16_t)(pwm_freq_arr / 2));
        HAL_TIM_PWM_Start_IT(&Stepper_timer, TIM_CHANNEL_1);
        for (;;)
        {
            if (stepper_run_flag == 0)
            {
                break;
            }
            osDelay(1);
        }
        if (Holding_angle == 0)
        {
            stepper_enable(DISABLE);
        }
    }
}

float last_position = 0; // 上一次的位置
void set_Slider_position(float position, float speed)
{
    position = clamp(position, 0.0f, 150.0f);
    // position = float_Map(position, 0.0f, 150.0f, 150.0f, 0.0f);
    float delta_position = position - last_position;
    // printf("delta_position=%f\r\n", delta_position);
    float angle = -delta_position * 360.0f / 40.0f;
    stepper_rotation(angle, speed, 1);
    last_position = position;
}

void Slider_position_init(void)
{
    if (Slider_is_OK == 0)
    {
        stepper_rotation(360.0f * 3.75f, 6, 1);
        for (;;)
        {
            if (Slider_is_OK == 1)
            {
                break;
            }
            osDelay(1);
        }
    }
}

void Slider_position_init_Callback(void)
{
    stepper_run_flag = 0;
    last_position = 0.0f;
    if (Slider_is_OK == 0)
    {
        HAL_TIM_PWM_Stop_IT(&Stepper_timer, TIM_CHANNEL_1);
        stepper_enable(0);
    }
    Slider_is_OK = 1;
    printf("Slider_position_init_OK\r\n");
}