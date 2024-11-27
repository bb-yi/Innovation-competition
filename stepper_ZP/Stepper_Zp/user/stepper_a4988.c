/*
 * @Author: bb-yi ledingqin@gmail.com
 * @Date: 2024-11-26 02:44:27
 * @LastEditors: bb-yi ledingqin@gmail.com
 * @LastEditTime: 2024-11-27 13:14:42
 * @FilePath: \MDK-ARMd:\DanPianJi\minicar\Innovation-competition\stepper_ZP\Stepper_Zp\user\stepper_a4988.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stepper_a4988.h"
#include "tim.h"
/*
42步进电机步距角1.8
一圈200步 a4988最大16细分 200*16=3200脉冲一圈
*/
#define STEPPER_TIM htim2
#define STEPPER_CH TIM_CHANNEL_1
#define STEPPER_MICROSTEP 16

uint16_t target_count, now_count;
void stepper_init(void)
{
    __HAL_TIM_SET_PRESCALER(&STEPPER_TIM, 71);
    __HAL_TIM_SET_AUTORELOAD(&STEPPER_TIM, 999);
    __HAL_TIM_SET_COMPARE(&STEPPER_TIM, STEPPER_CH, 499);
}

void stepper_Callback(void)
{
    now_count++;
    if (now_count >= target_count)
    {
        printf("目标脉冲:%d 已完成\r\n", target_count);
        HAL_TIM_PWM_Stop(&STEPPER_TIM, STEPPER_CH);
        HAL_TIM_PWM_Stop_IT(&STEPPER_TIM, STEPPER_CH);
        now_count = 0;
    }
}

void stepper_set_pwm_Frequency(uint32_t Frequency)
{
    uint16_t arr = 1000000 / Frequency; // 分频后的频率
    __HAL_TIM_SET_PRESCALER(&STEPPER_TIM, 71);
    __HAL_TIM_SET_AUTORELOAD(&STEPPER_TIM, (uint16_t)(arr - 1));
    __HAL_TIM_SET_COMPARE(&STEPPER_TIM, STEPPER_CH, (uint16_t)(arr / 2 - 1));
}
/**
 * @brief 旋转步进电机 3200脉冲/圈
 *
 * @param speed 步进电机速度 单位：度每秒 最大 360*6.2
 * @param Angle
 */
void Set_Stepper_Speed(float speed)
{
    stepper_set_pwm_Frequency((uint16_t)(200 * STEPPER_MICROSTEP / 360 * speed));
}
void stepperRotateAngle(float speed, float Angle)
{
    target_count = (uint16_t)(Angle * 200 * STEPPER_MICROSTEP / 360);
    now_count = 0;
    Set_Stepper_Speed(speed);
    HAL_TIM_PWM_Start(&STEPPER_TIM, STEPPER_CH);
    HAL_TIM_PWM_Start_IT(&STEPPER_TIM, STEPPER_CH);
    printf("开始发送脉冲脉冲:%d\r\n", target_count);
}
