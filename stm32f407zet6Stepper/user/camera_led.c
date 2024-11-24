#include "camera_led.h"
#include "tim.h"
#include "tool.h"
#define LED_FREQ 5000
void camera_led_init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    // set_pwm_param(htim4, TIM_CHANNEL_4, 5000, 0);
    // 配置PSC预分频值
    __HAL_TIM_SET_PRESCALER(&htim4, 167);
    // 配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim4, (uint16_t)200);
    // 配置PWM占空比
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, (uint16_t)0);
}
void Set_Camera_Led_light(uint8_t level)
{
    level = clamp(level, 0, 100);
    uint16_t output = int_Map(level, 0, 100, 200, 0);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, (uint16_t)output);
}
void camera_led_huxideng(void)
{
    for (uint8_t i = 0; i < 200; i++)
    {

        Set_Camera_Led_light(Abs(i - 100));
        osDelay(10);
    }
}