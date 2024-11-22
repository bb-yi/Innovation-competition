#include "camera_led.h"
#include "tim.h"
#include "tool.h"
#define LED_FREQ 5000
void camera_led_init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    set_pwm_param(htim4, TIM_CHANNEL_4, 5000, 0);
}
void Set_Camera_Led_light(uint8_t level)
{
    level = clamp(level, 0, 100);
    level = level - 100;
    set_pwm_param(htim4, TIM_CHANNEL_4, 5000, level);
}
void camera_led_huxideng(void)
{
    for (uint8_t i = 0; i < 200; i++)
    {

        Set_Camera_Led_light(Abs(i - 100));
        osDelay(10);
    }
}