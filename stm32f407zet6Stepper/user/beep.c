#include "beep.h"
#include "cmsis_os.h"

uint8_t beep_short_flag = 0;
uint8_t beep_long_flag = 0;
void beep_delay(uint32_t delay)
{
    // HAL_Delay(delay);
    osDelay(delay);
    // for (uint16_t i = 0; i < 1000; i++)
    // {
    // }
}
/**
 * @brief 使用标志位控制蜂鸣器的开关
 *
 */
void set_beep_short_flag(void)
{
    beep_short_flag = 1;
}

void set_beep_long_flag(void)
{
    beep_long_flag = 1;
}

void beep_short(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    beep_delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    beep_short_flag = 0;
}

void beep_long(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    beep_delay(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    beep_long_flag = 0;
}
