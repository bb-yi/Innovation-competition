#include "SliderElevatorControl.h"
#include "tool.h"
#include "tim.h"
#include "cmsis_os.h"
#define Solid_TIM_Handle htim3

float last_position = 150;
uint32_t pulse_count;
uint32_t target_pulse_count;
uint8_t Finish_flag = 0;
void Silder_TIM_Callback(void)
{
    // 检查是否是更新事件（ARR 重载）
    if (__HAL_TIM_GET_FLAG(&Solid_TIM_Handle, TIM_FLAG_UPDATE) != RESET)
    {
        // 清除中断标志
        __HAL_TIM_CLEAR_IT(&Solid_TIM_Handle, TIM_IT_UPDATE);

        // 计数 PWM 脉冲周期
        pulse_count++;
        // printf("pulse_count:%d\n", pulse_count);
        if (pulse_count >= target_pulse_count)
        {
            Finish_flag = 1;
            pulse_count = 0;
            // Set_Pwm_duty(0);
            HAL_TIM_PWM_Stop_IT(&Solid_TIM_Handle, TIM_CHANNEL_1);
            printf("已发送目标脉冲%d\n", target_pulse_count);
        }
    }
}
void set_solid_enable(uint8_t enable)
{
    if (enable)
    {
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
void set_solid_dir(uint8_t dir) // 0向下 1向上
{
    if (dir)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
}

uint32_t arr;
void set_pwm_HZ(uint32_t hz)
{
    uint32_t pl = 1000000;
    arr = pl / hz;
    // printf("arr:%d\n", arr);
    __HAL_TIM_SetAutoreload(&Solid_TIM_Handle, arr - 1);
}

void Set_Pwm_duty(uint8_t duty)
{
    if (duty == 1)
    {
        __HAL_TIM_SetCompare(&Solid_TIM_Handle, TIM_CHANNEL_1, arr / 2);
    }
    else
    {
        __HAL_TIM_SetCompare(&Solid_TIM_Handle, TIM_CHANNEL_1, 0);
    }
}
// pos 单位 mm   speed 单位 mm/s 120合适
void set_Slider_position(float position, float speed)
{
    // position = 150 - position;
    // position = clamp(position, 0, 150);
    if (position == 150)
    {
        position = 152;
    }
    else if (position == 0)
    {
        position = -2;
    }
    Finish_flag = 0;
    float delta_position = position - last_position;
    if (delta_position > 0)
    {
        set_solid_dir(1); // 0向下 1向上
    }
    else
    {
        set_solid_dir(0); // 0向下 1向上
    }
    target_pulse_count = (uint32_t)(Abs(delta_position) * 20);
    set_pwm_HZ(speed * 20);
    printf("delta_position:%f,target_pulse_count:%d\n", delta_position, target_pulse_count);
    HAL_TIM_PWM_Start_IT(&Solid_TIM_Handle, TIM_CHANNEL_1);
    // Set_Pwm_duty(1);

    last_position = clamp(position, 0, 150);
    for (;;)
    {
        osDelay(1);
        if (Finish_flag == 1)
        {
            printf("目标位置%f\n", position);
            break;
        }
    }
}
void set_Slider_position_2(float position, float speed)
{
    position = clamp(position, 5, 145);

    Finish_flag = 0;
    float delta_position = position - last_position;
    if (delta_position > 0)
    {
        set_solid_dir(1); // 0向下 1向上
    }
    else
    {
        set_solid_dir(0); // 0向下 1向上
    }
    target_pulse_count = (uint32_t)(Abs(delta_position) * 20);
    set_pwm_HZ(speed * 20);
    printf("delta_position:%f,target_pulse_count:%d\n", delta_position, target_pulse_count);
    HAL_TIM_PWM_Start_IT(&Solid_TIM_Handle, TIM_CHANNEL_1);

    last_position = position;
}

void Slider_position_init(void)
{

    // set_solid_enable(1);
    osDelay(1000);
    // set_solid_dir(1);
    // 配置PSC预分频值
    __HAL_TIM_SET_PRESCALER(&Solid_TIM_Handle, 83);
    // 配置PWM频率 ARR
    set_pwm_HZ(2000);
    // __HAL_TIM_SetAutoreload(&htim3, 65535);
    // 配置PWM占空比
    // printf("arr/2:%d\n", arr / 2);
    // HAL_TIM_PWM_Start_IT(&Solid_TIM_Handle, TIM_CHANNEL_1);

    __HAL_TIM_SetCompare(&Solid_TIM_Handle, TIM_CHANNEL_1, arr / 2);
    // set_pwm_param(htim3, TIM_CHANNEL_1, 1000, 50);
}