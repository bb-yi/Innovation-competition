#include "SliderElevatorControl.h"
#include "tool.h"
#include "tim.h"
#include "cmsis_os.h"

/**
 * @brief
 *
 * @param angle 转动角度 正为向上
 * @param speed 速度 单位转每秒
 */
void stepper_rotation(float angle, float speed)
{
}

float last_position = 0; // 上一次的位置
void set_Slider_position(float position, float speed)
{
    position = clamp(position, 0.0f, 150.0f);
    float delta_position = position - last_position;
    float angle = -delta_position * 360.0f / 40.0f;
    stepper_rotation(angle, speed);
    last_position = position;
}

void Slider_position_init(void)
{
}