#include "SliderElevatorControl.h"
#include "tool.h"
#include "tim.h"
#include "cmsis_os.h"
#include "ZDT_Stepper.h"
extern ZDTStepperData stepperdata_5;
float last_position = 0;
void set_Slider_position(float position, float speed)
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
    uint32_t start_time = HAL_GetTick();
    float max_time = ((d_position) / 50);
    for (;;)
    {
        ZDT_Stepper_Read_motor_status_flags(5);
        osDelay(10);
        uint32_t current_time = HAL_GetTick();
        if (stepperdata_5.motor_position_reached == 1)
        {
            printf("电机到位\r\n");
            break;
        }
        if (current_time - start_time > (uint32_t)(max_time * 1000))
        {
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