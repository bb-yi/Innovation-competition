#include "adc_data.h"

float get_Battery_Voltage_value(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t ADC_Value = HAL_ADC_GetValue(&hadc1);
        float Voltage = (float)ADC_Value * 3.3f * 11.0f * 0.994359f / 4096.0f; // 矫正系数0.994359
        return Voltage;
    }
    else
    {
        return 0;
    }
}

float get_Average_Battery_Voltage(uint8_t time)
{
    float temp;
    for (uint8_t i = 0; i < time; i++)
    {
        temp += get_Battery_Voltage_value();
    }

    float average_Voltage = temp / (float)time;
    return average_Voltage;
}