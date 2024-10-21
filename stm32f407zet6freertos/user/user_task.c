#include "user_task.h"

#include "cmsis_os.h"
#include "oled.h"
#include "adc_data.h"
#include "motor.h"
#include "mpu.h"
#include "stepper_a4988.h"
#include "openmv.h"
#include "servo.h"
#include "pid.h"

extern SYS_STATE_Data sys_state_data; // 陀螺仪数据
extern MOTOR_Encoder motor_encoder;   // 电机编码器数据
extern OPENMV_data openmv_data;       // 摄像头OpenMV数据
extern uint8_t MPU_RX_flag;           // 陀螺仪接收标志位
extern uint8_t openmv_rx_flag;        // OpenMV接收标志位
extern uint8_t Camera_now_mode;       // 摄像头当前模式

void Display_flag(uint8_t x, uint8_t y, uint8_t flag)
{
    if (flag)
    {
        OLED_ShowNumber(x, y, 8, 1, 12);
    }
    else
    {
        OLED_ShowChar(x, y, ' ', 12, 1);
    }
}

float battery_voltage = 0.0;
uint8_t run_time = 0;
/**
 * @brief OLED显示任务
 *
 */
void OLED_display_task(void)
{
    battery_voltage = get_Average_Battery_Voltage(10);
    OLED_ShowNumber(0, 0, run_time, 2, 12);

    // OLED_ShowString(42, 0, "T2:");
    // OLED_ShowNumber(62, 0, MPU_get_start_time(), 2, 12);
    Display_flag(18, 0, MPU_RX_flag);
    Display_flag(24, 0, openmv_rx_flag);
    OLED_ShowString(36, 0, "T:");
    OLED_ShowNumber(48, 0, openmv_data.object_list[0], 3, 12);
    OLED_ShowNumber(72, 0, openmv_data.object_list[1], 3, 12);
    switch (Camera_now_mode)
    {
    case QR_MODE:
        OLED_ShowString(96, 0, "E");
        break;
    case CENTER_POSITION_MODE:
        OLED_ShowString(96, 0, "Y");
        break;
    case HIGH_CENTER_POSITION_MODE:
        OLED_ShowString(96, 0, "C");
        break;
    case FIND_LINE_MODE:
        OLED_ShowString(96, 0, "L");
        break;
    default:
        break;
    }
    switch (openmv_data.last_identify_color)
    {
    case 1:
        OLED_ShowString(96, 12, "R");
        break;
    case 2:
        OLED_ShowString(96, 12, "G");
        break;
    case 3:
        OLED_ShowString(96, 12, "B");
        break;
    default:
        break;
    }
    OLED_ShowNumber(108, 12, openmv_data.hsa_circle, 1, 12);
    OLED_ShowString(00, 12, "V:");
    OLED_ShowFloatNum(18, 12, battery_voltage, 2, 2, 12);
    OLED_ShowFloatNum(60, 12, openmv_data.object_position_x, 1, 2, 12);
    OLED_ShowFloatNum(60, 24, openmv_data.object_position_y, 1, 2, 12);
    OLED_ShowFloatNum(0, 24, openmv_data.line_distance, 1, 2, 12);
    OLED_ShowNumber(0, 36, openmv_data.line_angle, 2, 12);
    if (Get_IMU_Is_Working())
    {
        OLED_ShowString(0, 48, "Y:");
        float yaw = radiansToDegrees(sys_state_data.FullYaw) >= 180 ? radiansToDegrees(sys_state_data.FullYaw) - 360 : radiansToDegrees(sys_state_data.FullYaw);
        OLED_ShowFloatNum(18, 48, yaw, 3, 1, 12);
    }
    else
    {
        MPU_Init();
    }
    sys_state_data.last_Microseconds = sys_state_data.Microseconds;
    OLED_Refresh_Gram();
    run_time++;
    if (run_time >= 100)
    {
        run_time = 0;
    }
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
}
// 二维码区域任务
void QrCode_Task(void)
{
    Camera_SendString("e");
}

pid find_circle_pid;
pid find_circle_yaw_pid;

void find_circle(uint8_t mode)
{
    // x 0.5 y 0.42
    float PositionThreshold;
    PositionThreshold = (mode == 0 ? 0.1f : 0.03f);
    float clamp_value = 5.0f;
    find_circle_pid.Kp = 15.0f;
    find_circle_pid.Ki = 0.001f;
    find_circle_pid.Kd = 0.0f;
    // find_circle_pid.Kp = 55.0f;
    // find_circle_pid.Ki = 0.05f;
    // find_circle_pid.Kd = 10.0f;
    find_circle_yaw_pid.Kp = 60.0f;
    find_circle_yaw_pid.Ki = 0.02f;
    find_circle_yaw_pid.Kd = 0.0f;

    float error_x, error_y, output_x, output_y;
    float start_yaw = Get_IMU_Yaw();
    float error_yaw, output_yaw;
    float target_x = 0.53f;
    float target_y = 0.50f;
    osDelay(200);
    for (;;)
    {
        error_x = (target_x - openmv_data.object_position_y);
        error_y = -(target_y - openmv_data.object_position_x);
        error_yaw = start_yaw - Get_IMU_Yaw();
        output_x = PID_Control(&find_circle_pid, error_x);
        output_y = PID_Control(&find_circle_pid, error_y);
        output_yaw = PID_Control(&find_circle_yaw_pid, error_yaw);
        // find_circle_pid.Ki = (error_x < 0.1f) && (error_y < 0.1f) ? 0.05f : 0.07f;
        // printf("error_x=%.2f,error_y=%.2f,output_x=%.2f,output_y=%.2f\r\n", error_x, error_y, output_x, output_y);
        // printf("error_yaw=%.2f,output_yaw=%.2f\r\n", error_yaw, output_yaw);
        output_x = clamp(output_x, -clamp_value, clamp_value);
        output_y = clamp(output_y, -clamp_value, clamp_value);
        if (openmv_data.hsa_circle == 1)
        {
            base_control(output_x, output_y, output_yaw);
            if (Abs(error_x) < PositionThreshold * 0.8f && Abs(error_y) < PositionThreshold)
            {
                for (uint16_t i = 0; i < 800; i++)
                {
                    base_control(0, 0, 0);
                    osDelay(1);
                }
                error_x = (0.53f - openmv_data.object_position_y);
                error_y = -(0.50f - openmv_data.object_position_x);
                if (Abs(error_x) < PositionThreshold && Abs(error_y) < PositionThreshold)
                {
                    for (uint16_t i = 0; i < 500; i++)
                    {

                        base_control(0, 0, 0);
                        osDelay(1);
                    }
                    find_circle_pid.integral = 0.0f;
                    motor_stop_all();
                    osDelay(1);
                    break;
                }
            }
        }
        else
        {
            base_control(0, 0, output_yaw);
        }

        osDelay(1);
    }
}

void find_line_calibrate_MPU(void)
{
    Camera_switch_mode(QR_MODE);
    osDelay(100);
    Camera_switch_mode(FIND_LINE_MODE);
    osDelay(2000);
    for (uint16_t i = 0; i < 3; i++)
    {

        float line_now_angle = Get_find_line_angle_avg(10);
        base_run_angle(line_now_angle * 0.8f, 1);
        calibrateAngleToZero();
    }
}

pid fine_line_PID;
void find_line_calibrate_MPU_PID(void)
{
    fine_line_PID.Kp = 10.0f;
    fine_line_PID.Ki = 0.0f;
    fine_line_PID.Kd = 0.0f;
    Camera_switch_mode(QR_MODE);
    osDelay(100);
    Camera_switch_mode(FIND_LINE_MODE);
    float error_angle, output;
    for (;;)
    {
        error_angle = Get_find_line_angle_avg(2);
        output = PID_Control(&fine_line_PID, error_angle);
        output = clamp(output, -5, 5);
        base_run_angle(0, 0, output);
        if (error_angle < 1.0f)
        {
            for (uint16_t i = 0; i < 800; i++)
            {
                base_control(0, 0, 0);
                osDelay(1);
            }
            motor_stop_all();
            break;
        }
    }
}

// 原料区任务

void MaterialArea_Task(void)
{
    openmv_data.object_list[0] = 231;
    openmv_data.object_list[1] = 132;
    Camera_switch_mode(CENTER_POSITION_MODE);
    set_Slider_position(150, 7);
    find_circle(0);
    printf("finish\r\n");
    for (;;)
    {
        if (openmv_data.identify_color == extract_digit(openmv_data.object_list[0], 3))
        {
            break;
        }
    }
    osDelay(1000);
    Get_material(openmv_data.last_identify_color - 1);
    for (;;)
    {
        if (openmv_data.identify_color == extract_digit(openmv_data.object_list[0], 2))
        {
            break;
        }
    }
    osDelay(1000);
    Get_material(openmv_data.last_identify_color - 1);
    for (;;)
    {
        if (openmv_data.identify_color == extract_digit(openmv_data.object_list[0], 1))
        {
            break;
        }
    }
    osDelay(1000);
    Get_material(openmv_data.last_identify_color - 1);
}
// 粗加工区任务
void RoughProcessingArea_Task(void)
{
    openmv_data.object_list[0] = 231;
    openmv_data.object_list[1] = 132;
    float move_distance;
    float sum_distance = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        switch (extract_digit(openmv_data.object_list[0], 3 - i))
        {
        case 1:
            move_distance = -10;
            break;
        case 2:
            move_distance = 0;
            break;
        case 3:
            move_distance = 10;
            break;
        default:
            break;
        }
        base_run_distance(move_distance + sum_distance, 2);
        sum_distance -= move_distance;
        Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
        find_circle(1);
        printf("finish\r\n");
        osDelay(1000);
        Put_material(openmv_data.last_identify_color - 1);
        osDelay(1000);
    }
    base_run_distance(sum_distance, 2);
}

void TemporaryStorageArea_Task(void)
{
}

extern uint8_t Slider_is_OK;
float run_speed = 1;
float rot_speed = 1;

uint8_t main_task(void)
{
    osDelay(2000);
    Camera_switch_mode(FIND_LINE_MODE);

    // Slider_position_init();
    // osDelay(1000);
    // set_Slider_position(150, 7);

    base_Horizontal_run_distance(15, run_speed); // 移出启停区
    osDelay(200);
    base_run_distance(64, run_speed); // 去往二维码区域
    osDelay(200);
    // QrCode_Task();

    osDelay(1000);
    base_run_distance(85, run_speed); // 去往原料区
    osDelay(1000);

    // MaterialArea_Task(); // 原料区任务
    osDelay(1000);

    base_run_distance(-40, run_speed); // 去往粗加工区
    osDelay(200);
    base_run_angle(-90, rot_speed);
    osDelay(200);
    base_run_distance(175, run_speed);
    osDelay(200);
    base_run_angle(-90, rot_speed);

    // RoughProcessingArea_Task(); // 粗加工区任务
    osDelay(1000);

    base_run_distance(-88, run_speed);
    osDelay(200);
    base_run_angle(90, rot_speed);
    osDelay(200);
    base_run_distance(-85, run_speed);

    // TemporaryStorageArea_Task(); // 暂存区任务
    osDelay(1000);

    base_run_distance(-92, run_speed); // 去往原料区2
    osDelay(200);
    base_run_angle(90, rot_speed);
    osDelay(200);
    base_run_distance(-40, run_speed);

    // MaterialArea_Task(); // 原料区任务2
    osDelay(1000);

    base_run_distance(-40, run_speed); // 去往粗加工区
    osDelay(200);
    base_run_angle(-90, rot_speed);
    osDelay(200);
    base_run_distance(175, run_speed);
    osDelay(200);
    base_run_angle(-90, rot_speed);

    // RoughProcessingArea_Task(); // 粗加工区任务
    osDelay(1000);

    base_run_distance(-80, run_speed);
    osDelay(200);
    base_run_angle(90, rot_speed);
    osDelay(200);
    base_run_distance(-85, run_speed);

    // TemporaryStorageArea_Task(); // 暂存区任务
    osDelay(1000);

    base_run_distance(-90, run_speed); // 去往原料区2
    osDelay(200);
    base_run_angle(90, rot_speed);
    osDelay(200);
    base_run_distance(-210, run_speed);

    // MaterialArea_Task(); // 原料区任务2
    osDelay(1000);

    // base_run_angle(90, run_speed);
    base_Horizontal_run_distance(-15, run_speed); // 移进启停区
    return 1;
}