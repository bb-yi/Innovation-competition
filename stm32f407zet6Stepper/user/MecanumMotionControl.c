#include "MecanumMotionControl.h"
#include "mpu.h"
#include "ZDT_Stepper.h"
uint16_t accel_accel = 240; // 加速度 单位RPM/s
float max_speed_f = 300.0f; // 最大速度 单位RPM
extern ZDTStepperData stepperdata_1;
extern ZDTStepperData stepperdata_2;
extern ZDTStepperData stepperdata_3;
extern ZDTStepperData stepperdata_4;
/*
控制小车运动的文件，包括电机控制、编码器读取、PID控制等。
*/

/**
 * @brief 停止所有电机
 *
 */
void motor_stop_all(void)
{
    ZDT_Stepper_stop(1, SYNC_ENABLE); // 立即停止
    ZDT_Stepper_stop(2, SYNC_ENABLE); // 立即停止
    ZDT_Stepper_stop(3, SYNC_ENABLE); // 立即停止
    ZDT_Stepper_stop(4, SYNC_ENABLE); // 立即停止
    ZDT_Stepper_start_sync_motion(0); // 开启多机同步运动
}

/**
 * @brief 初始化电机和编码器
 *
 */
void motor_init(void)
{
}

void calibrateDistanceToZero(void)
{
}

/**
 * @brief 设置电机速度
 *
 * @param motor_id
 * @param speed 单位，转/s
 */
void set_Stepper_speed(uint8_t motor_id, uint16_t speed_rate, float target_speed, uint8_t sync_flag)
{
    if (motor_id == 3 || motor_id == 4)
    {
        target_speed = -target_speed;
    }
    uint8_t dir = target_speed > 0 ? 0 : 1;
    ZDT_Stepper_Set_Speed(motor_id, dir, speed_rate, Abs(target_speed), sync_flag);
}

void Set_Stepper_run_T_angle(uint8_t motor_id, uint16_t accel_accel, float max_speed_f, float angle, uint8_t sync_flag)
{
    if (motor_id == 3 || motor_id == 4)
    {
        angle = -angle;
    }
    uint8_t dir = angle > 0 ? 0 : 1;
    ZDT_Stepper_Set_T_position(motor_id, dir, accel_accel, accel_accel, max_speed_f, Abs(angle), REL_POS_MODE, sync_flag);
}

float find_max_angle(float angle_A, float angle_B, float angle_C, float angle_D)
{
    float max_angle = angle_A;
    if (angle_B > max_angle)
    {
        max_angle = angle_B;
    }
    if (angle_C > max_angle)
    {
        max_angle = angle_C;
    }
    if (angle_D > max_angle)
    {
        max_angle = angle_D;
    }
    return max_angle;
}
void Set_all_stepper_angle(float angle_A, float angle_B, float angle_C, float angle_D, float max_speed)
{
    float speed_A, speed_B, speed_C, speed_D;
    float accel_accel_A, accel_accel_B, accel_accel_C, accel_accel_D;
    float max_angle = find_max_angle(Abs(angle_A), Abs(angle_B), Abs(angle_C), Abs(angle_D));
    speed_A = Abs(angle_A * max_speed_f / max_angle);
    speed_B = Abs(angle_B * max_speed_f / max_angle);
    speed_C = Abs(angle_C * max_speed_f / max_angle);
    speed_D = Abs(angle_D * max_speed_f / max_angle);
    accel_accel_A = Abs(angle_A * accel_accel / max_angle);
    accel_accel_B = Abs(angle_B * accel_accel / max_angle);
    accel_accel_C = Abs(angle_C * accel_accel / max_angle);
    accel_accel_D = Abs(angle_D * accel_accel / max_angle);

    Set_Stepper_run_T_angle(1, accel_accel_A, speed_A, angle_A, SYNC_ENABLE);
    Set_Stepper_run_T_angle(2, accel_accel_B, speed_B, angle_B, SYNC_ENABLE);
    Set_Stepper_run_T_angle(3, accel_accel_C, speed_C, angle_C, SYNC_ENABLE);
    Set_Stepper_run_T_angle(4, accel_accel_D, speed_D, angle_D, SYNC_ENABLE);
    ZDT_Stepper_start_sync_motion(0); // 开启多机同步运动
}

/**
 * @brief PID控制句柄初始化
 *
 * @param p 控制pid的句柄
 */
void pid_base_init(pid *p)
{
    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->err = 0.0f;
    p->err_last = 0.0f;
    p->Kp = 1.0f; // 默认比例系数
    p->Ki = 0.0f; // 默认积分系数
    p->Kd = 0.0f; // 默认微分系数
    p->integral = 0.0f;
    p->Output = 0.0f;
}

void MecanumWheelIK(float S_x, float S_y, float angle, float *wheel_A_angle, float *wheel_B_angle, float *wheel_C_angle, float *wheel_D_angle)
{
    *wheel_A_angle = radiansToDegrees((-S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    *wheel_B_angle = radiansToDegrees((S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    *wheel_C_angle = radiansToDegrees((-S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    *wheel_D_angle = radiansToDegrees((S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
}

uint8_t CheckMotorsAtTargetPosition(void)
{
    ZDT_Stepper_Read_motor_status_flags(1);
    ZDT_Stepper_Read_motor_status_flags(2);
    ZDT_Stepper_Read_motor_status_flags(3);
    ZDT_Stepper_Read_motor_status_flags(4);
    printf("1:%d, 2:%d, 3:%d, 4:%d\r\n", stepperdata_1.motor_position_reached, stepperdata_2.motor_position_reached, stepperdata_3.motor_position_reached, stepperdata_4.motor_position_reached);
    if (stepperdata_1.motor_position_reached == 1 && stepperdata_2.motor_position_reached == 1 && stepperdata_3.motor_position_reached == 1 && stepperdata_4.motor_position_reached == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 小车底盘运动
 *s
 * @param x_speed x方向速度，单位，cm/s
 * @param y_speed y方向速度，单位，cm/s
 * @param rot_speed 旋转速度，1左右就可以
 */
void base_control(float x_speed, float y_speed, float rot_speed)
{
}

/**
 * @brief 世界坐标的小车旋转角度PID控制
 *
 * @param target_angle  目标角度
 */

uint8_t base_rotation_control_world(float target_angle, float speed)
{
    return 0;
}

void base_run_distance_base(float distance_x, float distance_y, float angle, float speed, uint8_t mode)
{
    float wheel_A_angle, wheel_B_angle, wheel_C_angle, wheel_D_angle;
    MecanumWheelIK(distance_x, distance_y, angle, &wheel_A_angle, &wheel_B_angle, &wheel_C_angle, &wheel_D_angle);
    Set_all_stepper_angle(wheel_A_angle, wheel_B_angle, wheel_C_angle, wheel_D_angle, speed);
    for (;;)
    {

        if (CheckMotorsAtTargetPosition() == 1)
        {
            printf("到达目标位置\n");
            break;
        }
        osDelay(1);
    }
}
void base_run_distance(float distance, float speed)
{
    // base_run_distance_base(distance, speed, 0);
}

// 小车水平运动指定距离
void base_Horizontal_run_distance(float distance, float speed)
{
    // base_run_distance_base(distance, speed, 1);
}

void base_rotation_world(float angle, float speed)
{
}

/**
 * @brief 小车旋转指定角度
 *  阻塞型函数
 * @param angle 角度 顺时针为正，单位 度
 * @param speed
 */
void base_run_angle(float angle, float speed)
{
}

/**
 * @brief 测试函数
 *
 */
void motor_test(void)
{

    // for (int i = 0; i < 4; i++)
    // {
    //     for (uint8_t j = 0; j < 200; j++)
    //     {
    //         float now_sp = j < 100 ? j : j - 200;
    //         encoders_read();
    //         set_motor_Voltage(i, now_sp);
    //         HAL_Delay(50);
    //     }
    //     OLED_ShowNumber(0, 12, i, 1, 12);
    //     OLED_Refresh_Gram();
    //     HAL_Delay(1000);
    //     motor_stop_all();
    //     HAL_Delay(1000);
    // }
    base_control(70, 0, 0);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
    base_control(-70, 0, 0);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
    base_control(0, 70, 0);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
    base_control(0, -70, 0);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
    base_control(0, 0, 600);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
    base_control(0, 0, -600);
    osDelay(500);
    motor_stop_all();
    osDelay(2000);
}