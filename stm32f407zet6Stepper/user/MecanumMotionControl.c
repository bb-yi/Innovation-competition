#include "MecanumMotionControl.h"
#include "mpu.h"

/*
控制小车运动的文件，包括电机控制、编码器读取、PID控制等。
*/

/**
 * @brief 停止所有电机
 *
 */
void motor_stop_all(void)
{
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
void set_motor_speed(uint8_t motor_id, float target_speed)
{
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
}

void base_run_distance_base(float distance, float speed, uint8_t mode)
{
}
void base_run_distance(float distance, float speed)
{
    base_run_distance_base(distance, speed, 0);
}

// 小车水平运动指定距离
void base_Horizontal_run_distance(float distance, float speed)
{
    base_run_distance_base(distance, speed, 1);
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