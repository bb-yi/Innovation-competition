#include "MecanumMotionControl.h"
#include "mpu.h"
#include "ZDT_Stepper.h"
#include "beep.h"
#include "pid.h"
uint16_t accel_accel_max = 200; // 加速度 单位RPM/s
float max_speed_f = 120.0f;     // 最大速度 单位RPM
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

uint8_t check_motor_is_enable(void)
{
    ZDT_Stepper_Read_motor_status_flags(1);
    ZDT_Stepper_Read_motor_status_flags(2);
    ZDT_Stepper_Read_motor_status_flags(3);
    ZDT_Stepper_Read_motor_status_flags(4);
    // osDelay(3);
    if (stepperdata_1.motor_enabled == 1 && stepperdata_2.motor_enabled == 1 && stepperdata_3.motor_enabled == 1 && stepperdata_4.motor_enabled == 1)
    {
        return 1;
    }
    else
    {
        printf("Motor is not enable!,1:%d,2:%d,3:%d,4:%d\r\n", stepperdata_1.motor_enabled, stepperdata_2.motor_enabled, stepperdata_3.motor_enabled, stepperdata_4.motor_enabled);

        return 0;
    }
}

/**
 * @brief 初始化电机
 *
 */
void motor_init(void)
{
}

/**
 * @brief 设置电机速度
 *
 * @param motor_id
 * @param speed 单位，RPM
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
void Set_all_stepper_angle(float *angles, float max_speed)
{
    float speed[4];
    float accel_accel[4];
    float max_angle = find_max_angle(Abs(angles[0]), Abs(angles[1]), Abs(angles[2]), Abs(angles[3]));
    speed[0] = Abs(angles[0] * max_speed / max_angle);
    speed[1] = Abs(angles[1] * max_speed / max_angle);
    speed[2] = Abs(angles[2] * max_speed / max_angle);
    speed[3] = Abs(angles[3] * max_speed / max_angle);
    accel_accel[0] = Abs(angles[0] * accel_accel_max / max_angle);
    accel_accel[1] = Abs(angles[1] * accel_accel_max / max_angle);
    accel_accel[2] = Abs(angles[2] * accel_accel_max / max_angle);
    accel_accel[3] = Abs(angles[3] * accel_accel_max / max_angle);

    Set_Stepper_run_T_angle(1, accel_accel[0], speed[0], angles[0], SYNC_ENABLE);
    Set_Stepper_run_T_angle(2, accel_accel[1], speed[1], angles[1], SYNC_ENABLE);
    Set_Stepper_run_T_angle(3, accel_accel[2], speed[2], angles[2], SYNC_ENABLE);
    Set_Stepper_run_T_angle(4, accel_accel[3], speed[3], angles[3], SYNC_ENABLE);

    ZDT_Stepper_start_sync_motion(0); // 开启多机同步运动
}
void Set_all_stepper_speed(float *speeds, uint16_t accel_accel)
{
    set_Stepper_speed(1, accel_accel, speeds[0], SYNC_ENABLE);
    set_Stepper_speed(2, accel_accel, speeds[1], SYNC_ENABLE);
    set_Stepper_speed(3, accel_accel, speeds[2], SYNC_ENABLE);
    set_Stepper_speed(4, accel_accel, speeds[3], SYNC_ENABLE);
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

// 逆运动学解算
void MecanumWheelIK(float S_x, float S_y, float angle, float *wheel_angles)
{
    wheel_angles[0] = radiansToDegrees((-S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[1] = radiansToDegrees((S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[2] = radiansToDegrees((-S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[3] = radiansToDegrees((S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
}
// 逆运动学解算
void MecanumWheel_Speed_IK(float S_x, float S_y, float angle, float *wheel_angles)
{
    S_x = S_x * 10.0f / 5.729f; // 将函数的单位转化为cm/s
    S_y = S_y * 10.0f / 5.729f;
    angle = angle / 6.0f; // 将函数的单位转化为度/s
    wheel_angles[0] = radiansToDegrees((-S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[1] = radiansToDegrees((S_x + S_y - (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[2] = radiansToDegrees((-S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
    wheel_angles[3] = radiansToDegrees((S_x + S_y + (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) * degreesToRadians(angle)) / WHEEL_RADIUS);
}

// 正运动学解算
void MecanumWheelFK(float wheel_angles[4], float *S_x, float *S_y, float *angle)
{
    *angle = WHEEL_RADIUS * (wheel_angles[2] + wheel_angles[3] - wheel_angles[0] - wheel_angles[1]) / (HORIZONTAL_HALF_LENGTH + VERTICAL_HALF_LENGTH) / 4.0f;
    *S_x = WHEEL_RADIUS * (wheel_angles[1] + wheel_angles[3] - wheel_angles[0] - wheel_angles[2]) / 4.0f;
    *S_y = WHEEL_RADIUS * (wheel_angles[0] + wheel_angles[1] + wheel_angles[2] + wheel_angles[3]) / 4.0f;
}

/**
 * @brief 检查电机是否到达目标位置
 *
 * @return uint8_t 到达目标位置返回1，否则返回0
 */
uint8_t CheckMotorsAtTargetPosition(void)
{
    ZDT_Stepper_Read_motor_status_flags(1);
    ZDT_Stepper_Read_motor_status_flags(2);
    ZDT_Stepper_Read_motor_status_flags(3);
    ZDT_Stepper_Read_motor_status_flags(4);
    printf("A:%d, B:%d, C:%d, D:%d\r\n", stepperdata_1.motor_position_reached, stepperdata_2.motor_position_reached, stepperdata_3.motor_position_reached, stepperdata_4.motor_position_reached);
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
 * @brief 控制小车速度
 *s
 * @param x_speed x方向速度，单位，RPM
 * @param y_speed y方向速度，单位，RPM
 * @param rot_speed 旋转速度，1左右就可以 单位 度/秒
 */
void base_speed_control(float x_speed, float y_speed, float rot_speed, float accel_accel)
{
    float wheel_speeds[4];
    MecanumWheelIK(x_speed, y_speed, rot_speed, wheel_speeds);
    // printf("wheel_speeds:%f,%f,%f,%f,x_speed:%f,y_speed:%f,rot_speed:%f\r\n", wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3], x_speed, y_speed, rot_speed);
    Set_all_stepper_speed(wheel_speeds, accel_accel);
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

/**
 * @brief 小车底盘控制
 *
 * @param distance_x x 方向距离
 * @param distance_y y 方向距离
 * @param angle 旋转角度 逆时针为正，顺时针为负 单位 度
 * @param speed 速度 一般值 150
 */
void base_run_distance_base(float distance_x, float distance_y, float angle, float speed)
{
    set_beep_short_flag();
    distance_x = -distance_x * 10.0f; // 将函数的单位转化为cm
    distance_y = distance_y * 10.0f;
    float wheel_angles[4];
    MecanumWheelIK(distance_x, distance_y, angle, wheel_angles);
    Set_all_stepper_angle(wheel_angles, speed);
    uint32_t start_time = HAL_GetTick();
    float max_time = angle == 0 ? (Abs(distance_x) + Abs(distance_y)) / speed : 3;
    printf("max_time:%f\n", max_time);
    for (;;)
    {
        uint32_t current_time = HAL_GetTick();
        if (CheckMotorsAtTargetPosition() == 1)
        {
            set_beep_short_flag();
            printf("到达目标位置\n");
            break;
        }
        if (current_time - start_time > (uint32_t)(max_time * 1000))
        {
            set_beep_long_flag();
            printf("机身运动超时,max_time:%f\n", max_time);
            break;
        }
        osDelay(1);
    }
}
void base_run_distance(float distance, float speed)
{
    base_run_distance_base(0, distance, 0, speed);
}

// 小车水平运动指定距离
void base_Horizontal_run_distance(float distance, float speed)
{
    base_run_distance_base(distance, 0, 0, speed);
}

/**
 * @brief 小车旋转指定角度——开环
 *  阻塞型函数
 * @param angle 角度 逆时针为负，顺时针为正
 * @param speed
 */
void base_run_angle_open_loop(float angle, float speed)
{
    angle = -angle;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    base_run_distance_base(0, 0, angle, speed);
    float end_yaw = radiansToDegrees(Get_IMU_Yaw());
    float error_yaw = (start_yaw - angle) - end_yaw;
    printf("start_yaw:%f,end_yaw:%f,error_yaw:%f\n", start_yaw, end_yaw, error_yaw);
}

pid rotation_pid;
void base_rotation_world(float angle, float speed)
{
    uint32_t start_time = HAL_GetTick();

    set_beep_short_flag();
    speed = speed * 0.01f;
    pid_base_init(&rotation_pid);
    rotation_pid.Kp = 0.5f; // 60.0f
    rotation_pid.Ki = 0.0f; // 0.18f
    rotation_pid.Kd = 0.5f; // 40.0f
    float clamp_speed = 50.0f;
    float target_angle = angle;
    float now_angle, error_angle, output;
    for (;;)
    {
        uint32_t current_time = HAL_GetTick();

        now_angle = radiansToDegrees(Get_IMU_Yaw());
        error_angle = target_angle - now_angle;
        output = PID_Control(&rotation_pid, error_angle, 20);
        output = clamp(output * speed, -clamp_speed, clamp_speed);
        // printf("now_angle:%f,target_angle:%f,error_angle:%f,output:%f\n", now_angle, target_angle, error_angle, output);
        base_speed_control(0, 0, -output, 800);
        if (Abs(error_angle) < 0.02f)
        {
            set_beep_short_flag();
            motor_stop_all();
            printf("到达目标角度,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
            break;
        }
        if (current_time - start_time > (uint32_t)(3 * 1000))
        {
            set_beep_long_flag();
            motor_stop_all();
            printf("机身运动超时!,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
            break;
        }
    }
}

/**
 * @brief 小车旋转指定角度
 *  阻塞型函数
 * @param angle 角度 逆时针为负，顺时针为正
 * @param speed 参考值 100
 */
void base_run_angle(float angle, float speed)
{
    uint32_t start_time = HAL_GetTick();
    set_beep_short_flag();
    speed = speed * 0.01f;
    pid_base_init(&rotation_pid);
    rotation_pid.Kp = 0.5f; // 60.0f
    rotation_pid.Ki = 0.0f; // 0.18f
    rotation_pid.Kd = 0.5f; // 40.0f
    float clamp_speed = 50.0f;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    float target_angle = start_yaw + angle;
    float now_angle, error_angle, output;
    for (;;)
    {
        uint32_t current_time = HAL_GetTick();

        now_angle = radiansToDegrees(Get_IMU_Yaw());
        error_angle = target_angle - now_angle;
        output = PID_Control(&rotation_pid, error_angle, 20);
        output = clamp(output * speed, -clamp_speed, clamp_speed);
        // printf("now_angle:%f,target_angle:%f,error_angle:%f,output:%f\n", now_angle, target_angle, error_angle, output);
        base_speed_control(0, 0, -output, 800);
        if (Abs(error_angle) < 0.02f)
        {
            set_beep_short_flag();
            motor_stop_all();
            printf("到达目标角度,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
            break;
        }
        if (current_time - start_time > (uint32_t)(3 * 1000))
        {
            set_beep_long_flag();
            motor_stop_all();
            printf("机身运动超时!,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
            break;
        }
    }
}

void base_run_distance_and_rotation(float distance_x, float distance_y, float angle, float time)
{
    float target_speed[2] = {distance_x / time, distance_y / time};
    float wheel_speeds[4];
    float rot_speed[2];
    uint32_t time_start = HAL_GetTick();

    for (;;)
    {
        uint32_t time_now = HAL_GetTick();
        uint32_t delta_time = time_now - time_start;
        float alpha = delta_time / time / 1000.0f;
        rotate_vector(target_speed, -angle * alpha, rot_speed);
        MecanumWheel_Speed_IK(rot_speed[0], rot_speed[1], angle / time, wheel_speeds); // 角度6度每秒
        Set_all_stepper_speed(wheel_speeds, 1200);
        if (delta_time > time * 1000)
        {
            break;
        }
    }
}

/**
 * @brief 测试函数
 *
 */
void motor_test(void)
{
    // base_run_distance_and_rotation(0, 100, 360, 3);
    base_run_distance_base(15, 15, 0, 100);
    osDelay(1000);
    base_run_distance(30, 100);
    osDelay(1000);
    base_Horizontal_run_distance(-15, 100);
    osDelay(1000);
    base_Horizontal_run_distance(15, 100);
    osDelay(1000);
    base_run_distance(-30, 100);
    osDelay(1000);
    base_run_distance_base(-15, -15, 0, 100);
    osDelay(1000);
    base_run_angle(90, 100);
    osDelay(1000);
    base_run_angle(180, 100);
    osDelay(1000);
    base_run_angle(90, 100);
    osDelay(1000);
    base_rotation_world(90, 100);
    osDelay(1000);
    base_rotation_world(0, 100);
}

void motor_rotation_test(void)
{
    base_run_angle(90, 100);
    osDelay(1000);
    base_run_angle(-90, 100);
    osDelay(1000);
    base_run_angle(180, 100);
    osDelay(1000);
    base_run_angle(-180, 100);
    osDelay(1000);
    base_rotation_world(90, 80);
    osDelay(1000);
    base_rotation_world(0, 80);
    osDelay(1000);
    base_rotation_world(180, 80);
    osDelay(1000);
    base_rotation_world(0, 80);
    osDelay(1000);
}
