#include "MecanumMotionControl.h"
#include "mpu.h"
#include "ZDT_Stepper.h"
#include "beep.h"
#include "pid.h"
#include "tool.h"
#include "openmv.h"
uint16_t accel_accel_max = 400; // 加速度 单位RPM/s
float max_speed_f = 120.0f;     // 最大速度 单位RPM
extern uint8_t command_success_flag;
extern uint8_t get_stepper_data_flag;
uint8_t print_flag = 0;

extern ZDTStepperData stepperdata_1;
extern ZDTStepperData stepperdata_2;
extern ZDTStepperData stepperdata_3;
extern ZDTStepperData stepperdata_4;
/*
控制小车运动的文件，包括电机控制、编码器读取、PID控制等。
*/
void State_print(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // 使用 vprintf 实现可变参数
    if (print_flag == 1)
    {
        printf(format, args);
    }
    va_end(args);
}
void stepper_stop(uint8_t id, uint8_t sync_flag)
{
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_stop(id, sync_flag); // 立即停止
        osDelay(10);
        State_print("电机停止,等待返回,%d\n", id);
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
}

void Stepper_start_sync_motion_with_check(uint8_t id)
{
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_start_sync_motion(id); // 开启多机同步运动
        osDelay(10);
        State_print("多机同步,等待返回,%d\n", id);
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
}
/**
 * @brief 停止所有电机
 *
 */
void motor_stop_all(void)
{
    stepper_stop(1, SYNC_ENABLE); // 立即停止
    stepper_stop(2, SYNC_ENABLE); // 立即停止
    stepper_stop(3, SYNC_ENABLE); // 立即停止
    stepper_stop(4, SYNC_ENABLE); // 立即停止
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_start_sync_motion(0); // 开启多机同步运动
        osDelay(5);
        State_print("多机同步,等待返回\n");
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
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
    for (;;)
    {
        ZDT_Stepper_Set_Speed(motor_id, dir, speed_rate, Abs(target_speed), sync_flag);
        osDelay(3);
        State_print("控制速度,等待返回,%d\n", motor_id);
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
    // ZDT_Stepper_Set_Speed(motor_id, dir, speed_rate, Abs(target_speed), sync_flag);
}

void Set_Stepper_run_T_angle(uint8_t motor_id, uint16_t accel_accel, float max_speed_f, float angle, uint8_t position_mode, uint8_t sync_flag)
{
    if (motor_id == 3 || motor_id == 4)
    {
        angle = -angle;
    }
    uint8_t dir = angle > 0 ? 0 : 1;
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_Set_T_position(motor_id, dir, accel_accel, accel_accel, max_speed_f, Abs(angle), position_mode, sync_flag);
        osDelay(10);
        State_print("T形运动,等待返回,%d\n", motor_id);
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
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

    Set_Stepper_run_T_angle(1, accel_accel[0], speed[0], angles[0], REL_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(2, accel_accel[1], speed[1], angles[1], REL_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(3, accel_accel[2], speed[2], angles[2], REL_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(4, accel_accel[3], speed[3], angles[3], REL_POS_MODE, SYNC_ENABLE);
    Stepper_start_sync_motion_with_check(0); // 开启多机同步运动
}
void Set_all_stepper_speed(float *speeds, uint16_t accel_accel)
{
    set_Stepper_speed(1, accel_accel, speeds[0], SYNC_ENABLE);
    set_Stepper_speed(2, accel_accel, speeds[1], SYNC_ENABLE);
    set_Stepper_speed(3, accel_accel, speeds[2], SYNC_ENABLE);
    set_Stepper_speed(4, accel_accel, speeds[3], SYNC_ENABLE);
    Stepper_start_sync_motion_with_check(0); // 开启多机同步运动
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
    State_print("A:%d, B:%d, C:%d, D:%d\r\n", stepperdata_1.motor_position_reached, stepperdata_2.motor_position_reached, stepperdata_3.motor_position_reached, stepperdata_4.motor_position_reached);
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
    x_speed = -x_speed;
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
 * @brief 小车底盘控制 直接写入运行的距离 未进行校准 阻塞函数
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
    float max_time = angle == 0 ? (Abs(distance_x) + Abs(distance_y)) / 2.0f / speed : 1.2f;
    printf("max_time:%f\n", max_time);
    for (;;)
    {
        osDelay(1);
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
void Stepper_Read_current_position(uint8_t motor_id)
{
    get_stepper_data_flag = 0;
    for (;;)
    {
        ZDT_Stepper_Read_current_position(motor_id);
        osDelay(5);
        State_print("读电机位置,等待返回,%d\n", motor_id);
        if (get_stepper_data_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
}
void read_all_stepper_position(void)
{
    Stepper_Read_current_position(1);
    Stepper_Read_current_position(2);
    Stepper_Read_current_position(3);
    Stepper_Read_current_position(4);
}
void Set_Stepper_T_pos(uint8_t motor_id, uint16_t accel_accel, float max_speed_f, float angle, uint8_t sync_flag)
{
    if (motor_id == 3 || motor_id == 4)
    {
        angle = -angle;
    }
    uint8_t dir = angle > 0 ? 0 : 1;
    command_success_flag = 0;
    for (;;)
    {
        ZDT_Stepper_Set_T_position(motor_id, dir, accel_accel, accel_accel, max_speed_f, Abs(angle), ABS_POS_MODE, sync_flag);
        osDelay(10);
        State_print("等待返回,%d\n", motor_id);
        if (command_success_flag == 1)
        {
            State_print("成功\n");
            break;
        }
    }
}
// 函数定义
float findMinimum(float a, float b, float c)
{
    float min = a; // 假设a是最小的
    if (b < min)
    {
        min = b; // 如果b小于当前的最小值，则更新最小值
    }
    if (c < min)
    {
        min = c; // 如果c小于当前的最小值，则更新最小值
    }
    return min; // 返回最小值
}

float distance_to_angle(float distance)
{
    return radiansToDegrees(distance / WHEEL_RADIUS * 10);
}

float smooth_speed(float alpha, float run_distance, float speed, float accel)
{
    // float accel = 10.0f;
    float accel_speed_1, accel_speed_2, target_speed;
    run_distance = distance_to_angle(run_distance);
    accel_speed_1 = fmax(sqrt(2 * accel * run_distance * (1 - alpha)), 0);
    accel_speed_2 = fmax(sqrt(2 * accel * run_distance * alpha), 0);
    target_speed = findMinimum(accel_speed_1, accel_speed_2, speed);
    // printf("accel_speed_1=%.2f,accel_speed_2=%.2f,target_speed=%.2f\n", accel_speed_1, accel_speed_2, target_speed);
    target_speed = clamp(target_speed, 1, speed);
    return target_speed;
}
// 函数定义
float float_mix(float a, float b, float alpha)
{
    // 确保 alpha 在 0 和 1 之间
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    // 计算混合值
    return (1.0f - alpha) * a + alpha * b;
}
pid distance_rotation_pid;
pid offset_pid;
pid find_line_angle_pid;
extern float acc_speed;
void base_run_distance_base_fix(float distance_x, float distance_y, float speed, float mix_alpha, float line_distance)
{
    uint8_t run_mode;
    int8_t dir;
    float max_error = 0.01f;
    run_mode = distance_x == 0 ? 0 : (distance_y == 0 ? 1 : 2);
    float run_distance = Abs(run_mode == 0 ? distance_y : distance_x);
    set_beep_short_flag();
    pid_base_init(&distance_rotation_pid);
    pid_base_init(&offset_pid);
    pid_base_init(&find_line_angle_pid);

    distance_rotation_pid.Kp = 0.8f; // 0.2f
    distance_rotation_pid.Ki = 0.0f;
    distance_rotation_pid.Kd = 1.0f;
    offset_pid.Kp = 1.0f; // 0.2f
    offset_pid.Ki = 0.0f;
    offset_pid.Kd = 0.0f;
    find_line_angle_pid.Kp = 0.8f; // 0.2f
    find_line_angle_pid.Ki = 0.0f;
    find_line_angle_pid.Kd = 0.0f;
    float now_yaw, error_yaw, yaw_output;
    float start_angle_1, start_angle_2, start_angle_3, start_angle_4;
    float target_angle_1, target_angle_2, target_angle_3, target_angle_4;
    float error_angle_1, error_angle_2, error_angle_3, error_angle_4;
    float alpha, control_speed = 0;

    float error_offset, offset_output, find_line_error_angle, find_line_error_angle_output;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    // float start_yaw = target_angle;
    read_all_stepper_position();
    start_angle_1 = stepperdata_1.current_position;
    start_angle_2 = stepperdata_2.current_position;
    start_angle_3 = -stepperdata_3.current_position;
    start_angle_4 = -stepperdata_4.current_position;
    if (run_mode == 0)
    {
        dir = distance_y > 0 ? 1 : -1;
        target_angle_1 = start_angle_1 + distance_to_angle(distance_y);
        target_angle_2 = start_angle_2 + distance_to_angle(distance_y);
        target_angle_3 = start_angle_3 + distance_to_angle(distance_y);
        target_angle_4 = start_angle_4 + distance_to_angle(distance_y);
    }
    else if (run_mode == 1)
    {
        dir = distance_x > 0 ? 1 : -1;
        target_angle_1 = start_angle_1 + distance_to_angle(distance_x);
        target_angle_2 = start_angle_2 - distance_to_angle(distance_x);
        target_angle_3 = start_angle_3 + distance_to_angle(distance_x);
        target_angle_4 = start_angle_4 - distance_to_angle(distance_x);
    }
    else
    {
        return;
    }
    printf("d_angle_1:%.2f,d_angle_2:%.2f,d_angle_3:%.2f,d_angle_4:%.2f\n", radiansToDegrees(distance_y / WHEEL_RADIUS * 10), radiansToDegrees(distance_x / WHEEL_RADIUS * 10), radiansToDegrees(distance_y / WHEEL_RADIUS * 10), radiansToDegrees(distance_x / WHEEL_RADIUS * 10));
    for (;;)
    {
        error_offset = line_distance - Get_find_line_distance();
        offset_output = PID_Control(&offset_pid, error_offset, 20);
        offset_output = clamp(offset_output, -10, 10);
        find_line_error_angle = Get_find_line_angle();
        find_line_error_angle = clamp(find_line_error_angle, -5, 5);
        find_line_error_angle_output = -PID_Control(&find_line_angle_pid, find_line_error_angle, 20);
        // find_line_error_angle_output = clamp(find_line_error_angle_output, -3, 3);

        // printf("error_offset:%.2f,offset_output:%.2f\n", error_offset, offset_output);
        read_all_stepper_position();
        now_yaw = radiansToDegrees(Get_IMU_Yaw());
        error_yaw = start_yaw - now_yaw;
        yaw_output = -PID_Control(&distance_rotation_pid, error_yaw, 20);
        error_angle_1 = target_angle_1 - stepperdata_1.current_position;
        error_angle_2 = target_angle_2 - stepperdata_2.current_position;
        error_angle_3 = target_angle_3 - (-stepperdata_3.current_position);
        error_angle_4 = target_angle_4 - (-stepperdata_4.current_position);
        if (run_mode == 1)
        {
            error_angle_2 = -error_angle_2;
            error_angle_4 = -error_angle_4;
        }
        alpha = (float)dir * float_Map(error_angle_1 + error_angle_2 + error_angle_3 + error_angle_4, -4 * Abs(radiansToDegrees(run_distance / WHEEL_RADIUS * 10)), 4 * Abs(radiansToDegrees(run_distance / WHEEL_RADIUS * 10)), -1.0f, 1.0f);
        yaw_output = clamp(yaw_output, -5, 5);
        yaw_output = yaw_output * (control_speed / speed);
        control_speed = smooth_speed(alpha, run_distance, speed, acc_speed);
        // printf("alpha=%.2f,distance=%.2f,control_speed=%.2f,error_yaw=%.2f,yaw_output=%.2f\n", alpha, run_distance, control_speed, error_yaw, yaw_output);
        if (run_mode == 0)
        {
            base_speed_control(offset_output, dir * control_speed, float_mix(yaw_output, find_line_error_angle_output, mix_alpha), 800);
        }
        else if (run_mode == 1)
        {
            base_speed_control(dir * control_speed, 0, yaw_output, 800);
        }
        // yaw_output *(control_speed / speed);
        if (alpha < max_error)
        {
            motor_stop_all();
            printf("alpha=%f,error_distance:%f,start_yaw%.2f,end_yaw%.2f\n", alpha, alpha * run_distance, start_yaw, radiansToDegrees(Get_IMU_Yaw()));
            set_beep_short_flag();
            printf("到达目标位置\n");
            break;
        }
        osDelay(1);
    }
}

void Set_all_stepper_angle_ABS(float *angles, float *max_speed)
{
    Set_Stepper_run_T_angle(1, 100, max_speed[0], angles[0], ABS_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(2, 100, max_speed[1], angles[1], ABS_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(3, 100, max_speed[2], angles[2], ABS_POS_MODE, SYNC_ENABLE);
    Set_Stepper_run_T_angle(4, 100, max_speed[3], angles[3], ABS_POS_MODE, SYNC_ENABLE);
    Stepper_start_sync_motion_with_check(0); // 开启多机同步运动
}
void base_run_distance_fix(float distance, float speed, float mix_alpha, float line_distance)
{
    base_run_distance_base_fix(0, distance, speed, mix_alpha, line_distance);
}
void base_Horizontal_run_distance_fix(float distance, float speed)
{
    base_run_distance_base_fix(distance, 0, speed, 0, 0);
}

/**
 * @brief 小车旋转指定角度——开环
 *  阻塞型函数
 * @param angle 角度 逆时针为负，顺时针为正
 * @param speed
 */
float base_run_angle_open_loop(float angle, float speed)
{
    angle = -angle;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    base_run_distance_base(0, 0, angle, speed);
    float end_yaw = radiansToDegrees(Get_IMU_Yaw());
    float error_yaw = (start_yaw - angle) - end_yaw;
    printf("start_yaw:%f,end_yaw:%f,error_yaw:%f\n", start_yaw, end_yaw, error_yaw);
    return error_yaw;
}

void base_rotation_open_loop(float angle, float speed)
{
    float max_error = 0.05f;
    float now_error, now_yaw;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    float target_angle = start_yaw + angle;
    uint32_t start_time = HAL_GetTick();
    uint8_t first_run_flag = 0;
    for (;;)
    {
        now_yaw = radiansToDegrees(Get_IMU_Yaw());
        now_error = target_angle - now_yaw;
        printf("now_angle=%f,target_angle=%f,error_angle=%f\n", now_yaw, target_angle, now_error);
        base_run_angle_open_loop(now_error, (first_run_flag == 1 ? 30 : speed));
        first_run_flag = 1;
        if (Abs(now_error) < max_error)
        {
            motor_stop_all();
            printf("到达目标角度,error_angle:%f\n", now_error);
            break;
        }
        float current_time = HAL_GetTick();
        if (current_time - start_time > (uint32_t)(5000))
        {
            set_beep_long_flag();
            motor_stop_all();
            printf("机身运动超时!,error_angle:%f\n", now_error);
            break;
        }
        osDelay(1);
    }
}
void base_rotation_open_loop_world(float angle, float speed)
{
    // float max_error = 0.05f;
    float now_error, now_yaw;
    float target_angle = angle;
    uint32_t start_time = HAL_GetTick();
    now_yaw = radiansToDegrees(Get_IMU_Yaw());
    now_error = target_angle - now_yaw;
    base_run_angle_open_loop(now_error, 60);

    // for (;;)
    // {
    //     now_yaw = radiansToDegrees(Get_IMU_Yaw());
    //     now_error = target_angle - now_yaw;
    //     printf("now_angle=%f,target_angle=%f,error_angle=%f\n", now_yaw, target_angle, now_error);
    //     base_run_angle_open_loop(now_error, 60);
    //     if (Abs(now_error) < max_error)
    //     {
    //         motor_stop_all();
    //         printf("到达目标角度,error_angle:%f\n", now_error);
    //         break;
    //     }
    //     float current_time = HAL_GetTick();
    //     if (current_time - start_time > (uint32_t)(4000))
    //     {
    //         set_beep_long_flag();
    //         motor_stop_all();
    //         printf("机身运动超时!,error_angle:%f\n", now_error);
    //         break;
    //     }
    //     osDelay(1);
    // }
}

pid rotation_pid;
void base_rotation_world_base(float angle, float speed)
{
    uint32_t start_time = HAL_GetTick();
    set_beep_short_flag();
    speed = speed * 0.01f;
    pid_base_init(&rotation_pid);
    rotation_pid.Kp = 3.0;  // 60.0f
    rotation_pid.Ki = 0.0f; // 0.18f
    rotation_pid.Kd = 3.0f; // 40.0f
    float clamp_speed = 25.0f;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    float target_angle = angle;
    float now_angle, error_angle, output;
    for (;;)
    {
        osDelay(1);
        uint32_t current_time = HAL_GetTick();
        now_angle = radiansToDegrees(Get_IMU_Yaw());
        error_angle = target_angle - now_angle;
        output = PID_Control(&rotation_pid, error_angle, 20);
        output = clamp(output * speed, -clamp_speed, clamp_speed);
        // printf("now_angle:%f,target_angle:%f,error_angle:%f,output:%f\n", now_angle, target_angle, error_angle, output);
        base_speed_control(0, 0, -output, 800);
        if (Abs(error_angle) < 0.02f)
        {

            motor_stop_all();
            if (Abs(target_angle - radiansToDegrees(Get_IMU_Yaw())) < 0.02f)
            {
                set_beep_short_flag();
                printf("到达目标角度,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
                break;
            }
        }
        if (current_time - start_time > (uint32_t)(Abs(angle - start_yaw) / 90.0f * 4000))
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
void base_run_angle_base(float angle, float speed)
{
    uint32_t start_time = HAL_GetTick();
    set_beep_short_flag();
    speed = speed * 0.01f;
    pid_base_init(&rotation_pid);
    rotation_pid.Kp = 1.0;  // 60.0f
    rotation_pid.Ki = 0.0f; // 0.18f
    rotation_pid.Kd = 7.0f; // 40.0f
    float clamp_speed = 50.0f;
    float start_yaw = radiansToDegrees(Get_IMU_Yaw());
    float target_angle = start_yaw + angle;
    float now_angle, error_angle, output;
    for (;;)
    {
        osDelay(1);
        uint32_t current_time = HAL_GetTick();
        now_angle = radiansToDegrees(Get_IMU_Yaw());
        error_angle = target_angle - now_angle;
        output = PID_Control(&rotation_pid, error_angle, 20);
        output = clamp(output * speed, -clamp_speed, clamp_speed);
        printf("now_angle=%f,target_angle=%f,error_angle=%f,output=%f\n", now_angle, target_angle, error_angle, output);
        base_speed_control(0, 0, -output, 800);
        if (Abs(error_angle) < 0.02f)
        {

            motor_stop_all();
            if (Abs(target_angle - radiansToDegrees(Get_IMU_Yaw())) < 0.02f)
            {
                set_beep_short_flag();
                printf("到达目标角度,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
                break;
            }
        }
        if (current_time - start_time > (uint32_t)(Abs(angle) / 90.0f * 3000))
        {
            set_beep_long_flag();
            motor_stop_all();
            printf("机身运动超时!,error_angle:%f\n", target_angle - radiansToDegrees(Get_IMU_Yaw()));
            break;
        }
    }
}
void base_rotation_world(float angle, float speed)
{
    base_rotation_open_loop_world(angle, speed);
}
void base_run_angle(float angle, float speed)
{
    base_run_angle_open_loop(angle, speed);
}
void base_run_distance_and_rotation(float distance_x, float distance_y, float angle, float time)
{
    float target_speed[2] = {distance_x / time, distance_y / time};
    float wheel_speeds[4];
    float rot_speed[2];
    uint32_t time_start = HAL_GetTick();

    for (;;)
    {
        osDelay(1);
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
}

void motor_rotation_test(void)
{
    // base_run_angle(90, 100);
    // osDelay(1000);
    // base_run_angle(-90, 100);
    // osDelay(1000);
    // base_run_angle(180, 100);
    // osDelay(1000);
    // base_run_angle(-180, 100);
    // osDelay(1000);
    base_rotation_open_loop(90, 180);
    osDelay(1000);
    base_rotation_open_loop(-90, 180);
    osDelay(1000);
    base_rotation_open_loop(180, 180);
    osDelay(1000);
    base_rotation_open_loop(-180, 180);
    osDelay(1000);
    // base_rotation_world(90, 100);
    // osDelay(1000);
    // base_rotation_world(0, 100);
    // osDelay(1000);
    // base_rotation_world(180, 100);
    // osDelay(1000);
    // base_rotation_world(0, 100);
    // osDelay(1000);
}
