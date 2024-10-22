#include "motor.h"
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
    set_motor_Voltage(0, 0.0f);
    set_motor_Voltage(1, 0.0f);
    set_motor_Voltage(2, 0.0f);
    set_motor_Voltage(3, 0.0f);
}
/**
 * @brief 设置电机电压
 *
 * @param motor_id 电机编号，0~3 — —> A,B,C,D
 * @param speed 范围为-100~100
 */
void set_motor_Voltage(uint8_t motor_id, float speed)
{
    speed = -speed; // 反向
    float pwm_in_1 = 0.0f;
    float pwm_in_2 = 0.0f;
    speed = clamp(speed, -100.0f, 100.0f);
    if (speed >= 0)
    {
        pwm_in_1 = float_Map(speed, 0.0f, 100.0f, 0.0f, 20.0f);
        pwm_in_2 = 0;
    }
    else if (speed < 0)
    {
        pwm_in_1 = 0;
        pwm_in_2 = float_Map(speed, -100.0f, 0.0f, 20.0f, 0.0f);
    }
    // printf("motor_id:%d, pwm_in_1:%f, pwm_in_2:%f\r\n", motor_id, pwm_in_1, pwm_in_2);
    switch (motor_id)
    {
    case 0:
        set_pwm_param(MOTOR_A_IN1_TIM, MOTOR_A_IN1_CH, MOTOR_PWM_FREQ, pwm_in_1);
        set_pwm_param(MOTOR_A_IN2_TIM, MOTOR_A_IN2_CH, MOTOR_PWM_FREQ, pwm_in_2);
        break;
    case 1:
        set_pwm_param(MOTOR_B_TIM, MOTOR_B_IN1_CH, MOTOR_PWM_FREQ, pwm_in_1);
        set_pwm_param(MOTOR_B_TIM, MOTOR_B_IN2_CH, MOTOR_PWM_FREQ, pwm_in_2);
        break;

    case 2:
        set_pwm_param(MOTOR_C_TIM, MOTOR_C_IN1_CH, MOTOR_PWM_FREQ, pwm_in_1);
        set_pwm_param(MOTOR_C_TIM, MOTOR_C_IN2_CH, MOTOR_PWM_FREQ, pwm_in_2);
        break;

    case 3:
        set_pwm_param(MOTOR_D_TIM, MOTOR_D_IN1_CH, MOTOR_PWM_FREQ, pwm_in_2);
        set_pwm_param(MOTOR_D_TIM, MOTOR_D_IN2_CH, MOTOR_PWM_FREQ, pwm_in_1);
        break;
    default:
        break;
    }
}

/**
 * @brief 编码器初始化
 *
 */
void encoders_init(void)
{
    HAL_TIM_Encoder_Start_IT(&MOTOR_A_ENCODER_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&MOTOR_B_ENCODER_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&MOTOR_C_ENCODER_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&MOTOR_D_ENCODER_TIM, TIM_CHANNEL_ALL);
}

/**
 * @brief 初始化电机和编码器
 *
 */
void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&MOTOR_A_IN1_TIM, MOTOR_A_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_B_TIM, MOTOR_B_IN2_CH);
    HAL_TIM_PWM_Start(&MOTOR_B_TIM, MOTOR_B_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_C_TIM, MOTOR_C_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_C_TIM, MOTOR_C_IN2_CH);
    HAL_TIM_PWM_Start(&MOTOR_D_TIM, MOTOR_D_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_D_TIM, MOTOR_D_IN2_CH);
    encoders_init();
}
MOTOR_Encoder motor_encoder;
/**
 * @brief 读取编码器
 *
 */
static int wheel_A_num = 0; // 轮子转过多少圈计数
static int wheel_B_num = 0;
static int wheel_C_num = 0;
static int wheel_D_num = 0;
void set_wheel_num(int *deta, int *wheel_num)
{
    if (*deta > 416)
    {
        (*wheel_num)--; // 对 wheel_num 解引用后递减
    }
    else if (*deta < -416)
    {
        (*wheel_num)++; // 对 wheel_num 解引用后递增
    }
}

/**
 * @brief 修正编码器值
 *
 * @param encoder_value
 * @return int
 */
int Correction_encoder_value(int encoder_value)
{
    int output;
    if (encoder_value > 0)
    {
        output = encoder_value % Encoder_Frequency;
    }
    else if (encoder_value < 0)
    {
        output = Encoder_Frequency - Abs(encoder_value % Encoder_Frequency);
    }
    return output;
}

void encoders_read(void)
{
    float alpha = 0.5f; // 低通滤波器的衰减率
    uint32_t now_time = HAL_GetTick();
    uint16_t d_t = now_time - motor_encoder.last_read_time; // 两次读取的时间间隔
    motor_encoder.last_read_time = now_time;
    // 获取编码器的当前计数值
    motor_encoder.motor_A_encoder_val = (short)__HAL_TIM_GET_COUNTER(&MOTOR_A_ENCODER_TIM);
    motor_encoder.motor_B_encoder_val = (short)__HAL_TIM_GET_COUNTER(&MOTOR_B_ENCODER_TIM);
    motor_encoder.motor_C_encoder_val = (short)__HAL_TIM_GET_COUNTER(&MOTOR_C_ENCODER_TIM);
    motor_encoder.motor_D_encoder_val = (short)__HAL_TIM_GET_COUNTER(&MOTOR_D_ENCODER_TIM);
    // 方向矫正
    motor_encoder.motor_A_encoder_val = -motor_encoder.motor_A_encoder_val;
    motor_encoder.motor_B_encoder_val = -motor_encoder.motor_B_encoder_val;
    // float speed_k = Mecanum_75 * PI / 520.0f;
    float speed_k = 1.0f / (float)Encoder_Frequency;
    // 速度计算 单位，转/s
    motor_encoder.motor_A_speed = (float)(motor_encoder.motor_A_encoder_val - motor_encoder.last_motor_A_encoder_val) * speed_k * 1000.0f / (float)d_t;
    motor_encoder.motor_B_speed = (float)(motor_encoder.motor_B_encoder_val - motor_encoder.last_motor_B_encoder_val) * speed_k * 1000.0f / (float)d_t;
    motor_encoder.motor_C_speed = (float)(motor_encoder.motor_C_encoder_val - motor_encoder.last_motor_C_encoder_val) * speed_k * 1000.0f / (float)d_t;
    motor_encoder.motor_D_speed = (float)(motor_encoder.motor_D_encoder_val - motor_encoder.last_motor_D_encoder_val) * speed_k * 1000.0f / (float)d_t;

    float clampFilter_value = 1000.0f; // 最大变化阈值;
    // 限幅滤波
    motor_encoder.motor_A_speed = clampFilter(motor_encoder.motor_A_speed, motor_encoder.last_motor_A_speed, clampFilter_value);
    motor_encoder.motor_B_speed = clampFilter(motor_encoder.motor_B_speed, motor_encoder.last_motor_B_speed, clampFilter_value);
    motor_encoder.motor_C_speed = clampFilter(motor_encoder.motor_C_speed, motor_encoder.last_motor_C_speed, clampFilter_value);
    motor_encoder.motor_D_speed = clampFilter(motor_encoder.motor_D_speed, motor_encoder.last_motor_D_speed, clampFilter_value);

    // 低通滤波器
    motor_encoder.motor_A_speed = FirstOrderLagFilter(motor_encoder.motor_A_speed, motor_encoder.last_motor_A_speed, alpha);
    motor_encoder.motor_B_speed = FirstOrderLagFilter(motor_encoder.motor_B_speed, motor_encoder.last_motor_B_speed, alpha);
    motor_encoder.motor_C_speed = FirstOrderLagFilter(motor_encoder.motor_C_speed, motor_encoder.last_motor_C_speed, alpha);
    motor_encoder.motor_D_speed = FirstOrderLagFilter(motor_encoder.motor_D_speed, motor_encoder.last_motor_D_speed, alpha);

    // 保存当前速度为下次计算滤波的初始值
    motor_encoder.last_motor_A_speed = motor_encoder.motor_A_speed;
    motor_encoder.last_motor_B_speed = motor_encoder.motor_B_speed;
    motor_encoder.last_motor_C_speed = motor_encoder.motor_C_speed;
    motor_encoder.last_motor_D_speed = motor_encoder.motor_D_speed;

    int deta_A = Correction_encoder_value(motor_encoder.motor_A_encoder_val) - Correction_encoder_value(motor_encoder.last_motor_A_encoder_val);
    int deta_B = Correction_encoder_value(motor_encoder.motor_B_encoder_val) - Correction_encoder_value(motor_encoder.last_motor_B_encoder_val);
    int deta_C = Correction_encoder_value(motor_encoder.motor_C_encoder_val) - Correction_encoder_value(motor_encoder.last_motor_C_encoder_val);
    int deta_D = Correction_encoder_value(motor_encoder.motor_D_encoder_val) - Correction_encoder_value(motor_encoder.last_motor_D_encoder_val);

    set_wheel_num(&deta_A, &wheel_A_num);
    set_wheel_num(&deta_B, &wheel_B_num);
    set_wheel_num(&deta_C, &wheel_C_num);
    set_wheel_num(&deta_D, &wheel_D_num);

    motor_encoder.last_motor_A_encoder_val = motor_encoder.motor_A_encoder_val;
    motor_encoder.last_motor_B_encoder_val = motor_encoder.motor_B_encoder_val;
    motor_encoder.last_motor_C_encoder_val = motor_encoder.motor_C_encoder_val;
    motor_encoder.last_motor_D_encoder_val = motor_encoder.motor_D_encoder_val;
    motor_encoder.wheel_A_distance = (float)(wheel_A_num)*Mecanum_75 * PI + (float)Correction_encoder_value(motor_encoder.motor_A_encoder_val) * Mecanum_75 * PI / 520.0f;
    motor_encoder.wheel_B_distance = (float)(wheel_B_num)*Mecanum_75 * PI + (float)Correction_encoder_value(motor_encoder.motor_B_encoder_val) * Mecanum_75 * PI / 520.0f;
    motor_encoder.wheel_C_distance = (float)(wheel_C_num)*Mecanum_75 * PI + (float)Correction_encoder_value(motor_encoder.motor_C_encoder_val) * Mecanum_75 * PI / 520.0f;
    motor_encoder.wheel_D_distance = (float)(wheel_D_num)*Mecanum_75 * PI + (float)Correction_encoder_value(motor_encoder.motor_D_encoder_val) * Mecanum_75 * PI / 520.0f;
    // printf("num_a=%d, num_b=%d, num_c=%d, num_d=%d\n", wheel_A_num, wheel_B_num, wheel_C_num, wheel_D_num);
    // printf("A_speed=%f, B_speed=%f, C_speed=%f, D_speed=%f\n", motor_encoder.motor_A_speed, motor_encoder.motor_B_speed, motor_encoder.motor_C_speed, motor_encoder.motor_D_speed);
    // printf("A_encoder_val=%d, B_encoder_val=%d, C_encoder_val=%d, D_encoder_val=%d\n", motor_encoder.motor_A_encoder_val, motor_encoder.motor_B_encoder_val, motor_encoder.motor_C_encoder_val, motor_encoder.motor_D_encoder_val);
    // printf("A_speed=%d, B_speed=%d, C_speed=%d, D_speed=%d\n", motor_encoder.motor_A_encoder_val, motor_encoder.motor_B_encoder_val, motor_encoder.motor_C_encoder_val, motor_encoder.motor_D_encoder_val);
    // printf("d_time=%d\n", d_t);
}

void calibrateDistanceToZero(void)
{
    __HAL_TIM_SET_COUNTER(&MOTOR_A_ENCODER_TIM, 0);
    __HAL_TIM_SET_COUNTER(&MOTOR_B_ENCODER_TIM, 0);
    __HAL_TIM_SET_COUNTER(&MOTOR_C_ENCODER_TIM, 0);
    __HAL_TIM_SET_COUNTER(&MOTOR_D_ENCODER_TIM, 0);
    wheel_A_num = 0;
    wheel_B_num = 0;
    wheel_C_num = 0;
    wheel_D_num = 0;
}

/**
 * @brief PID控制句柄
 *
 */
pid motor_A_pid;
pid motor_B_pid;
pid motor_C_pid;
pid motor_D_pid;
pid base_rotation_pid; // 小车旋转角度 pid控制句柄

/**
 * @brief 设置电机速度
 *
 * @param motor_id
 * @param speed 单位，转/s
 */
float last_error = 0.0f;
void set_motor_speed(uint8_t motor_id, float target_speed)
{
    float real_speed, error, voltage;
    pid *pid_ctrl = NULL; // 定义指向pid结构体的指针变量

    encoders_read(); // 读取编码器数据

    switch (motor_id)
    {
    case 0:
        real_speed = motor_encoder.motor_A_speed;
        pid_ctrl = &motor_A_pid; // 赋值给指针变量
        break;
    case 1:
        real_speed = motor_encoder.motor_B_speed;
        pid_ctrl = &motor_B_pid; // 赋值给指针变量
        break;
    case 2:
        real_speed = motor_encoder.motor_C_speed;
        pid_ctrl = &motor_C_pid; // 赋值给指针变量
        break;
    case 3:
        real_speed = motor_encoder.motor_D_speed;
        pid_ctrl = &motor_D_pid; // 赋值给指针变量
        break;
    default:
        // 无效的motor_id处理
        return;
    }

    error = target_speed - real_speed;
    error = FirstOrderLagFilter(error, last_error, 0.1f);
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET) // 电机使能开关
    {
        voltage = PID_Control(pid_ctrl, error);
    }
    else
    {
        voltage = 0.0f;
    }
    // voltage = clamp(voltage, -100.0f, 100.0f);
    // 使用指针变量传递到PID_Control中
    // printf("error=%f, voltage=%f,real_speed=%f\n", error, voltage, real_speed);
    set_motor_Voltage(motor_id, voltage); // 设置电机电压
    last_error = error;
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
 * @brief pid数值初始化
 *
 */
void PID_Init(void)
{
    pid_base_init(&motor_A_pid);
    pid_base_init(&motor_B_pid);
    pid_base_init(&motor_C_pid);
    pid_base_init(&motor_D_pid);
    pid_base_init(&base_rotation_pid);
    motor_A_pid.Kp = 100.0f;
    motor_A_pid.Ki = 0.0f;
    motor_A_pid.Kd = 5.0f;
    // motor_A_pid.Kp = 60.0f;
    // motor_A_pid.Ki = 0.05f;
    // motor_A_pid.Kd = 0.0f;

    motor_B_pid.Kp = motor_A_pid.Kp;
    motor_B_pid.Ki = motor_A_pid.Ki;
    motor_B_pid.Kd = motor_A_pid.Kd;

    motor_C_pid.Kp = motor_A_pid.Kp;
    motor_C_pid.Ki = motor_A_pid.Ki;
    motor_C_pid.Kd = motor_A_pid.Kd;

    motor_D_pid.Kp = motor_A_pid.Kp;
    motor_D_pid.Ki = motor_A_pid.Ki;
    motor_D_pid.Kd = motor_A_pid.Kd;

    base_rotation_pid.Kp = 100.0f; // 60.0f
    base_rotation_pid.Ki = 0.0f;   // 0.18f
    base_rotation_pid.Kd = 0.0f;   // 40.0f
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
    x_speed = -x_speed;
    rot_speed = degreesToRadians(rot_speed);
    float V_A, V_B, V_C, V_D;
    float MOTOR_TO_CENTER = MEC_wheelspacing + MEC_axlespacing; // 轮子间距的一半
    V_A = (-x_speed + y_speed + rot_speed * MOTOR_TO_CENTER) / Mecanum_75 * 2.0f;
    V_B = (x_speed + y_speed + rot_speed * MOTOR_TO_CENTER) / Mecanum_75 * 2.0f;
    V_C = (-x_speed + y_speed - rot_speed * MOTOR_TO_CENTER) / Mecanum_75 * 2.0f;
    V_D = (x_speed + y_speed - rot_speed * MOTOR_TO_CENTER) / Mecanum_75 * 2.0f;
    set_motor_speed(0, V_A);
    set_motor_speed(1, V_B);
    set_motor_speed(2, V_C);
    set_motor_speed(3, V_D);
    // printf("x_speed=%f, y_speed=%f, rot_speed=%f\n", x_speed, y_speed, rot_speed);
    // printf("V_A=%f, V_B=%f, V_C=%f, V_D=%f\n", V_A, V_B, V_C, V_D);
}

/**
 * @brief 世界坐标的小车旋转角度PID控制
 *
 * @param target_angle  目标角度
 */

uint8_t base_rotation_control_world(float target_angle, float speed)
{
    speed = speed / 4.0f;
    float clamp_value = 40.0f; // 限幅值
    float error, output;
    target_angle = degreesToRadians(target_angle);
    error = target_angle - Get_IMU_Yaw();
    // error = FirstOrderLagFilter(error, base_rotation_pid.err_last, 0.10f);

    output = PID_Control(&base_rotation_pid, error);

    output = clamp(output, -clamp_value, clamp_value);
    if (output > 0)
    {
        output = float_Map(output, 0, clamp_value, 7, clamp_value);
        output = clamp(output, 7, clamp_value * speed);
    }
    else if (output < 0)
    {
        output = float_Map(output, -clamp_value, 0, -clamp_value, -7);
        output = -clamp(-output, 7, clamp_value * speed);
    }
    // printf("error=%.2f,target_angle=%.2f,integral=%.2f,output=%.2f\n", radiansToDegrees(error), radiansToDegrees(target_angle), base_rotation_pid.integral, output);
    // printf("output=%.2f\n", output);
    base_control(0, 0, output);
    return Abs(radiansToDegrees(error)) < 2.0f ? 1 : 0;
}

void base_run_distance2_base(float distance, float speed, uint8_t mode)
{
    uint8_t dir = 0;
    if (distance < 0)
    {
        distance = -distance;
        dir = 1;
    }
    speed = Abs(speed);
    float start_yaw, error_angle, rotate_speed;
    float start_distance_A, start_distance_B, start_distance_C, start_distance_D;
    float now_distance_A, now_distance_B, now_distance_C, now_distance_D;
    float avg_distance, error_distance;
    float smoothed_speed = 0;
    float alpha = 0.005;         // 加速平滑系数
    float SlowDown_alpha = 0.03; // 减速平滑系数
    start_distance_A = motor_encoder.wheel_A_distance;
    start_distance_B = motor_encoder.wheel_B_distance;
    start_distance_C = motor_encoder.wheel_C_distance;
    start_distance_D = motor_encoder.wheel_D_distance;
    float start_time = (float)HAL_GetTick() / 1000.0f;
    start_yaw = Get_IMU_Yaw();
    for (;;)
    {
        now_distance_A = Abs(motor_encoder.wheel_A_distance - start_distance_A);
        now_distance_B = Abs(motor_encoder.wheel_B_distance - start_distance_B);
        now_distance_C = Abs(motor_encoder.wheel_C_distance - start_distance_C);
        now_distance_D = Abs(motor_encoder.wheel_D_distance - start_distance_D);
        avg_distance = (now_distance_A + now_distance_B + now_distance_C + now_distance_D) / 4.0f;
        error_angle = start_yaw - Get_IMU_Yaw();
        error_distance = distance - avg_distance;
        rotate_speed = error_angle * 60.0f;
        // printf("error_angle=%.2f,smoothed_speed=%.2f,D_A=%.2f,D_B=%.2f,D_C=%.2f,D_D=%.2f,avg_D=%.2f\n", radiansToDegrees(error_angle), smoothed_speed, now_distance_A, now_distance_B, now_distance_C, now_distance_D, avg_distance);
        if (mode == 0)
        {
            base_control(0, (dir == 0 ? 1 : -1) * smoothed_speed, rotate_speed);
        }
        else if (mode == 1)
        {
            base_control((dir == 0 ? 1 : -1) * smoothed_speed, 0, rotate_speed);
        }
        // smoothed_speed = smoothed_speed * (1 - alpha) + alpha * speed; // 平滑加速
        float switch_distance = distance < 20.0f ? 0.3f * distance : 40.0f;
        if (error_distance > switch_distance)
        {
            smoothed_speed = smoothed_speed * (1 - alpha) + alpha * speed; // 平滑加速
        }
        else
        {
            smoothed_speed = smoothed_speed * (1 - SlowDown_alpha);           // 平滑加速
            smoothed_speed = smoothed_speed > speed ? speed : smoothed_speed; // 限幅
            smoothed_speed = smoothed_speed < 3 ? 3 : smoothed_speed;         // 限幅
        }
        if (avg_distance >= distance)
        {

            for (uint16_t i = 0; i < 800; i++)
            {
                base_control(0, 0, 0);
                osDelay(1);
            }

            osDelay(1);
            motor_stop_all();

            break;
        }
        if (((float)HAL_GetTick() / 1000.0f) - start_time > distance / 10.0f * 2.0f)
        {
            break;
        }
    }
}
void base_run_distance2(float distance, float speed)
{
    base_run_distance2_base(distance, speed, 0);
}
void base_Horizontal_run_distance2(float distance, float speed)
{
    base_run_distance2_base(distance, speed, 1);
}
void base_run_distance_base(float distance, float speed, uint8_t mode)
{
    uint8_t dir = 0;
    if (distance < 0)
    {
        distance = -distance;
        dir = 1;
    }
    speed = Abs(speed);
    float start_yaw, error_angle, rotate_speed;
    float start_distance_A, start_distance_B, start_distance_C, start_distance_D;
    float now_distance_A, now_distance_B, now_distance_C, now_distance_D;
    float avg_distance;
    float smoothed_speed = 0;
    start_distance_A = motor_encoder.wheel_A_distance;
    start_distance_B = motor_encoder.wheel_B_distance;
    start_distance_C = motor_encoder.wheel_C_distance;
    start_distance_D = motor_encoder.wheel_D_distance;

    start_yaw = Get_IMU_Yaw();
    uint16_t cycle_time = 0;
    for (;;)
    {
        now_distance_A = Abs(motor_encoder.wheel_A_distance - start_distance_A);
        now_distance_B = Abs(motor_encoder.wheel_B_distance - start_distance_B);
        now_distance_C = Abs(motor_encoder.wheel_C_distance - start_distance_C);
        now_distance_D = Abs(motor_encoder.wheel_D_distance - start_distance_D);
        avg_distance = (now_distance_A + now_distance_B + now_distance_C + now_distance_D) / 4.0f;
        error_angle = start_yaw - Get_IMU_Yaw();
        rotate_speed = error_angle * 60.0f;
        smoothed_speed = S_Curve_Smoothing(0, speed, 4, cycle_time, 1000, 0.01f);
        // printf("smoothed_speed=%.2f,cycle_time=%d\n", smoothed_speed, cycle_time);
        // printf("error_angle=%.2f,smoothed_speed=%.2f,D_A=%.2f,D_B=%.2f,D_C=%.2f,D_D=%.2f,avg_D=%.2f\n", radiansToDegrees(error_angle), smoothed_speed, now_distance_A, now_distance_B, now_distance_C, now_distance_D, avg_distance);
        if (mode == 0)
        {
            base_control(0, (dir == 0 ? 1 : -1) * smoothed_speed, rotate_speed);
        }
        else if (mode == 1)
        {
            base_control((dir == 0 ? 1 : -1) * smoothed_speed, 0, rotate_speed);
        }

        cycle_time++;
        if (avg_distance >= distance)
        {

            for (uint16_t i = 0; i < 800; i++)
            {
                base_control(0, 0, 0);
                osDelay(1);
            }
            motor_stop_all();
            // base_run_angle(radiansToDegrees(start_yaw - Get_IMU_Yaw()), 3);
            // motor_stop_all();
            osDelay(1);

            break;
        }
    }
}
void base_run_distance(float distance, float speed)
{
    base_run_distance2_base(distance, speed, 0);
}

// 小车水平运动指定距离
void base_Horizontal_run_distance(float distance, float speed)
{
    base_run_distance2_base(distance, speed, 1);
}

void base_rotation_world(float angle, float speed)
{
    float flag = 0;

    for (;;)
    {
        flag = base_rotation_control_world(angle, speed);
        // printf("flag=%d, yaw=%.2f\n", flag, radiansToDegrees(Get_IMU_Yaw_speed()));
        if ((flag != 0) && (Abs(Get_IMU_Yaw_speed()) < 0.01f))
        {
            for (uint8_t i = 0; i < 255; i++)
            {
                base_control(0, 0, 0);
                osDelay(1);
            }
            motor_stop_all();
            osDelay(1);
            break;
        }

        osDelay(1);
    }
}

/**
 * @brief 小车旋转指定角度
 *  阻塞型函数
 * @param angle 角度 顺时针为正，单位 度
 * @param speed
 */
void base_run_angle(float angle, float speed)
{

    angle = degreesToRadians(angle);
    float flag = 0;
    float start_angle, target_angle;
    osDelay(100);
    start_angle = Get_IMU_Yaw();
    target_angle = start_angle + angle;

    for (;;)
    {
        flag = base_rotation_control_world(radiansToDegrees(target_angle), speed);
        // printf("flag=%d, yaw=%.2f\n", flag, radiansToDegrees(Get_IMU_Yaw_speed()));
        if ((flag != 0) && (Abs(Get_IMU_Yaw_speed()) < 0.01f))
        {
            for (uint8_t i = 0; i < 255; i++)
            {
                base_control(0, 0, 0);
                osDelay(1);
            }
            motor_stop_all();
            osDelay(1);
            break;
        }

        osDelay(1);
    }
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