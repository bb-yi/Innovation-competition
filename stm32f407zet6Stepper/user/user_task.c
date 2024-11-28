#include "user_task.h"

#include "cmsis_os.h"
#include "oled.h"
#include "adc_data.h"
#include "MecanumMotionControl.h"
#include "mpu.h"
#include "SliderElevatorControl.h"
#include "openmv.h"
#include "servo.h"
#include "pid.h"
#include "ZDT_Stepper.h"
#include "beep.h"
#include "uart_screen.h"
#include "camera_led.h"

extern SYS_STATE_Data sys_state_data; // 陀螺仪数据
extern OPENMV_data openmv_data;       // 摄像头OpenMV数据
extern uint8_t MPU_RX_flag;           // 陀螺仪接收标志位
extern uint8_t openmv_rx_flag;        // OpenMV接收标志位
extern uint8_t Camera_now_mode;       // 摄像头当前模式
uint8_t firs_flag = 0;

extern osThreadId_t myTask03Handle;

void check_stack_usage(TaskHandle_t task)
{
    UBaseType_t stack_water_mark = uxTaskGetStackHighWaterMark(task);

    printf("Stack high water mark: %u\n", stack_water_mark);
}

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
float now_yaw;
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

    OLED_ShowFloatNum(0, 24, openmv_data.line_distance, 3, 2, 12);
    OLED_ShowFloatNum(0, 36, openmv_data.line_angle, 2, 2, 12);
    if (Get_IMU_Is_Working())
    {
        OLED_ShowString(0, 48, "Y:");
        now_yaw = radiansToDegrees(sys_state_data.FullYaw) >= 180 ? radiansToDegrees(sys_state_data.FullYaw) - 360 : radiansToDegrees(sys_state_data.FullYaw);
        OLED_ShowFloatNum(18, 48, now_yaw, 3, 1, 12);
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
uint8_t is_printf = 1;
void uasrt_screen_task(void)
{
    screen_printf(is_printf, "page1.x1.val=%d\xff\xff\xff", (int16_t)(battery_voltage * 100)); // 电压
    screen_printf(is_printf, "page1.n4.val=%d\xff\xff\xff", MPU_RX_flag);
    screen_printf(is_printf, "page1.n7.val=%d\xff\xff\xff", openmv_rx_flag);
    switch (Camera_now_mode)
    {
    case QR_MODE:
        screen_printf(is_printf, "page1.t15.txt=\"E\"\xff\xff\xff");
        break;
    case CENTER_POSITION_MODE:
        screen_printf(is_printf, "page1.t15.txt=\"Y\"\xff\xff\xff");
        break;
    case HIGH_CENTER_POSITION_MODE:
        screen_printf(is_printf, "page1.t15.txt=\"C\"\xff\xff\xff");
        break;
    case FIND_LINE_MODE:
        screen_printf(is_printf, "page1.t15.txt\"L\"\xff\xff\xff");
        break;
    default:
        break;
    }
    switch (openmv_data.last_identify_color)
    {
    case 1:
        screen_printf(is_printf, "page1.t6.bco=RED\xff\xff\xff");
        break;
    case 2:
        screen_printf(is_printf, "page1.t6.bco=GREEN\xff\xff\xff");
        break;
    case 3:
        screen_printf(is_printf, "page1.t6.bco=BLUE\xff\xff\xff");
        break;
    default:
        screen_printf(is_printf, "page1.t6.bco=WHITE\xff\xff\xff");
        break;
    }
    screen_printf(is_printf, "page1.x4.val=%d\xff\xff\xff", (int16_t)(openmv_data.line_angle * 100));
    screen_printf(is_printf, "page1.x5.val=%d\xff\xff\xff", (int16_t)(openmv_data.line_distance * 100));
    screen_printf(is_printf, "page1.n0.val=%d\xff\xff\xff", (openmv_data.hsa_circle));
    screen_printf(is_printf, "page1.x0.val=%d\xff\xff\xff", (int16_t)(openmv_data.object_position_x * 100));
    screen_printf(is_printf, "page1.x2.val=%d\xff\xff\xff", (int16_t)(openmv_data.object_position_y * 100));
    screen_printf(is_printf, "page1.x3.val=%d\xff\xff\xff", (int16_t)(now_yaw * 10));
    screen_printf(is_printf, "page1.n3.val=%d\xff\xff\xff", (int16_t)(firs_flag));
    UBaseType_t neicun = uxTaskGetStackHighWaterMark(myTask03Handle);

    screen_printf(is_printf, "page1.n5.val=%d\xff\xff\xff", (int16_t)(neicun));

    // screen_printf(is_printf, "page1.n1.val=%d\xff\xff\xff", openmv_data.object_list[0]);
    // screen_printf(is_printf, "page1.n2.val=%d\xff\xff\xff", openmv_data.object_list[1]);
}

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

void check_uart_receive_status(void)
{
    if (!__HAL_DMA_GET_IT_SOURCE(&hdma_usart2_rx, DMA_IT_TC))
    {
        uart_screen_init();
    }
    if (!__HAL_DMA_GET_IT_SOURCE(&hdma_uart4_rx, DMA_IT_TC))
    {
        openmv_uart_init();
    }
    if (!__HAL_DMA_GET_IT_SOURCE(&hdma_uart5_rx, DMA_IT_TC))
    {
        MPU_Init();
    }
    if (!__HAL_DMA_GET_IT_SOURCE(&hdma_usart3_rx, DMA_IT_TC))
    {
        ZDT_Stepper_USRT_Init();
    }
}

uint8_t delay_time = 5;
void check_stepper_is_working(void)
{
    osDelay(500);
    ZDT_Stepper_Read_version(1);
    osDelay(delay_time);
    ZDT_Stepper_Read_version(2);
    osDelay(delay_time);
    ZDT_Stepper_Read_version(3);
    osDelay(delay_time);
    ZDT_Stepper_Read_version(4);
    osDelay(delay_time);
    ZDT_Stepper_Read_version(5);
    osDelay(delay_time);
}

void enable_stepper_task(void)
{
    ZDT_Stepper_release_stall_protection(0);
    osDelay(delay_time);
    ZDT_Stepper_release_stall_protection(1);
    osDelay(delay_time);
    ZDT_Stepper_release_stall_protection(2);
    osDelay(delay_time);
    ZDT_Stepper_release_stall_protection(3);
    osDelay(delay_time);
    ZDT_Stepper_Enable(1, Enable, SYNC_ENABLE);
    beep_short();
    // osDelay(delay_time);
    ZDT_Stepper_Enable(2, Enable, SYNC_ENABLE);
    beep_short();
    // osDelay(delay_time);
    ZDT_Stepper_Enable(3, Enable, SYNC_ENABLE);
    beep_short();
    // osDelay(delay_time);
    ZDT_Stepper_Enable(4, Enable, SYNC_ENABLE);
    beep_short();
    // osDelay(delay_time);
    ZDT_Stepper_start_sync_motion(0);
    osDelay(100);
    if (check_motor_is_enable() == 1)
    {
        beep_short();
        beep_short();
    }
    else
    {
        beep_long();
    }
}
extern osThreadId_t myTask03Handle;
uint8_t start_flag = 0;
void init_task(void)
{
    set_solid_enable(0);
    Camera_switch_mode(FIND_LINE_MODE);
    Set_Camera_Led_light(100);

    check_stepper_is_working();
    for (;;)
    {
        osDelay(1);
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
        {
            set_beep_long_flag();
            break;
        }
    }
    Set_display_solid_num(0, 0);
    enable_stepper_task();
    printf("电机使能状态: %d\n", check_motor_is_enable());
    motor_stop_all();
    osDelay(200);
    Slider_position_init();
    for (;;)
    {
        // printf("start_flag=%d\r\n", start_flag);
        osDelay(1);
        if (start_flag == 1)
        {
            printf("start\r\n");
            break;
        }
    }
    Set_Table_Pos(3);

    set_solid_enable(1);
    Set_Sliding_table_Pos(0); // 滑台舵机展开
}

// 二维码区域任务
void QrCode_Task(void)
{
    uart_screen_init();
    uint32_t start_time = HAL_GetTick();
    uint32_t now_time;
    for (;;)
    {
        osDelay(1);
        if (openmv_data.object_list[0] != 0 && openmv_data.object_list[1] != 0)
        {
            printf("扫到二维码\r\n");
            break;
        }
        now_time = HAL_GetTick();
        if (now_time - start_time > 4000)
        {
            base_run_distance(15, 20);
            base_run_distance(-30, 20);
            base_run_distance(15, 20);
        }
    }
    // set_Slider_position(148, 60);
}

pid find_circle_pid;
pid find_circle_yaw_pid;
// 原料区为0 放置为1
void find_circle(uint8_t mode, uint8_t error_mode)
{
    // x 0.5 y 0.42
    float PositionThreshold;
    float target_x, target_y;
    float default_error_threshold = 0.01f;
    default_error_threshold = (error_mode == 0 ? 0.002f : 0.01f);
    PositionThreshold = (mode == 0 ? 0.01f : default_error_threshold);
    float clamp_value = 10.0f;
    if (mode == 0)
    {
        find_circle_pid.Kp = 35.0f;
        find_circle_pid.Ki = 0.00f;
        find_circle_pid.Kd = 0.0f;
        clamp_value = 10.0f;
        target_x = 0.540f;
        target_y = 0.505f;
    }
    else
    {
        find_circle_pid.Kp = 50.0f;
        find_circle_pid.Ki = 0.0f;
        find_circle_pid.Kd = 0.0f;
        clamp_value = 10.0f;
        target_x = 0.468f;
        target_y = 0.492f;
    }
    if (error_mode == 1)
    {
        target_x = 0.46f;
        target_y = 0.5f;
    }
    find_circle_yaw_pid.Kp = 0.0f;
    find_circle_yaw_pid.Ki = 0.00f;
    find_circle_yaw_pid.Kd = 0.0f;

    float error_x, error_y, output_x, output_y;
    float start_yaw = Get_IMU_Yaw();
    float error_yaw, output_yaw;

    set_beep_short_flag();
    osDelay(200);
    for (;;)
    {
        error_x = -(target_x - openmv_data.object_position_y);
        error_y = (target_y - openmv_data.object_position_x);
        error_yaw = start_yaw - Get_IMU_Yaw();
        output_x = PID_Control(&find_circle_pid, error_x, 1000);
        output_y = PID_Control(&find_circle_pid, error_y, 1000);
        output_yaw = PID_Control(&find_circle_yaw_pid, error_yaw, 20);
        output_x = clamp(output_x, -clamp_value, clamp_value);
        output_y = clamp(output_y, -clamp_value, clamp_value);
        // printf("error_x=%.2f,error_y=%.2f,x=%.2f,y=%.2f,yaw:%.2f,has_circle:%d,color:%d\r\n", error_x, error_y, output_x, output_y, output_yaw, openmv_data.hsa_circle, openmv_data.last_identify_color);
        if (openmv_data.hsa_circle == 1)
        {
            base_speed_control(output_x, output_y, output_yaw, 200);
            if (Abs(error_x) < PositionThreshold && Abs(error_y) < PositionThreshold)
            {
                motor_stop_all();
                set_beep_short_flag();
                printf("find circle,now_x%.4f,now_y%.4f,error_x%.4f,error_y%.4f\r\n", openmv_data.object_position_x, openmv_data.object_position_y, error_x, error_y);
                find_circle_pid.integral = 0.0f;
                break;
            }
        }
        else
        {
            printf("no circle\r\n");
            set_beep_long_flag();
            find_circle_pid.integral = 0.0f;
            motor_stop_all();
            osDelay(10);
        }

        osDelay(1);
    }
}

void find_line_calibrate_MPU(float now_angle)
{

    Camera_switch_mode(FIND_LINE_MODE);
    osDelay(1000);
    for (uint16_t i = 0; i < 3; i++)
    {

        float line_now_angle = Get_find_line_angle_avg(10);
        base_run_angle(line_now_angle * 0.8f, 1);
        calibrateAngleToZero(now_angle);
    }
}

pid fine_line_angle_PID;
void find_line_calibrate_MPU_PID(float now_angle)
{
    Camera_switch_mode(FIND_LINE_MODE);

    fine_line_angle_PID.Kp = 2.0f; // 0.5f
    fine_line_angle_PID.Ki = 0.0f;
    fine_line_angle_PID.Kd = 0.0f;

    osDelay(200);
    float error_angle, output;
    for (;;)
    {
        error_angle = Get_find_line_angle();
        error_angle = clamp(error_angle, -5, 5);
        output = PID_Control(&fine_line_angle_PID, error_angle, 10);
        output = clamp(output, -3, 3);
        // printf("error_angle=%.2f,output=%.2f\r\n", error_angle, output);
        base_speed_control(0, 0, -output, 800);
        calibrateAngleToZero(now_angle);
        if (Abs(error_angle) < 0.2f)
        {
            motor_stop_all();
            for (uint16_t i = 0; i < 200; i++)
            {
                osDelay(1);
            }
            if (Abs(error_angle) < 0.2f)
            {
                printf("find line\r\n");
                motor_stop_all();
                break;
            }
        }
        osDelay(1);
    }
}
pid fine_line_pos_PID;
void find_line_distance(float distance)
{
    fine_line_pos_PID.Kp = 1.0f;
    fine_line_pos_PID.Ki = 0.0f;
    fine_line_pos_PID.Kd = 0.0f;

    Camera_switch_mode(FIND_LINE_MODE);
    float error_pos, output;
    for (;;)
    {
        error_pos = distance - Get_find_line_distance();
        error_pos = clamp(error_pos, -10, 10);
        output = PID_Control(&fine_line_pos_PID, error_pos, 10);
        output = clamp(output, -10, 10);
        printf("error_pos=%.2f,output=%.2f\r\n", error_pos, output);
        base_speed_control(output, 0, 0, 800);
        if (Abs(error_pos) < 1.0f)
        {
            for (uint16_t i = 0; i < 500; i++)
            {
                osDelay(1);
            }
            if (Abs(error_pos) < 1.0f)
            {
                printf("find line\r\n");
                motor_stop_all();
                break;
            }
        }
        osDelay(1);
    }
}

// 原料区任务

void MaterialArea_Task(uint8_t part)
{
    firs_flag = 0;
    Camera_switch_mode(CENTER_POSITION_MODE);
    // Camera_switch_mode(CENTER_POSITION_MODE);

    uint16_t object_list_num = openmv_data.object_list[part];
    set_Slider_position(150, 100);
    // find_circle(0);
    printf("finish\r\n");
    osDelay(200);

    for (;;)
    {
        osDelay(1);
        if (openmv_data.identify_color != extract_digit(object_list_num, 3) && openmv_data.identify_color != 0 && openmv_data.hsa_circle == 1)
        {
            firs_flag = 1;
        }
        printf("color:%d,firs_flag:%d\r\n", openmv_data.identify_color, firs_flag);

        if (openmv_data.identify_color == extract_digit(object_list_num, 3) && firs_flag == 1 && openmv_data.hsa_circle == 1)
        {
            set_beep_short_flag();
            break;
        }
    }
    printf("找到第一个\r\n");

    Get_material(openmv_data.last_identify_color - 1);
    osDelay(200);

    for (;;)
    {
        osDelay(1);
        printf("color:%d\r\n", openmv_data.identify_color);

        if (openmv_data.identify_color == extract_digit(object_list_num, 2) && openmv_data.hsa_circle == 1)
        {

            set_beep_short_flag();
            break;
        }
    }
    printf("找到第二个\r\n");
    Get_material(openmv_data.last_identify_color - 1);
    osDelay(200);

    for (;;)
    {
        osDelay(1);
        printf("color:%d\r\n", openmv_data.identify_color);
        if (openmv_data.identify_color == extract_digit(object_list_num, 1) && openmv_data.hsa_circle == 1)
        {
            set_beep_short_flag();
            break;
        }
    }
    printf("找到第三个\r\n");
    Get_material(openmv_data.last_identify_color - 1);
    Set_Table_Pos(3);

    Camera_switch_mode(FIND_LINE_MODE);
    osDelay(1000);
}
float task_move_speed = 120.0f;
// 粗加工区任务
void RoughProcessingArea_Task(uint16_t object_num)
{

    // find_line_calibrate_MPU_PID(0);
    osDelay(500);
    Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    object_num = openmv_data.object_list[object_num];

    float move_distance = 15;
    int16_t first_num = extract_digit(object_num, 3);
    int16_t second_num = extract_digit(object_num, 2);
    int16_t third_num = extract_digit(object_num, 1);

    find_circle(1, 0);
    printf("finish\r\n");
    if (openmv_data.last_identify_color != extract_digit(object_num, 3))
    {
        printf("颜色不对\r\n");
        base_run_distance(move_distance * (extract_digit(object_num, 3) - openmv_data.last_identify_color), 100);
        osDelay(200);
        find_circle(1, 0);
        printf("finish\r\n");
    }
    osDelay(500);
    Put_material(first_num - 1);
    // printf("放下第一个%d\r\n", first_num);
    osDelay(200);
    // find_line_calibrate_MPU_PID(0);
    // base_rotation_world_base(180, 30);

    Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    base_run_distance(move_distance * (second_num - first_num), task_move_speed);

    osDelay(200);
    find_circle(1, 0);
    printf("finish\r\n");
    osDelay(500);
    Put_material(second_num - 1);
    // printf("放下第二个%d\r\n", second_num);
    osDelay(200);
    // base_rotation_world_base(180, 30);

    // find_line_calibrate_MPU_PID(0);
    Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    base_run_distance(move_distance * (third_num - second_num), task_move_speed);

    osDelay(1000);
    find_circle(1, 0);
    printf("finish\r\n");
    osDelay(500);
    Put_material(third_num - 1);
    // printf("放下第三个%d\r\n", third_num);
    osDelay(200);
    // find_line_calibrate_MPU_PID(0);
    // base_rotation_world_base(180, 30);

    // base_run_distance_base(1, 0, 0, Close_speed); // 远离原料区

    // Camera_switch_mode(HIGH_CENTER_POSITION_MODE);

    base_run_distance(move_distance * (first_num - third_num), task_move_speed);

    Get_material_floor(first_num - 1);
    osDelay(200);
    base_run_distance(move_distance * (second_num - first_num), task_move_speed);
    osDelay(200);

    Get_material_floor(second_num - 1);
    osDelay(200);
    base_run_distance(move_distance * (third_num - second_num), task_move_speed);
    osDelay(200);
    Camera_switch_mode(FIND_LINE_MODE);

    Get_material_floor(third_num - 1);
    Set_Table_Pos(3);

    osDelay(1000);
}

void TemporaryStorageArea_Task(uint8_t part)
{
    // find_line_calibrate_MPU_PID(0);
    osDelay(500);
    if (part == 0)
    {
        Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    }
    else
    {
        Camera_switch_mode(Stacking_MODE);
    }
    uint16_t object_list_num = openmv_data.object_list[part];
    float move_distance = 15;
    int16_t first_num = extract_digit(object_list_num, 3);
    int16_t second_num = extract_digit(object_list_num, 2);
    int16_t third_num = extract_digit(object_list_num, 1);
    uint8_t error_mode = 0;
    error_mode = (part == 0 ? 0 : 1);
    find_circle(1, error_mode);
    printf("finish\r\n");
    if (openmv_data.last_identify_color != extract_digit(object_list_num, 3))
    {
        printf("颜色不对\r\n");
        base_run_distance(move_distance * (extract_digit(object_list_num, 3) - openmv_data.last_identify_color), task_move_speed);
        osDelay(200);
        find_circle(1, error_mode);
        printf("finish\r\n");
    }
    osDelay(500);
    if (part == 0)
    {
        Put_material(first_num - 1);
    }
    else
    {
        Put_material_in_obj(first_num - 1);
    }
    // printf("放下第一个%d\r\n", first_num);
    osDelay(200);
    base_run_distance(move_distance * (second_num - first_num), task_move_speed);
    osDelay(200);
    find_circle(1, error_mode);
    printf("finish\r\n");
    osDelay(500);
    if (part == 0)
    {
        Put_material(second_num - 1);
    }
    else
    {
        Put_material_in_obj(second_num - 1);
    }
    // printf("放下第二个%d\r\n", second_num);
    osDelay(200);
    base_run_distance(move_distance * (third_num - second_num), task_move_speed);

    osDelay(200);
    find_circle(1, error_mode);
    printf("finish\r\n");
    osDelay(500);
    if (part == 0)
    {
        Put_material(third_num - 1);
    }
    else
    {
        Put_material_in_obj(third_num - 1);
    }
    // printf("放下第三个%d\r\n", third_num);
    Camera_switch_mode(FIND_LINE_MODE);
    Set_Table_Pos(3);

    osDelay(1000);
}

extern uint8_t Slider_is_OK;
float run_speed = 145.0f; // 最大速度 150
float acc_speed = 25.0f;  // 加速度 10.0f
float rot_speed = 250;
float Close_speed = 50;
float line_distance = 80;
uint16_t default_delay = 1;
uint8_t run_task = 1;

uint8_t main_task(void)
{
    float target_cycle_distance = run_task == 0 ? 0 : 15;
    // !开始-----------------------------------------------------------------------------------------------------------
    printf("开始\r\n");
    osDelay(default_delay);
    Camera_switch_mode(FIND_LINE_MODE);
    base_run_distance_base(15, 25, 0, run_speed); // 移出启停区
    set_Slider_position_2(0, 500);                // #提前降低滑台 二维码高度
    osDelay(10);

    base_run_distance_fix(38, run_speed, 1, 80); // 去往二维码区域

    // $ 二维码任务------------------------------------------------------------------------------------------------------
    printf("二维码任务\r\n");
    if (run_task == 1)
    {
        QrCode_Task();
    }
    // $ 二维码任务------------------------------------------------------------------------------------------------------

    osDelay(10);
    set_Slider_position_2(150, 500); // #提前升起
    osDelay(10);
    base_run_distance_fix(79, run_speed, 1, line_distance); // 去往原料区
    osDelay(default_delay);
    find_line_calibrate_MPU_PID(0);                  // 寻线校准
    base_run_distance_base(-7, 0, 0, Close_speed); // 靠近原料区
    osDelay(default_delay);

    printf("原料区任务\r\n");
    // $ 原料区任务------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        MaterialArea_Task(0); // 原料区任务
    }
    // $ 原料区任务------------------------------------------------------------------------------------------------------------

    osDelay(default_delay);
    base_run_distance_base(6, 0, 0, Close_speed); // 远离原料区
    osDelay(default_delay);
    base_run_distance_fix(42, run_speed, 1, line_distance); // 去往粗加工区1
    osDelay(default_delay);
    base_run_angle(-90, rot_speed); // 转向
    osDelay(default_delay);
    base_run_distance_fix(170, run_speed, 1, line_distance); // 去往粗加工区2
    osDelay(default_delay);
    base_run_angle(-90, rot_speed); // 转向
    osDelay(default_delay);
    float d_distance = target_cycle_distance * (extract_digit(openmv_data.object_list[0], 3) - 2);
    float d_distance2 = target_cycle_distance * (extract_digit(openmv_data.object_list[0], 1) - 2);
    base_run_distance_fix(81 + d_distance, run_speed, 1, line_distance); // 去往粗加工区3
    osDelay(default_delay);
    // find_line_calibrate_MPU_PID(0);                // 寻线校准
    base_run_distance_base(-6, 0, 0, Close_speed); // 靠近粗加工区
    osDelay(default_delay);

    printf("粗加工区任务\r\n");
    // $ 粗加工区任务------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        RoughProcessingArea_Task(0); // 粗加工区任务
    }
    // $ 粗加工区任务------------------------------------------------------------------------------------------------------------

    osDelay(default_delay);
    base_run_distance_fix(-(81 + d_distance2), run_speed, 1, line_distance); // 粗加工区到暂存区1
    osDelay(default_delay);
    base_run_angle(90, rot_speed); // 转向
    osDelay(default_delay);
    base_run_distance_fix(-(79 - d_distance), run_speed, 1, line_distance); // 粗加工区到暂存区2
    osDelay(default_delay);
    base_run_distance_base(-4, 0, 0, Close_speed); // 靠近暂存区
    osDelay(default_delay);

    printf("暂存区任务\r\n");
    // $ 暂存区任务-------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        TemporaryStorageArea_Task(0); // 暂存区任务
    }
    else
    {
        osDelay(1000);
    }
    // $ 暂存区任务-------------------------------------------------------------------------------------------------------------

    base_run_distance_base(4, 0, 0, Close_speed); // 远离暂存区
    osDelay(default_delay);

    printf("第二次搬运\r\n");
    // ~第二次搬运-----------------------------------------------------------------------------------------------------------
    base_run_distance_fix(-(86 + d_distance2), run_speed, 1, line_distance); //  暂存区到原料区1
    osDelay(default_delay);
    base_run_angle(90, rot_speed); // 转向
    osDelay(default_delay);
    base_run_distance_fix(-41, run_speed, 1, line_distance); // 暂存区到原料区2
    // find_line_calibrate_MPU_PID(0);                          // 寻线校准
    base_run_distance_base(-5, 0, 0, Close_speed); // 靠近原料区2
    osDelay(default_delay);

    printf("第二次原料区任务\r\n");
    // $ 第二次原料区任务-------------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        MaterialArea_Task(1); // 原料区任务2
    }
    // $ 第二次原料区任务-------------------------------------------------------------------------------------------------------------------

    base_run_distance_base(8, 0, 0, Close_speed); // 远离原料区2
    osDelay(default_delay);
    base_run_distance_fix(42, run_speed, 1, line_distance); // 第二次 去往粗加工区1
    osDelay(default_delay);
    base_run_angle(-90, rot_speed); // 转向

    base_run_distance_fix(170, run_speed, 1, line_distance); // 第二次 去往粗加工区2
    osDelay(default_delay);
    base_run_angle(-90, rot_speed); // 转向
    osDelay(default_delay);
    d_distance = target_cycle_distance * (extract_digit(openmv_data.object_list[1], 3) - 2);
    d_distance2 = target_cycle_distance * (extract_digit(openmv_data.object_list[1], 1) - 2);
    base_run_distance_fix(81 + d_distance, run_speed, 1, line_distance); // 第二次 去往粗加工区3
    osDelay(default_delay);
    base_run_distance_base(-6, 0, 0, Close_speed); // 靠近粗加工区2
    osDelay(default_delay);

    printf("第二次粗加工区任务\r\n");
    // $ 第二次粗加工区任务-------------------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        RoughProcessingArea_Task(1); // 粗加工区任务2
    }
    // $ 第二次粗加工区任务-------------------------------------------------------------------------------------------------------------------------

    osDelay(default_delay);
    base_run_distance_fix(-(81 + d_distance2), run_speed, 1, line_distance); // 第二次 粗加工区到暂存区1
    osDelay(default_delay);
    base_run_angle(90, rot_speed); // 转向
    osDelay(default_delay);
    base_run_distance_fix(-(80 - d_distance), run_speed, 1, line_distance); // 第二次  粗加工区到暂存区2
    osDelay(default_delay);
    base_run_distance_base(-4, 0, 0, Close_speed); // 靠近暂存区2
    osDelay(default_delay);

    printf("第二次暂存区任务\r\n");
    // $ 第二次暂存区任务-------------------------------------------------------------------------------------------------------------------------------
    if (run_task == 1)
    {
        TemporaryStorageArea_Task(1); // 暂存区任务2
    }
    else
    {
        osDelay(1000);
    }
    // $ 第二次暂存区任务-------------------------------------------------------------------------------------------------------------------------------

    // ~回到起始点--------------------------------------------------------------------------------------------------------------------------------------
    base_run_distance_base(4, 0, 0, Close_speed); // 原理暂存区2
    osDelay(default_delay);
    base_run_distance_fix(-(90 + d_distance2), run_speed, 1, line_distance); // 暂存区到起始点1
    osDelay(default_delay);
    base_run_angle(90, rot_speed); // 转向
    osDelay(default_delay);
    base_run_distance_fix(-172, run_speed, 1, line_distance); // 暂存区到起始点2
    osDelay(default_delay);
    base_run_distance_base(-16, -17, 0, run_speed); // 移入启停区
    motor_stop_all();
    printf("结束\r\n");
    // !结束------------------------------------------------------------------------------------------------------------------------------------------------

    return 1;
}