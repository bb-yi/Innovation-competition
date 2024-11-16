#include "mpu.h"
#include "usart.h"
#include "tool.h"
#define MAX_FRAME_SIZE 324 // 空闲中断接收的最大帧长
extern DMA_HandleTypeDef hdma_uart5_rx;

SYS_STATE_Data sys_state_data;
uint8_t rx_buffer[MAX_FRAME_SIZE];
uint8_t temp_data_buf[MAX_FRAME_SIZE];
uint8_t MPU_RX_flag = 0;

/**
 * @brief 初始化MPU 打开空闲中断接收
 *
 */
void MPU_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buffer, sizeof(rx_buffer)); // 启用空闲中断接收
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

/**
 * @brief 四个字节的数据转换成浮点数
 *
 * @param Data_1
 * @param Data_2
 * @param Data_3
 * @param Data_4
 * @return float
 */
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    long long transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= Data_4 << 24;
    transition_32 |= Data_3 << 16;
    transition_32 |= Data_2 << 8;
    transition_32 |= Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1;
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    return tmp;
}

/**
 * @brief 8个字节的数据转换成双精度浮点数
 *
 * @param Data_1
 * @param Data_2
 * @param Data_3
 * @param Data_4
 * @param Data_5
 * @param Data_6
 * @param Data_7
 * @param Data_8
 * @return double
 */
double DATA_Trans_64(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4, uint8_t Data_5, uint8_t Data_6, uint8_t Data_7, uint8_t Data_8)
{
    uint64_t transition_64 = 0;
    double tmp = 0;
    int sign = 0;
    int exponent = 0;
    double mantissa = 0;

    // 将 8 个字节合并成 64 位的无符号整数
    transition_64 |= (uint64_t)Data_8 << 56;
    transition_64 |= (uint64_t)Data_7 << 48;
    transition_64 |= (uint64_t)Data_6 << 40;
    transition_64 |= (uint64_t)Data_5 << 32;
    transition_64 |= (uint64_t)Data_4 << 24;
    transition_64 |= (uint64_t)Data_3 << 16;
    transition_64 |= (uint64_t)Data_2 << 8;
    transition_64 |= (uint64_t)Data_1;

    // 提取符号位
    sign = (transition_64 & 0x8000000000000000) ? -1 : 1;

    // 提取指数位并减去偏移量（1023）
    exponent = ((transition_64 >> 52) & 0x7FF) - 1023;

    // 提取尾数部分（隐含的 1 要加进去）
    mantissa = 1 + ((double)(transition_64 & 0xFFFFFFFFFFFFF) / (1ULL << 52));

    // 计算最终的浮点数值
    tmp = sign * mantissa * pow(2, exponent);

    return tmp;
}

/**
 * @brief向串口5发送字符串并添加换行符
 *
 * @param str
 */
void UART5_Send_String_With_Newline(const char *str)
{
    // 定义一个带有换行符的字符串缓冲区
    char buffer[100]; // 假设最大长度为100，可以根据需要调整

    // 拼接原始字符串和换行符
    snprintf(buffer, sizeof(buffer), "%s\r\n", str);
    // 发送带有换行符的字符串
    HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * @brief 串口设置MPU进入配置模式
 *
 */
void MPU_SetConfigMode(void)
{
    char config_string[] = "#fconfig";
    // printf("进入配置模式...\r\n");
    UART5_Send_String_With_Newline(config_string);
}

/**
 * @brief 串口设置MPU重启
 *
 */
void MPU_Restart(void)
{
    char freboot_string[] = "#freboot";
    // uint16_t temp_time = 0;
    // for (;;)
    // {
    //     temp_time++;
    //     if (temp_time > 10000)
    //     {
    //         temp_time = 0;
    //         break;
    //     }
    // }
    delay_us(2000000);
    UART5_Send_String_With_Newline(freboot_string);
    // printf("重启中...\r\n");

    char y_string[] = "y";
    delay_us(2000000);

    UART5_Send_String_With_Newline(y_string);
    // printf("yes\r\n");
}

/**
 * @brief 格式1和格式2需要在水平静止状态下执行该命
令；格式3只需要模块保持静止
 * 串口设置MPU校准
 * @param mode
 */
void MPU_Calibrate(uint8_t mode)
{
    delay_us(2000000);
    char level_string[] = "#fimucal_level"; // 将IMU坐标系调平至水平面，不改变陀螺和加表零偏
    char acce_string[] = "#fimucal_acce";   // 执行加速度计零偏校准
    char gyro_string[] = "#fimucal_gyro";   // 执行陀螺仪零偏校准
    switch (mode)
    {
    case 0:
        printf(level_string);
        UART5_Send_String_With_Newline(level_string);
        // printf("水平面校准中...\r\n");
        break;
    case 1:
        printf(acce_string);
        UART5_Send_String_With_Newline(acce_string);
        // printf("加速度计校准中...\r\n");
        break;
    case 2:
        printf(gyro_string);
        UART5_Send_String_With_Newline(gyro_string);
        // printf("陀螺仪校准中...\r\n");
        break;
    default:
        break;
    }
}

/**
 * @brief 获取MPU自从上电后到当前的时间
 *
 * @return float
 */
float MPU_get_start_time(void)
{
    float now_time = (float)(sys_state_data.Unix_time - 315964800) + (float)sys_state_data.Microseconds / 1000000.0f;
    return now_time;
}
static int yaw_num = 0;

/**
 * @brief 按键中断中将MPU数据置0的函数
 *
 */
void calibrateAngleToZero(float now_angle)
{
    sys_state_data.roll_offset = sys_state_data.real_Roll;
    sys_state_data.pitch_offset = sys_state_data.real_Pitch;
    sys_state_data.yaw_offset = sys_state_data.real_Yaw - degreesToRadians(now_angle);
    yaw_num = 0;
}

float Get_IMU_Yaw_speed(void)
{
    return sys_state_data.Angular_velocity_Z;
}

/**
 * @brief 获取mpu当前fullyaw值 单位弧度
 *
 * @return float
 */
float Get_IMU_Yaw(void)
{
    return sys_state_data.FullYaw;
}

/**
 * @brief 获取mpu是否正常工作
 *
 * @return uint8_t 正常接受数据返回1，否则返回0
 */
uint8_t Get_IMU_Is_Working(void)
{
    return (sys_state_data.last_Microseconds != sys_state_data.Microseconds);
}

/**
 * @brief 空闲中断接收函数，接受并解析MPU数据
 *
 * @param Size
 */
void MPU_UARTE_Rx_Callback(uint16_t Size)
{
    MPU_Init();
    uint16_t frame_head_index = 0;

    // HAL_UART_Transmit(&huart1, rx_buffer, Size, HAL_MAX_DELAY);
    // printf("Size = %d\r\n", Size);
    for (int i = 0; i < MAX_FRAME_SIZE; i++)
    {
        if (rx_buffer[i] == FRAME_HEAD)
        {
            frame_head_index = i;
            break;
        }
    }
    if (0)
    {
    }
    else if (Size < 30) // 如果数据长度小于30，则说明数据是返回的指令
    {
        // UART5_Send_String_With_Newline("Received 0x2A23");
        // HAL_UART_Transmit(&huart1, rx_buffer, MAX_FRAME_SIZE, HAL_MAX_DELAY);
    }
    else if (rx_buffer[frame_head_index + 1] == TYPE_SYS_STATE)
    {
        if (rx_buffer[frame_head_index + ISYS_STATE_LEN + 7] == FRAME_END)
        {
            // printf("SYS_STATE\r\n");
            memcpy(&temp_data_buf, &rx_buffer[frame_head_index], ISYS_STATE_LEN);
            sys_state_data.System_status = (temp_data_buf[7] << 8) | temp_data_buf[8];
            sys_state_data.Filter_status = (temp_data_buf[9] << 8) | temp_data_buf[10];

            sys_state_data.Unix_time = temp_data_buf[11] | (temp_data_buf[12] << 8) | (temp_data_buf[13] << 16) | (temp_data_buf[14] << 24);
            sys_state_data.Microseconds = temp_data_buf[15] | (temp_data_buf[16] << 8) | (temp_data_buf[17] << 16) | (temp_data_buf[18] << 24);

            sys_state_data.Latitude = DATA_Trans_64(temp_data_buf[19], temp_data_buf[20], temp_data_buf[21], temp_data_buf[22], temp_data_buf[23], temp_data_buf[24], temp_data_buf[25], temp_data_buf[26]);
            sys_state_data.Longitude = DATA_Trans_64(temp_data_buf[27], temp_data_buf[28], temp_data_buf[29], temp_data_buf[30], temp_data_buf[31], temp_data_buf[32], temp_data_buf[33], temp_data_buf[34]);
            sys_state_data.Height = DATA_Trans_64(temp_data_buf[35], temp_data_buf[36], temp_data_buf[37], temp_data_buf[38], temp_data_buf[39], temp_data_buf[40], temp_data_buf[41], temp_data_buf[42]);

            sys_state_data.Velocity_north = DATA_Trans(temp_data_buf[43], temp_data_buf[44], temp_data_buf[45], temp_data_buf[46]);
            sys_state_data.Velocity_east = DATA_Trans(temp_data_buf[47], temp_data_buf[48], temp_data_buf[49], temp_data_buf[50]);
            sys_state_data.Velocity_down = DATA_Trans(temp_data_buf[51], temp_data_buf[52], temp_data_buf[53], temp_data_buf[54]);

            sys_state_data.Body_acceleration_X = DATA_Trans(temp_data_buf[55], temp_data_buf[56], temp_data_buf[57], temp_data_buf[58]);
            sys_state_data.Body_acceleration_Y = DATA_Trans(temp_data_buf[59], temp_data_buf[60], temp_data_buf[61], temp_data_buf[62]);
            sys_state_data.Body_acceleration_Z = DATA_Trans(temp_data_buf[63], temp_data_buf[64], temp_data_buf[65], temp_data_buf[66]);
            sys_state_data.G_force = DATA_Trans(temp_data_buf[67], temp_data_buf[68], temp_data_buf[69], temp_data_buf[70]);

            sys_state_data.real_Roll = DATA_Trans(temp_data_buf[71], temp_data_buf[72], temp_data_buf[73], temp_data_buf[74]);
            sys_state_data.real_Pitch = DATA_Trans(temp_data_buf[75], temp_data_buf[76], temp_data_buf[77], temp_data_buf[78]);
            sys_state_data.real_Yaw = DATA_Trans(temp_data_buf[79], temp_data_buf[80], temp_data_buf[81], temp_data_buf[82]);

            sys_state_data.Angular_velocity_X = DATA_Trans(temp_data_buf[83], temp_data_buf[84], temp_data_buf[85], temp_data_buf[86]);
            sys_state_data.Angular_velocity_Y = DATA_Trans(temp_data_buf[87], temp_data_buf[88], temp_data_buf[89], temp_data_buf[90]);
            sys_state_data.Angular_velocity_Z = DATA_Trans(temp_data_buf[91], temp_data_buf[92], temp_data_buf[93], temp_data_buf[94]);

            sys_state_data.Roll = sys_state_data.real_Roll - sys_state_data.roll_offset;
            sys_state_data.Pitch = sys_state_data.real_Pitch - sys_state_data.pitch_offset;
            sys_state_data.Yaw = sys_state_data.real_Yaw - sys_state_data.yaw_offset;

            // 确保置零后的数据在原来的范围之间
            if (sys_state_data.Roll > PI)
                sys_state_data.Roll -= 2 * PI;
            else if (sys_state_data.Roll < -PI)
                sys_state_data.Roll += 2 * PI;

            if (sys_state_data.Pitch > PI / 2.0f)
                sys_state_data.Pitch = PI / 2.0f;
            else if (sys_state_data.Pitch < -PI / 2.0f)
                sys_state_data.Pitch = -PI / 2.0f;

            if (sys_state_data.Yaw > PI)
                sys_state_data.Yaw -= 2 * PI;
            else if (sys_state_data.Yaw < -PI)
                sys_state_data.Yaw += 2 * PI;

            // 得到无限制的yaw值
            float delta_yaw = sys_state_data.Yaw - sys_state_data.LastYaw;
            // 检测到角度跳变时的处理
            if (delta_yaw > 0.8f * 2.0f * PI)
            {
                yaw_num--;
            }
            else if (delta_yaw < -0.8f * 2.0f * PI)
            {
                yaw_num++;
            }
            sys_state_data.FullYaw = sys_state_data.Yaw + (float)yaw_num * 2.0f * PI;
            // printf("yaw_num=%d, FullYaw=%f\r\n", yaw_num, sys_state_data.FullYaw);
            sys_state_data.LastYaw = sys_state_data.Yaw;
        }
    }
    memset(rx_buffer, 0, sizeof(rx_buffer));
}