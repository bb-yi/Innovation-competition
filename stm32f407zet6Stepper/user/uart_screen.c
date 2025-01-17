#include "uart_screen.h"
#include "usart.h"
extern DMA_HandleTypeDef hdma_usart2_rx;

uint8_t uart_screen_rx_data[50] = {0};

void uart_screen_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_screen_rx_data, 49);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

// 发送字符串
void screen_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
// 格式化并发送字符串
void screen_printf(uint8_t enable, const char *format, ...)
{
    char buffer[128]; // 缓冲区大小可以根据需要调整
    va_list args;

    // 初始化参数列表
    va_start(args, format);

    // 将格式化字符串写入缓冲区
    vsnprintf(buffer, sizeof(buffer), format, args);

    // 清理参数列表
    va_end(args);

    // 发送格式化后的字符串
    if (enable == 1)
    {
        screen_SendString(buffer);
    }
}

void screen_printf_with_quotes(const char *input_str)
{
    // 定义一个缓冲区用于存储格式化后的字符串
    char formatted_str[256]; // 根据实际需要的长度调整缓冲区大小

    // 格式化字符串，将输入字符串放入双引号中
    snprintf(formatted_str, sizeof(formatted_str), "page1.t10.txt=\"%s\"\xff\xff\xff", input_str);

    // 调用 screen_printf 输出格式化后的字符串
    screen_printf(1, formatted_str);
}

void Set_display_solid_num(uint16_t num1, uint16_t num2)
{
    screen_printf(1, "obj_list1=%d\xff\xff\xff", num1);
    screen_printf(1, "obj_list2=%d\xff\xff\xff", num2);
}

// screen_print("t0.txt=\"%s\"\xff\xff\xff", Serial_RxPacket);
// screen_print("n0.val=%d\xff\xff\xff", a);