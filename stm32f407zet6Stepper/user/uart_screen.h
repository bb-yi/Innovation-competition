#ifndef __UART_SCREEN_H__
#define __UART_SCREEN_H__
#include "main.h"
void uart_screen_init(void);
void screen_SendString(const char *str);
void screen_printf(uint8_t enable, const char *format, ...);
void screen_printf_with_quotes(const char *input_str);

void Set_display_solid_num(uint16_t num1, uint16_t num2);
#endif
