#ifndef __UART_SCREEN_H__
#define __UART_SCREEN_H__
void uart_screen_init(void);
void screen_SendString(const char *str);
void screen_printf(const char *format, ...);
#endif
