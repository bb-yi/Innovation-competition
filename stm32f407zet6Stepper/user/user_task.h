#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "main.h"
void check_stack_usage(TaskHandle_t task);
void OLED_display_task(void);
void uasrt_screen_task(void);
void check_uart_receive_status(void);

void init_task(void);
void find_circle(uint8_t mode, uint8_t error_mode);
void find_line_calibrate_MPU_PID(float now_angle);
void find_line_calibrate_MPU(float now_angle);
void find_line_distance(float distance);
void QrCode_Task(void);

void MaterialArea_Task(uint8_t part);
void RoughProcessingArea_Task(uint16_t object_num);
void TemporaryStorageArea_Task(uint8_t part);
uint8_t main_task(void);
#endif
