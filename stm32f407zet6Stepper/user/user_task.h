#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "main.h"
void check_stack_usage(TaskHandle_t task);
void OLED_display_task(void);
void uasrt_screen_task(void);

void init_task(void);
void find_circle(uint8_t mode);
void find_line_calibrate_MPU(void);
void find_line_calibrate_MPU_PID(void);

void QrCode_Task(void);

void MaterialArea_Task(void);
void RoughProcessingArea_Task(void);
uint8_t main_task(void);
#endif
