#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "main.h"
void OLED_display_task(void);
void find_line_calibrate_MPU(void);
void find_line_calibrate_MPU_PID(void);

void QrCode_Task(void);

void MaterialArea_Task(void);
void RoughProcessingArea_Task(void);
uint8_t main_task(void);
#endif