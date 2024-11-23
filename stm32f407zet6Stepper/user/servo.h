#ifndef __SERVO_H
#define __SERVO_H
#include "main.h"

/*
0 滑台 ch1 PC6
45 抓取 225 放下

1 物料盘 ch2 PC7
15 左1物料盘 135 中物料盘 255 右1物料盘

2 爪子舵机 ch3 PC8
正向拿取 125 抓 55 放
*/

#define Sliding_table 0
#define Material_tray 1
#define catch 2
void Set_Servo_angle(uint8_t servo, uint16_t angle);
void Servo_Init(void);
void Set_Sliding_table_Pos(uint8_t pos);
void Set_Table_Pos(uint8_t pos);
void Catch_material(void);
void Release_material(void);
void catch_material_in_middle(void);
void Get_material(uint8_t pos);
void Get_material_floor(uint8_t pos);

void Put_material(uint8_t pos);
void Put_material_in_obj(uint8_t pos);
#endif
