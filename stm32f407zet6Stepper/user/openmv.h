#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"
#include "usart.h"
#include "dma.h"
extern DMA_HandleTypeDef hdma_uart4_rx;

#define OPENMV_UART_HANDLE huart4
#define OPENMV_UART_DMA_HANDLE hdma_uart4_rx

/*
1  2  3
R  G  B
红 绿 蓝
[e,321,132] 识别二维码
[y,0.5348,0.5076,1] 识别中心位置和颜色
[c,0.5348,0.5076,1] 高分辨率识别中心位置和颜色
[l，0.6500，-14] 直线距离和角度
*/
#define QR_MODE 0
#define CENTER_POSITION_MODE 1
#define HIGH_CENTER_POSITION_MODE 2
#define FIND_LINE_MODE 3
#define Stacking_MODE 4

typedef struct
{
    int object_list[2];      // 物料抓取顺序
    float object_position_x; // 物料位置X
    float object_position_y; // 物料位置Y
    int identify_color;      // 当前识别的颜色
    int last_identify_color; // 上一次识别的颜色
    float line_distance;     // 距离边缘距离
    float line_angle;        // 边缘角度
    uint16_t edge_diastance; // 边缘距离
    uint8_t hsa_circle;      // 是否识别到圆
    uint8_t heartbeat;       // 心跳
} OPENMV_data;

void openmv_uart_init(void);

void openmv_uart_rx_callback(uint16_t Size, uint8_t uart_id);

int extract_digit(int number, int position);
void Camera_SendString(const char *str);
void Camera_switch_mode(uint8_t mode);
float Get_find_line_angle(void);
float Get_find_line_angle_avg(uint8_t times);
float Get_find_line_distance(void);

#endif