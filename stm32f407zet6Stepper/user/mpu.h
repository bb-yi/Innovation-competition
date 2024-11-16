#ifndef __MPU_H__
#define __MPU_H__
#include "main.h"

#define FRAME_HEAD 0xfc     // 帧头
#define FRAME_END 0xfd      // 帧尾
#define TYPE_IMU 0x40       // 经过校准的IMU数据
#define TYPE_AHRS 0x41      // AHRS数据
#define TYPE_INSGPS 0x42    // INS/GPS数据
#define TYPE_SYS_STATE 0x50 // 系统全部数据
#define TYPE_GROUND 0xf0    // 传感器原始数�??
#define IMU_LEN 0x38        // 56+8  8组数�??
#define AHRS_LEN 0x30       // 48+8  7组数�??
#define INSGPS_LEN 0x42     // 72+8  10组数�??
#define ISYS_STATE_LEN 0x64 // 72+8  10组数�??

typedef struct
{
    uint16_t System_status;     // 系统状态，单位：无
    uint16_t Filter_status;     // 滤波器状态，单位：无
    uint32_t Unix_time;         // Unix时间，单位：秒（s）
    uint32_t Microseconds;      // 微秒时间，单位：微秒（μs）
    uint32_t last_Microseconds; // 上一次的微秒时间，单位：微秒（μs）

    double Latitude;           // 纬度，单位：弧度（rad）
    double Longitude;          // 经度，单位：弧度（rad）
    double Height;             // 高度，单位：米（m）
    float Velocity_north;      // 北向速度，单位：米/秒（m/s）
    float Velocity_east;       // 东向速度，单位：米/秒（m/s）
    float Velocity_down;       // 下向速度，单位：米/秒（m/s）
    float Body_acceleration_X; // X轴加速度，单位：米/秒²（m/s²）
    float Body_acceleration_Y; // Y轴加速度，单位：米/秒²（m/s²）
    float Body_acceleration_Z; // Z轴加速度，单位：米/秒²（m/s²）
    float G_force;             // 重力加速度，单位：米/秒²（m/s²）
    float real_Roll;           // 原始横滚角，单位：弧度（rad）
    float real_Pitch;          // 原始俯仰角，单位：弧度（rad）
    float real_Yaw;            // 原始偏航角，单位：弧度（rad）
    float Angular_velocity_X;  // X轴角速度，单位：弧度/秒（rad/s）
    float Angular_velocity_Y;  // Y轴角速度，单位：弧度/秒（rad/s）
    float Angular_velocity_Z;  // Z轴角速度，单位：弧度/秒（rad/s）
    float Roll;                // 校准后横滚角，单位：弧度（rad）
    float Pitch;               // 校准后俯仰角，单位：弧度（rad）
    float Yaw;                 // 校准后偏航角，单位：弧度（rad）
    float LastYaw;             // 上一次的偏航角，单位：弧度（rad）
    float FullYaw;             // 无限制偏航角，单位：弧度（rad）
    float roll_offset;         // 偏移量，单位：弧度（rad）
    float pitch_offset;        // 偏移量，单位：弧度（rad）
    float yaw_offset;          // 偏移量，单位：弧度（rad）
} SYS_STATE_Data;

void MPU_Init(void);
void MPU_UARTE_Rx_Callback(uint16_t Size);
void UART5_Send_String_With_Newline(const char *str);

void MPU_SetConfigMode(void);
void MPU_Restart(void);
void MPU_Calibrate(uint8_t mode);

float MPU_get_start_time(void);
void calibrateAngleToZero(float now_angle);
float Get_IMU_Yaw_speed(void);
float Get_IMU_Yaw(void);
uint8_t Get_IMU_Is_Working(void);

#endif