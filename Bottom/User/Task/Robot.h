#ifndef ROBOT_H
#define ROBOT_H

#include "stdio.h"
#include "stdint.h"

// #define REMOTE_CONTROL
#define VIDEO_CONTROL

typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    int16_t rotor_speed;
    int16_t torque_current;
    uint8_t temp;
    int16_t set_current;
    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} motor_info_t;

#endif // !ROBOT_H