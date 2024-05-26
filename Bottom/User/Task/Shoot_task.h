#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"

#define SHOOT_DELAY_NORMAL 1000
#define SHOOT_DELAY_BUFF 200

typedef struct
{
    pid_struct_t pid;         // 拨盘speed的pid结构体
    pid_struct_t pid_angle;   // 单发angle的pid结构体
    pid_struct_t pid_speed;   // 单发speed的pid结构体
    float pid_value[3];       // 拨盘speed的pid参数
    float pid_angle_value[3]; // 单发angle的pid参数
    float pid_speed_value[3]; // 单发speed的pid参数
    float target_speed;       // 拨盘的目标速度
    float target_angle;       // 拨盘的目标角度
} trigger_t;

void Shoot_task(void const *argument);

#endif