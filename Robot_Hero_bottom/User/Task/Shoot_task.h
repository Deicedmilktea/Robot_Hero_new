#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "Chassis_task.h"
#include "pid.h"

typedef struct
{
    pid_struct_t pid_speed;   // 拨盘speed的pid结构体
    pid_struct_t pid_angle;   // 拨盘angle的pid结构体
    float pid_speed_value[3]; // 拨盘speed的pid参数
    float pid_angle_value[3]; // 拨盘angle的pid参数
    float target_speed;       // 拨盘的目标速度
    int16_t target_angle;     // 拨盘的目标角度
} trigger_t;

void Shoot_task(void const *argument);

#endif