#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "main.h"
#include "pid.h"
#include "Robot.h"

typedef struct
{
    pid_struct_t pid_speed;   // speed的pid结构体
    pid_struct_t pid_angle;   // angle的pid结构体
    float speed_pid_value[3]; // speed的pid参数
    float angle_pid_value[3]; // speed的pid参数
    float target_angle;       // 目标速度
    float relative_pitch;     // 相对pitch
    float pid_angle_out;      // angle的pid输出
    float pid_speed_out;      // speed的pid输出
} pitch_t;

void Pitch_task(void const *argument);

#endif