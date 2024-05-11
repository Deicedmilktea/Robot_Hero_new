#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "pid.h"
#include "Robot.h"
typedef struct
{
    pid_struct_t pid_angle;   // angle的pid结构体
    pid_struct_t pid_speed;   // speed的pid结构体
    float angle_pid_value[3]; // speed的pid参数
    float speed_pid_value[3]; // speed的pid参数
    float target_angle;       // 目标角度
    float target_speed;       // 目标速度
    float relative_pitch;     // 相对pitch
} pitch_t;

void Pitch_task(void const *argument);

#endif