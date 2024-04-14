#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    pid_struct_t pid_speed;          // speed的pid结构体
    pid_struct_t pid_angle;          // angle的pid结构体
    pid_struct_t vision_pid_speed;   // vision_speed的pid结构体
    pid_struct_t vision_pid_angle;   // vision_angle的pid结构体
    float speed_pid_value[3];        // speed的pid参数
    float angle_pid_value[3];        // speed的pid参数
    float vision_speed_pid_value[3]; // vision_speed的pid参数
    float vision_angle_pid_value[3]; // vision_angle的pid参数
    float target_speed;              // 目标速度
    float speed_max;                 // 最大速度
    float vision_manual_pitch;       // 视觉模式下的手动微调
    float vision_target_pitch;       // 视觉模式下的目标pitch
    float relative_pitch;            // 相对pitch
} pitch_t;

void Pitch_task(void const *argument);

#endif