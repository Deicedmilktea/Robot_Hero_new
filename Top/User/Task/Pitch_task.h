#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "main.h"
#include "pid.h"
#include "Robot.h"

// #define PITCH_3508
#define PITCH_6020

#ifdef PITCH_3508

typedef struct
{
    pid_struct_t pid_speed;          // speed的pid结构体
    pid_struct_t vision_pid_angle;   // vision_angle的pid结构体
    pid_struct_t vision_pid_speed;   // vision_speed的pid结构体
    float speed_pid_value[3];        // speed的pid参数
    float vision_angle_pid_value[3]; // vision_angle的pid参数
    float vision_speed_pid_value[3]; // vision_speed的pid参数
    float target_speed;              // 目标速度
    float speed_max;                 // 最大速度
    float vision_remote_pitch;       // 视觉模式下的遥控器微调
    float vision_target_pitch;       // 视觉模式下的目标pitch
    float relative_pitch;            // 相对pitch
} pitch_t;

#endif // PITCH_3508

#ifdef PITCH_6020

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

#endif // PITCH_6020

void Pitch_task(void const *argument);

#endif