#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include "Robot.h"

typedef struct
{
    pid_struct_t pid_angle;          // 云台电机angle的pid结构体
    pid_struct_t pid_speed;          // 云台电机speed的pid结构体
    pid_struct_t pid_angle_vision;   // 云台电机vision_angle的pid结构体
    pid_struct_t pid_speed_vision;   // 云台电机vision_speed的pid结构体
    float pid_angle_value[3];        // 云台电机angle的pid参数
    float pid_speed_value[3];        // 云台电机speed的pid参数
    float pid_angle_vision_value[3]; // 云台电机vision_angle的pid参数
    float pid_speed_vision_value[3]; // 云台电机vision_speed的pid参数
    float target_angle;              // 云台电机的目标角度
    float pid_angle_out;             // pid angle输出
    float pid_speed_out;             // pid speed输出
    float init_angle;                // 云台电机的初始角度
    float err_yaw;                   // 上下yaw值差
} gimbal_t;

void Gimbal_task(void const *pvParameters);

#endif