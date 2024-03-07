#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include "main.h"

typedef struct
{
    motor_info_t motor_info; // 电机信息结构体
    pid_struct_t pid_angle;  // 云台电机angle的pid结构体
    pid_struct_t pid_speed;  // 云台电机speed的pid结构体
    fp32 pid_angle_value[3]; // 云台电机angle的pid参数
    fp32 pid_speed_value[3]; // 云台电机speed的pid参数
    fp32 target_angle;       // 云台电机的目标角度
    float pid_angle_out;     // pid angle输出
    float pid_speed_out;     // pid speed输出
    fp32 init_angle;         // 云台电机的初始角度
} gimbal_t;

void Gimbal_task(void const *pvParameters);

#endif