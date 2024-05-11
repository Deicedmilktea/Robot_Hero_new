#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"
#include "Robot.h"

typedef struct
{
    pid_struct_t pid;   // 摩擦轮speed的pid结构体
    float pid_value[3]; // 摩擦轮speed的pid参数
    float target_speed; // 摩擦轮的目标速度
} shoot_t;

typedef struct
{
    pid_struct_t pid;   // 摩擦轮speed的pid结构体
    float pid_value[3]; // 摩擦轮speed的pid参数
    float target_angle; // 摩擦轮的目标速度
} lens_t;

typedef enum
{
    FRICTION_NORMAL, // 摩擦轮1档
    FRICTION_LOW,
    FRICTION_HIGH,
    FRICTION_STOP,
} friction_mode_e;

void Shoot_task(void const *argument);

#endif