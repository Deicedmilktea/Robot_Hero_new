#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"
#include "Robot.h"

typedef struct
{
    pid_struct_t pid;  // 摩擦轮speed的pid结构体
    fp32 pid_value[3]; // 摩擦轮speed的pid参数
    fp32 target_speed; // 摩擦轮的目标速度
} shoot_t;

void Shoot_task(void const *argument);

#endif