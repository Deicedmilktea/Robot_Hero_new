#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "Chassis_task.h"
#include "pid.h"

typedef struct
{
    pid_struct_t pid;  // 拨盘speed的pid结构体
    fp32 pid_value[3]; // 拨盘speed的pid参数
    fp32 target_speed; // 拨盘的目标速度
} shoot_t;

void Shoot_task(void const *argument);

#endif