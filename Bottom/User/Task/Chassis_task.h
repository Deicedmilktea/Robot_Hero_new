#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "remote_control.h"
#include "main.h"

typedef struct
{
    pid_struct_t pid;     // 底盘电机speed的pid结构体
    float pid_value[3];   // 底盘电机speed的pid参数
    int16_t target_speed; // 底盘电机的目标速度
    int16_t speed_max;    // 底盘电机的最大速度
} chassis_t;

void Chassis_task(void const *pvParameters);

#endif
