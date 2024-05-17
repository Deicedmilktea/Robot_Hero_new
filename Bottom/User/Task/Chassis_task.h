#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "remote_control.h"
#include "main.h"

#define CHASSIS_SPEED_MAX_13 7000
#define CHASSIS_SPEED_MAX_2 7500
#define CHASSIS_SPEED_MAX_3 8500
#define CHASSIS_SPEED_MAX_46 9500
#define CHASSIS_SPEED_MAX_5 10000
#define CHASSIS_SPEED_MAX_6 10000
#define CHASSIS_SPEED_MAX_7 10000
#define CHASSIS_SPEED_MAX_8 10000
#define CHASSIS_SPEED_MAX_9 10000
#define CHASSIS_SPEED_MAX_710 12000
#define CHASSIS_SPEED_SUPERCAP 10000
#define CHASSIS_WZ_MAX_1 4000 // 低速，g键触发
#define CHASSIS_WZ_MAX_2 6000 // 高速，b键触发
#define KEY_START_OFFSET 20
#define KEY_STOP_OFFSET 30
#define FOLLOW_WEIGHT 160

typedef struct
{
    pid_struct_t pid;     // 底盘电机speed的pid结构体
    float pid_value[3];   // 底盘电机speed的pid参数
    int16_t target_speed; // 底盘电机的目标速度
    int16_t speed_max;    // 底盘电机的最大速度
} chassis_t;

void Chassis_task(void const *pvParameters);

#endif
