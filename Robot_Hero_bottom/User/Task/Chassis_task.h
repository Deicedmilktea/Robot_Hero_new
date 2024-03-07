#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"

typedef struct
{
    uint16_t can_id;
    int16_t set_current;
    uint16_t rotor_angle;
    int16_t rotor_speed;
    int16_t torque_current;
    uint8_t temp;
} motor_info_t;

typedef struct
{
    pid_struct_t pid;     // 云台电机speed的pid结构体
    fp32 pid_value[3];    // 云台电机speed的pid参数
    int16_t target_speed; // 云台电机的目标速度
} chassis_t;

void Chassis_task(void const *pvParameters);

#endif
