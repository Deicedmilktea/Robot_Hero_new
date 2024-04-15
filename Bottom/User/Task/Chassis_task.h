#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "remote_control.h"
#include "main.h"

typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    int16_t rotor_speed;
    int16_t torque_current;
    uint8_t temp;
    int16_t set_current;
    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} motor_info_t;

typedef struct
{
    pid_struct_t pid;     // 底盘电机speed的pid结构体
    float pid_value[3];   // 底盘电机speed的pid参数
    int16_t target_speed; // 底盘电机的目标速度
    int16_t speed_max;    // 底盘电机的最大速度
} chassis_t;

void Chassis_task(void const *pvParameters);

#endif
