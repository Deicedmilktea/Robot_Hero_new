#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    pid_struct_t pid_speed;  // speed的pid结构体
    pid_struct_t pid_angle;  // angle的pid结构体
    fp32 speed_pid_value[3]; // speed的pid参数
    fp32 angle_pid_value[3]; // speed的pid参数
    fp32 target_speed;       // 目标速度
    fp32 speed_max;          // 最大速度
} pitch_t;

void Pitch_task(void const *argument);

// 初始化
void pitch_loop_init();

/*can1发送电流*/
void pitch_can2_cmd(int16_t v3);

// PID计算速度并发送电流
void pitch_current_give();

// pitch速度映射
int16_t pitch_speed_map(int value, int from_min, int from_max, int to_min, int to_max);

// 判断pitch位置
void pitch_position_limit();

#endif