#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"

typedef struct
{
    pid_struct_t pid;            // 摩擦轮speed的pid结构体
    fp32 pid_value[3];           // 摩擦轮speed的pid参数
    fp32 target_speed;           // 摩擦轮的目标速度
} shoot_t;

void Shoot_task(void const * argument);

// 初始化
void shoot_loop_init();

// 射击模式
void shoot_start();

// 停止射击模式
void shoot_stop();

// can2发送电流
void shoot_can2_cmd(int16_t v1, int16_t v2);

//PID计算速度并发送电流
void shoot_current_give();

#endif