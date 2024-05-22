#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"
#include "Robot.h"

#define FRICTION_MAX_SPEED 20000
#define FRICTION_UP_SPEED 1000
#define FRICTION_SPEED_NORMAL 5500
#define FRICTION_SPEED_LOW 6000
#define FRICTION_SPEED_HIGH 6500
#define FRICTION_SPEED_STOP 0
#define FRICTION_UP_SPEED_STOP 0

#define LENS_ANGLE_ON -45
#define LENS_ANGLE_OFF -2000
#define LENS_UP_ANGLE 2200 // 初始化-200，开镜2000

#define LENS_ANGLE_HIGH -520
#define LENS_ANGLE_LOW 310
#define LENS_DOWN_ANGLE 500 // 初始化200，低头700

typedef struct
{
    pid_struct_t pid;   // 摩擦轮speed的pid结构体
    float pid_value[3]; // 摩擦轮speed的pid参数
    float target_speed; // 摩擦轮的目标速度
} shoot_t;

typedef struct
{
    pid_struct_t pid;   // speed的pid结构体
    float pid_value[3]; // speed的pid参数
    float target_angle; // 目标速度
    int16_t init_angle; // 初始角度
} lens_t;

typedef enum
{
    FRICTION_NORMAL, // 摩擦轮1档
    FRICTION_LOW,
    FRICTION_HIGH,
    FRICTION_STOP,
} friction_mode_e;

typedef enum
{
    GIMBAL_OFF = 0,
    GIMBAL_ON,
} is_gimbal_on_t;

void Shoot_task(void const *argument);

#endif