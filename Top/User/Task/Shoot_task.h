#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "pid.h"
#include "Robot.h"

#define FRICTION_MAX_SPEED 20000
#define FRICTION_UP_SPEED 2000
#define FRICTION_SPEED_NORMAL 5200
#define FRICTION_SPEED_LOW 5000
#define FRICTION_SPEED_HIGH 5400
#define FRICTION_SPEED_STOP 0
#define FRICTION_UP_SPEED_STOP 0

#define LENS_UP_MOVE_ANGLE 1950
#define LENS_DOWN_MOVE_ANGLE 1400

typedef struct
{
    pid_struct_t pid;   // 摩擦轮speed的pid结构体
    float pid_value[3]; // 摩擦轮speed的pid参数
    float target_speed; // 摩擦轮的目标速度
} shoot_t;

typedef struct
{
    pid_struct_t pid;         // angle的pid结构体
    pid_struct_t pid_speed;   // speed的pid结构体
    float pid_value[3];       // angle的pid参数
    float pid_speed_value[3]; // speed的pid参数
    int16_t target_angle;     // 目标角度
    float target_speed;       // 目标速度
    int16_t init_angle;       // 初始角度
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
} is_gimbal_on_e;

typedef enum
{
    VIDEO_NORMAL = 0,
    VIDEO_ADAPTIVE,
} video_mode_e;

void Shoot_task(void const *argument);

#endif