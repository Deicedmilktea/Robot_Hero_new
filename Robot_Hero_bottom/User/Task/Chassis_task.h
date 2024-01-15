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
    pid_struct_t pid;  // 云台电机speed的pid结构体
    fp32 pid_value[3]; // 云台电机speed的pid参数
    fp32 target_speed; // 云台电机的目标速度
} chassis_t;

// 获取imu——Yaw角度差值参数
static void Get_Err();

// 参数重置
static void Chassis_loop_Init();
void Chassis_task(void const *pvParameters);

// speed mapping
int16_t Speedmapping(int value, int from_min, int from_max, int to_min, int to_max);

/***************************************正常运动模式************************************/
void chassis_mode_normal();

/******************************小陀螺模式*********************************/
void chassis_mode_top();

/*****************************底盘跟随云台模式*******************************/
void chassis_mode_follow();

// 电机电流控制
void chassis_current_give(void);

// chassis CAN2发送信号
void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif
