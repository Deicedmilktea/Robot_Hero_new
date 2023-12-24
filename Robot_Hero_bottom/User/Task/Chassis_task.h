#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include  "drv_can.h"
#include "rc_potocal.h"
#include "main.h"

typedef struct
{
    uint16_t can_id;
    int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}motor_info_t;


typedef enum {
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;


//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 
void Chassis_task(void const *pvParameters);

//speed mapping
int16_t Speedmapping(int value, int from_min, int from_max, int to_min, int to_max);

void Calculate_speed();

void RC_move();

void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
void chassis_current_give(void);
#endif
