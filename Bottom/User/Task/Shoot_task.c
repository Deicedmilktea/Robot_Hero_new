/*
**********Shoot_task射击任务**********
包含对拨盘的控制
拨盘为3508，ID = 5，CAN2控制, motor_can2[4]
遥控器左边拨杆控制，拨到最上面启动 (从上到下分别为132)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include <stdbool.h>

#define TRIGGER_SINGLE_ANGLE 1140 // 19*360/6

trigger_t trigger; // 拨盘can1，id = 5

bool is_angle_control = false;
float current_time = 0;
float last_time = 0;

extern RC_ctrl_t rc_ctrl;
extern motor_info_t motor_can2[6];

// 初始化
static void shoot_loop_init();

// 射击模式
static void shoot_start();

// 反转
static void shoot_reverse();

// 停止射击模式
static void shoot_stop();

// 拨盘旋转固定角度
static void trigger_single_angle_move();

// 拨盘can1发送电流
static void trigger_can2_cmd(int16_t v1);

// PID计算速度并发送电流
static void shoot_current_give();

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
    // 遥控器左边拨到上，电机启动
    if (rc_ctrl.rc.s[1] == 1)
    {
      is_angle_control = false;
      shoot_start();
    }
    // 单发，鼠标控制
    else if (press_left)
    {
      is_angle_control = true;
      trigger_single_angle_move();
    }
    else if (z_flag)
    {
      is_angle_control = false;
      shoot_reverse();
    }
    else
    {
      shoot_stop();
    }

    shoot_current_give();
    osDelay(1);
  }
}

/***************初始化***************/
static void shoot_loop_init()
{
  // trigger
  // trigger.pid_value[0] = 50;
  // trigger.pid_value[1] = 1;
  // trigger.pid_value[2] = 0.05;

  trigger.pid_speed_value[0] = 200;
  trigger.pid_speed_value[1] = 0;
  trigger.pid_speed_value[2] = 0;

  trigger.pid_angle_value[0] = 2;
  trigger.pid_angle_value[1] = 0;
  trigger.pid_angle_value[2] = 0;

  // 初始化目标速度
  trigger.target_speed = 0;
  trigger.target_angle = motor_can2[4].total_angle;

  // 初始化PID
  pid_init(&trigger.pid_speed, trigger.pid_speed_value, 8000, 8000); // trigger_speed
  pid_init(&trigger.pid_angle, trigger.pid_angle_value, 8000, 8000); // trigger_angle
}

/***************射击模式*****************/
static void shoot_start()
{
  trigger.target_speed = -250;
}

/*************拨盘旋转固定角度***********/
static void trigger_single_angle_move()
{
  current_time = DWT_GetTimeline_ms();
  // 判断两次发射时间间隔，避免双发
  if (current_time - last_time > 1000)
  {
    last_time = DWT_GetTimeline_ms();
    trigger.target_angle = motor_can2[4].total_angle - TRIGGER_SINGLE_ANGLE;
  }
}

/*****************反转******************/
static void shoot_reverse()
{
  trigger.target_speed = 250;
}

/***************停止射击模式**************/
static void shoot_stop()
{
  if (!is_angle_control)
    trigger.target_speed = 0;
}

/********************************拨盘can1发送电流***************************/
static void trigger_can2_cmd(int16_t v1)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = 0x1FF;
  tx_header.IDE = CAN_ID_STD;   // 标准帧
  tx_header.RTR = CAN_RTR_DATA; // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff;
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = NULL;
  tx_data[3] = NULL;
  tx_data[4] = NULL;
  tx_data[5] = NULL;
  tx_data[6] = NULL;
  tx_data[7] = NULL;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/********************************PID计算速度并发送电流****************************/
static void shoot_current_give()
{
  if (is_angle_control)
    motor_can2[4].set_current = pid_calc_trigger(&trigger.pid_angle, trigger.target_angle, motor_can2[4].total_angle);
  else
    motor_can2[4].set_current = pid_calc(&trigger.pid_speed, trigger.target_speed, motor_can2[4].rotor_speed);

  // trigger_can2_cmd(motor_can2[4].set_current);
}
