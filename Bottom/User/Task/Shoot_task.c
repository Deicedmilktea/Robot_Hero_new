/*
**********Shoot_task射击任务**********
包含对拨盘的控制
拨盘为3508，ID = 5，CAN2控制, motor_bottom[4]
遥控器左边拨杆控制，拨到最上面启动 (从上到下分别为132)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "remote_control.h"
#include "video_control.h"
#include "Robot.h"

#define TRIGGER_SINGLE_ANGLE 1178 // 19*62=1178
#define TRIGGER_ROTATE_SPEED 650

static trigger_t trigger; // 拨盘can1，id = 5
static bool is_angle_control = false;
static float current_time = 0;
static float last_time = 0;
static int16_t shoot_delay;

uint8_t trigger_flag = 0;

extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern motor_info_t motor_bottom[5];
extern CAN_HandleTypeDef hcan2;
extern uint8_t is_remote_online;
extern uint8_t is_friction_on;

static void shoot_loop_init();            // 初始化
static void shoot_start();                // 射击模式
static void read_keyboard();              // 读取键盘
static void shoot_reverse();              // 反转
static void shoot_stop();                 // 停止射击模式
static void trigger_single_angle_move();  // 拨盘旋转固定角度
static void trigger_can2_cmd(int16_t v1); // 拨盘can1发送电流
static void shoot_current_give();         // PID计算速度并发送电流

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
    read_keyboard();

    // 遥控器链路
    if (is_remote_online)
    {
      // 右拨杆下，遥控器控制
      if (switch_is_down(rc_ctrl[TEMP].rc.switch_right) || switch_is_up(rc_ctrl[TEMP].rc.switch_right))
      {
        // 遥控器左边拨到上，电机启动
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_left))
        {
          is_angle_control = false;
          shoot_start();
        }
        else
        {
          is_angle_control = false;
          shoot_stop();
        }
      }

      // 右拨杆中，键鼠控制
      else if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right))
      {
        // 鼠标左键按下，控制拨盘旋转固定角度
        if (rc_ctrl[TEMP].mouse.press_l)
        {
          if (is_friction_on)
          {
            if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl)
            {
              is_angle_control = false;
              shoot_start();
            }
            else
            {
              is_angle_control = true;
              trigger_single_angle_move();
            }
          }
        }

        // z键按下，反转
        else if (rc_ctrl[TEMP].key[KEY_PRESS].z)
        {
          is_angle_control = false;
          shoot_reverse();
        }

        else
        {
          shoot_stop();
        }
      }

      else
      {
        shoot_stop();
      }
    }

    // 图传链路
    else
    {
      // 鼠标左键按下，控制拨盘旋转固定角度
      if (video_ctrl[TEMP].key_data.left_button_down)
      {
        if (is_friction_on)
        {
          if (video_ctrl[TEMP].key[KEY_PRESS].ctrl)
          {
            is_angle_control = false;
            shoot_start();
          }
          else
          {
            is_angle_control = true;
            trigger_single_angle_move();
          }
        }
      }

      // z键按下，反转
      else if (video_ctrl[TEMP].key[KEY_PRESS].z)
      {
        is_angle_control = false;
        shoot_reverse();
      }

      else
      {
        shoot_stop();
      }
    }

    shoot_current_give();
    osDelay(1);
  }
}

/***************初始化***************/
static void shoot_loop_init()
{
  trigger.pid_value[0] = 40;
  trigger.pid_value[1] = 0.3;
  trigger.pid_value[2] = 0;

  trigger.pid_angle_value[0] = 20;
  trigger.pid_angle_value[1] = 0;
  trigger.pid_angle_value[2] = 500;

  trigger.pid_speed_value[0] = 10;
  trigger.pid_speed_value[1] = 0;
  trigger.pid_speed_value[2] = 0;

  // 初始化目标速度
  trigger.target_speed = 0;
  trigger.target_angle = motor_bottom[4].total_angle;

  // 初始化PID
  pid_init(&trigger.pid, trigger.pid_value, 30000, 30000);            // trigger_speed
  pid_init(&trigger.pid_angle, trigger.pid_angle_value, 3000, 12000); // trigger_angle
  pid_init(&trigger.pid_speed, trigger.pid_speed_value, 3000, 12000);
}

/***************射击模式*****************/
static void shoot_start()
{
  trigger.target_speed = -TRIGGER_ROTATE_SPEED;
}

// 读取键盘
static void read_keyboard()
{
  // 遥控器链路
  if (is_remote_online)
  {
    // Q键切换发射模式，单发和爆破
    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2 == 1)
    {
      trigger_flag = 1;
      shoot_delay = SHOOT_DELAY_BUFF;
    }
    else
    {
      trigger_flag = 0;
      shoot_delay = SHOOT_DELAY_NORMAL;
    }

    // R键切换发射模式，速度环和位置环
  }

  // 图传链路
  else
  {
    // Q键切换发射模式，单发和爆破
    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2 == 1)
    {
      trigger_flag = 1;
      shoot_delay = SHOOT_DELAY_BUFF;
    }
    else
    {
      trigger_flag = 0;
      shoot_delay = SHOOT_DELAY_NORMAL;
    }
  }
}

/*************拨盘旋转固定角度***********/
static void trigger_single_angle_move()
{
  current_time = DWT_GetTimeline_ms();
  // 判断两次发射时间间隔，避免双发
  if (current_time - last_time > shoot_delay)
  {
    last_time = DWT_GetTimeline_ms();
    trigger.target_angle = motor_bottom[4].total_angle - TRIGGER_SINGLE_ANGLE;
  }
}

/*****************反转******************/
static void shoot_reverse()
{
  trigger.target_speed = TRIGGER_ROTATE_SPEED;
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
  {
    trigger.target_speed = pid_calc_trigger(&trigger.pid_angle, trigger.target_angle, motor_bottom[4].total_angle);
    motor_bottom[4].set_current = pid_calc(&trigger.pid_speed, trigger.target_speed, motor_bottom[4].rotor_speed);
  }
  else
    motor_bottom[4].set_current = pid_calc(&trigger.pid_speed, trigger.target_speed, motor_bottom[4].rotor_speed);

  trigger_can2_cmd(motor_bottom[4].set_current);
}