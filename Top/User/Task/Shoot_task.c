/*
**********Shoot_task射击任务**********
包含对三个摩擦轮以及开镜电机的控制
摩擦轮分别为3508, ID = 1(left) ID = 2(right) ID = 3(up) ID = 4(lens_up开关镜), ID = 5(lens_down图传上下), CAN2 控制
遥控器控制：左边拨杆，拨到中间启动摩擦轮(lr)，最上面发射(up) (从上到下分别为132)
键鼠控制：默认开启摩擦轮(lr)，左键发射(up)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control.h"
#include "video_control.h"
#include "stdlib.h"
#include "ins_task.h"

motor_info_t motor_top[7]; //[0]-[2]:left, right, up, [3]:lens_up, [4]:lens_down, [5]:pitch, [6]:yaw
uint8_t friction_flag = 0; // 摩擦轮转速标志位，012分别为low, normal, high, 默认为normal
uint8_t video_mode;        // 图传模式
uint8_t is_friction_on;    // 摩擦轮是否开启

static shoot_t shoot_motor[3]; // 摩擦轮can2，id = 12
static lens_t lens_motor[2];   // 开镜can2，up,down,id = 45
static int16_t friction_speed = 0;
static int16_t friction_up_speed = 0;
static uint8_t is_lens_ready = 0; // 开镜是否到达机械限位

extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern uint8_t is_remote_online;
extern INS_t INS;

static void shoot_loop_init();                                                            // 初始化
static void read_keyboard();                                                              // 读取摩擦轮速度
static void shoot_start_lr();                                                             // 左右摩擦轮开启模式
static void shoot_start_remote();                                                         // 遥控器三摩擦轮开启模式
static void shoot_start_mouse();                                                          // 键鼠三摩擦轮开启模式
static void shoot_stop();                                                                 // 摩擦轮关闭模式
static void lens_judge();                                                                 // 判断开关镜
static void shoot_can2_cmd(uint8_t mode, int16_t v1, int16_t v2, int16_t v3, int16_t v4); // can2发送电流
static void shoot_current_give();                                                         // PID计算速度并发送电流
static void read_keyboard();                                                              // 读取键鼠是否开启摩擦轮
static void lens_prepare();                                                               // 开镜准备
static void video_adaptive();                                                             // 图传自适应调节
static void read_friction_on();                                                           // 摩擦轮是否开启

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
    read_keyboard();
    read_friction_on();
    lens_prepare();
    video_adaptive();
    lens_judge();

    // 遥控器链路
    if (is_remote_online)
    {
      // 右拨杆中，键鼠控制
      if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right))
      {
        shoot_start_mouse();
      }

      // 右拨杆下，遥控器控制
      else
      {
        // 遥控器左边拨到上和中，电机启动
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_left) || switch_is_mid(rc_ctrl[TEMP].rc.switch_left))
        {
          friction_speed = FRICTION_SPEED_NORMAL;
          friction_up_speed = -FRICTION_UP_SPEED;
          shoot_start_remote();
        }
        else
        {
          shoot_stop();
        }
      }
    }

    // 图传链路
    else
    {
      shoot_start_mouse();
    }

    shoot_current_give();
    osDelay(1);
  }
}

/***************初始化***************/
static void shoot_loop_init()
{
  // friction_left
  shoot_motor[0].pid_value[0] = 20;
  shoot_motor[0].pid_value[1] = 0;
  shoot_motor[0].pid_value[2] = 0;

  // friction_right
  shoot_motor[1].pid_value[0] = 20;
  shoot_motor[1].pid_value[1] = 0;
  shoot_motor[1].pid_value[2] = 0;

  // friction_up
  shoot_motor[2].pid_value[0] = 20;
  shoot_motor[2].pid_value[1] = 0;
  shoot_motor[2].pid_value[2] = 0;

  // lens_up
  lens_motor[0].pid_value[0] = 10;
  lens_motor[0].pid_value[1] = 0;
  lens_motor[0].pid_value[2] = 600;

  // lens_down
  lens_motor[1].pid_value[0] = 30;
  lens_motor[1].pid_value[1] = 0.05;
  lens_motor[1].pid_value[2] = 600;

  // lens_up_speed
  lens_motor[0].pid_speed_value[0] = 10;
  lens_motor[0].pid_speed_value[1] = 0.01;
  lens_motor[0].pid_speed_value[2] = 0;

  // lens_down_speed
  lens_motor[1].pid_speed_value[0] = 10;
  lens_motor[1].pid_speed_value[1] = 0.01;
  lens_motor[1].pid_speed_value[2] = 0;

  // 初始化目标速度
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;

  // 初始化PID
  pid_init(&shoot_motor[0].pid, shoot_motor[0].pid_value, 1000, FRICTION_MAX_SPEED); // friction_right
  pid_init(&shoot_motor[1].pid, shoot_motor[1].pid_value, 1000, FRICTION_MAX_SPEED); // friction_left
  pid_init(&shoot_motor[2].pid, shoot_motor[2].pid_value, 1000, 4000);               // friction_up

  pid_init(&lens_motor[0].pid, lens_motor[0].pid_value, 4000, 8000); // lens_up
  pid_init(&lens_motor[1].pid, lens_motor[1].pid_value, 4000, 8000); // lens_down

  pid_init(&lens_motor[0].pid_speed, lens_motor[0].pid_speed_value, 2000, 2000); // lens_up
  pid_init(&lens_motor[1].pid_speed, lens_motor[1].pid_speed_value, 2000, 2000); // lens_down
}

// 读取摩擦轮速度
static void read_keyboard()
{
  // 遥控器链路
  if (is_remote_online)
  {
    // E键切换摩擦轮速度
    switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 4)
    {
    case 1:
      friction_speed = FRICTION_SPEED_NORMAL;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_NORMAL;
      break;
    case 2:
      friction_speed = FRICTION_SPEED_LOW;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_LOW;
      break;
    case 3:
      friction_speed = FRICTION_SPEED_HIGH;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_HIGH;
      break;
    default:
      friction_speed = FRICTION_SPEED_STOP;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_STOP;
      break;
    }
  }

  // 图传链路
  else
  {
    // E键切换摩擦轮速度
    switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 4)
    {
    case 1:
      friction_speed = FRICTION_SPEED_NORMAL;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_NORMAL;
      break;
    case 2:
      friction_speed = FRICTION_SPEED_LOW;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_LOW;
      break;
    case 3:
      friction_speed = FRICTION_SPEED_HIGH;
      friction_up_speed = FRICTION_UP_SPEED;
      friction_flag = FRICTION_HIGH;
      break;
    default:
      friction_speed = FRICTION_SPEED_STOP;
      // friction_up_speed = FRICTION_UP_SPEED;
      friction_up_speed = FRICTION_SPEED_STOP;
      friction_flag = FRICTION_STOP;
      break;
    }
  }
}

/*************** 左右摩擦轮开启模式 *****************/
static void shoot_start_lr()
{
  shoot_motor[0].target_speed = -friction_speed;
  shoot_motor[1].target_speed = friction_speed;
  shoot_motor[2].target_speed = 0;
}

/**************** 键鼠三摩擦轮开启模式 *****************/
static void shoot_start_mouse()
{
  shoot_motor[0].target_speed = -friction_speed;
  shoot_motor[1].target_speed = friction_speed;
  // if (rc_ctrl[TEMP].mouse.press_l || video_ctrl[TEMP].key_data.left_button_down)
  shoot_motor[2].target_speed = -friction_up_speed;
  // else
  // shoot_motor[2].target_speed = 4 * friction_up_speed;
}

/**************** 遥控器三摩擦轮开启模式 *****************/
static void shoot_start_remote()
{
  shoot_motor[0].target_speed = -friction_speed;
  shoot_motor[1].target_speed = friction_speed;
  if (!friction_speed)
    shoot_motor[2].target_speed = -friction_up_speed;
  else
    shoot_motor[2].target_speed = friction_up_speed;
}

/*************** 摩擦轮关闭模式 **************/
static void shoot_stop()
{
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;
}

/***************** 判断开关镜 *******************/
static void lens_judge()
{
  // 遥控器链路
  if (is_remote_online)
  {
    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_G] % 2 == 1)
      lens_motor[0].target_angle = lens_motor[0].init_angle + LENS_UP_MOVE_ANGLE;
    else
      lens_motor[0].target_angle = lens_motor[0].init_angle;

    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 2 == 1)
      video_mode = VIDEO_ADAPTIVE;
    else
      video_mode = VIDEO_NORMAL;
  }

  // 图传链路
  else
  {
    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_G] % 2 == 1)
      lens_motor[0].target_angle = lens_motor[0].init_angle + LENS_UP_MOVE_ANGLE;
    else
      lens_motor[0].target_angle = lens_motor[0].init_angle;

    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 2 == 1)
      video_mode = VIDEO_ADAPTIVE;
    else
      video_mode = VIDEO_NORMAL;
  }
}

void lens_prepare()
{
  if (!is_lens_ready)
  {
    lens_motor[0].target_speed = -1000;
    lens_motor[1].target_speed = -1000;

    if ((motor_top[3].torque_current < -1500) && (motor_top[4].torque_current < -1500))
    {
      is_lens_ready = 1;
      lens_motor[0].init_angle = motor_top[3].total_angle;
      lens_motor[1].init_angle = motor_top[4].total_angle;
    }
  }
}

void video_adaptive()
{
  if (is_lens_ready)
  {
    if (video_mode == VIDEO_ADAPTIVE)
    {
      // 调节角度
      int16_t adptive_angle = 36 * (INS.Roll + 6);
      lens_motor[1].target_angle = lens_motor[1].init_angle + adptive_angle;
    }

    else
    {
      lens_motor[1].target_angle = lens_motor[1].init_angle;
    }
  }
}

void read_friction_on()
{
  if (friction_speed)
    is_friction_on = 1;
  else
    is_friction_on = 0;
}

/********************************摩擦轮can2发送电流***************************/
static void shoot_can2_cmd(uint8_t mode, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = (mode == 0) ? 0x200 : 0x1FF;
  tx_header.IDE = CAN_ID_STD;   // 标准帧
  tx_header.RTR = CAN_RTR_DATA; // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff;
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/********************************PID计算速度并发送电流****************************/
static void shoot_current_give()
{
  motor_top[0].set_current = pid_calc(&shoot_motor[0].pid, shoot_motor[0].target_speed, motor_top[0].rotor_speed);
  motor_top[1].set_current = pid_calc(&shoot_motor[1].pid, shoot_motor[1].target_speed, motor_top[1].rotor_speed);
  motor_top[2].set_current = pid_calc(&shoot_motor[2].pid, shoot_motor[2].target_speed, motor_top[2].rotor_speed);

  if (!is_lens_ready)
  {
    motor_top[3].set_current = pid_calc(&lens_motor[0].pid_speed, lens_motor[0].target_speed, motor_top[3].rotor_speed);
    motor_top[4].set_current = pid_calc(&lens_motor[1].pid_speed, lens_motor[1].target_speed, motor_top[4].rotor_speed);
  }

  else
  {
    motor_top[3].set_current = pid_calc(&lens_motor[0].pid, lens_motor[0].target_angle, motor_top[3].total_angle);
    motor_top[4].set_current = pid_calc(&lens_motor[1].pid, lens_motor[1].target_angle, motor_top[4].total_angle);
  }

  shoot_can2_cmd(0, motor_top[0].set_current, motor_top[1].set_current, motor_top[2].set_current, motor_top[3].set_current);
  shoot_can2_cmd(1, motor_top[4].set_current, motor_top[5].set_current, 0, 0);
}
