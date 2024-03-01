/*
*************Chassis_task底盘任务**************
采用3508，CAN2，ID = 1234
遥控器控制：左拨杆上下→前后
           左拨杆左右→左右
           左滑轮→旋转
*/

#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

#define KEY_START_OFFSET 10
#define KEY_STOP_OFFSET 20
#define CHASSIS_SPEED_MAX 2000
#define CHASSIS_TOP_SPEED 1000

motor_info_t motor_can2[6]; // can2电机信息结构体, 0123：底盘，4：拨盘, 5: 云台
chassis_t chassis[4];
volatile int16_t Vx = 0, Vy = 0, Wz = 0;
int fllowflag = 0;
float relative_yaw = 0;
int yaw_correction_flag = 1; // yaw值校正标志

extern RC_ctrl_t rc_ctrl;
extern INS_t INS;
extern INS_t INS_top;
extern uint16_t shift_flag;

// 底盘跟随云台计算
static pid_struct_t pid_yaw_angle;
static pid_struct_t pid_yaw_speed;
static float pid_yaw_angle_value[3];
static float pid_yaw_speed_value[3];

int error10 = 0;
fp32 speed10 = 0;

double rx = 0.2, ry = 0.2;
// Save imu data

int16_t chassis_mode = 1; // 判断底盘状态，用于UI编写

int chassis_mode_flag = 0;

void Chassis_task(void const *pvParameters)
{

  Chassis_loop_Init();

  for (;;)
  {
    // 校正yaw值
    yaw_correct();

    // 左拨杆拨到上，小陀螺模式
    if (rc_ctrl.rc.s[1] == 1 || shift_flag == 1)
    {
      chassis_mode_top();
    }

    // 底盘跟随云台模式
    else
    {
      chassis_mode_follow();
    }

    chassis_current_give();
    error10++;
    osDelay(1);
  }
}

static void Chassis_loop_Init()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis[i].pid_value[0] = 30;
    chassis[i].pid_value[1] = 0.5;
    chassis[i].pid_value[2] = 0;
  }

  // 底盘跟随云台
  pid_yaw_angle_value[0] = 1;
  pid_yaw_angle_value[1] = 0;
  pid_yaw_angle_value[2] = 0;

  pid_yaw_speed_value[0] = 10;
  pid_yaw_speed_value[1] = 0;
  pid_yaw_speed_value[2] = 0;

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis[i].pid, chassis[i].pid_value, 6000, 6000);
  }

  // 底盘跟随云台
  pid_init(&pid_yaw_angle, pid_yaw_angle_value, 6000, 1000);
  pid_init(&pid_yaw_speed, pid_yaw_speed_value, 6000, 2000);

  Vx = 0;
  Vy = 0;
  Wz = 0;
}

/**********************************mapping****************************************/
int16_t mapping(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值从 [a, b] 映射到 [0, 1] 范围内
  double normalized_value = (value * 1.0 - from_min) / (from_max - from_min);

  // 然后将标准化后的值映射到 [C, D] 范围内
  int16_t mapped_value = (int16_t)(to_min + (to_max - to_min) * normalized_value);

  return mapped_value;
}

/***************************************正常运动模式************************************/
void chassis_mode_normal()
{
  Vx = mapping(rc_ctrl.rc.ch[2], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // left and right
  Vy = mapping(rc_ctrl.rc.ch[3], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // front and back
  Wz = mapping(rc_ctrl.rc.ch[4], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // rotate

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;

  relative_yaw = INS.yaw_update - INS_top.Yaw;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  chassis[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  chassis[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);
}

/******************************小陀螺模式*********************************/
void chassis_mode_top()
{
  // remote control
  if (!w_flag && !s_flag && !a_flag && !d_flag)
  {
    Vx = mapping(rc_ctrl.rc.ch[2], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // left and right
    Vy = mapping(rc_ctrl.rc.ch[3], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // front and back
  }
  // keyboard control
  else
  {
    if (w_flag)
      Vy += KEY_START_OFFSET;
    else if (s_flag)
      Vy -= KEY_STOP_OFFSET;
    else
      Vy = 0;

    if (Vy > CHASSIS_SPEED_MAX)
      Vy = CHASSIS_SPEED_MAX;
    if (Vy < -CHASSIS_SPEED_MAX)
      Vy = -CHASSIS_SPEED_MAX;

    if (a_flag)
      Vx -= KEY_STOP_OFFSET;
    else if (d_flag)
      Vx += KEY_START_OFFSET;
    else
      Vx = 0;

    if (Vx > CHASSIS_SPEED_MAX)
      Vx = CHASSIS_SPEED_MAX;
    if (Vx < -CHASSIS_SPEED_MAX)
      Vx = -CHASSIS_SPEED_MAX;
  }

  Wz = CHASSIS_TOP_SPEED;

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;

  relative_yaw = INS.yaw_update - INS_top.Yaw;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  chassis[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  chassis[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);
}

/*****************************底盘跟随云台模式*******************************/
void chassis_mode_follow()
{
  // remote control
  if (!w_flag && !s_flag && !a_flag && !d_flag)
  {
    Vx = mapping(rc_ctrl.rc.ch[2], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // left and right
    Vy = mapping(rc_ctrl.rc.ch[3], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // front and back
  }
  // keyboard control
  else
  {
    if (w_flag)
      Vy += KEY_START_OFFSET;
    else if (s_flag)
      Vy -= KEY_STOP_OFFSET;
    else
      Vy = 0;

    if (Vy > CHASSIS_SPEED_MAX)
      Vy = CHASSIS_SPEED_MAX;
    if (Vy < -CHASSIS_SPEED_MAX)
      Vy = -CHASSIS_SPEED_MAX;

    if (a_flag)
      Vx -= KEY_STOP_OFFSET;
    else if (d_flag)
      Vx += KEY_START_OFFSET;
    else
      Vx = 0;

    if (Vx > CHASSIS_SPEED_MAX)
      Vx = CHASSIS_SPEED_MAX;
    if (Vx < -CHASSIS_SPEED_MAX)
      Vx = -CHASSIS_SPEED_MAX;
  }

  relative_yaw = INS.yaw_update - INS_top.Yaw;
  int16_t yaw_speed = pid_calc_a(&pid_yaw_angle, 0, relative_yaw);
  int16_t rotate_w = (motor_can2[0].rotor_speed + motor_can2[1].rotor_speed + motor_can2[2].rotor_speed + motor_can2[3].rotor_speed) / (4 * 19);
  // 消除静态旋转
  if (relative_yaw > -2 && relative_yaw < 2)
  {
    Wz = 0;
  }
  else
  {
    Wz = pid_calc(&pid_yaw_speed, yaw_speed, rotate_w);
  }

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反
  Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  // chassis[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  // chassis[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  // chassis[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  // chassis[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);

  chassis[0].target_speed = 0;
  chassis[1].target_speed = 0;
  chassis[2].target_speed = 0;
  chassis[3].target_speed = 0;
}

/*************************** 视觉运动模式 ****************************/
void chassis_mode_vision()
{
  // int16_t Temp_Vx = vision_Vx;
  // int16_t Temp_Vy = vision_Vy;
  // Wz = mapping(rc_ctrl.rc.ch[4], -660, 660, -CHASSIS_SPEED_MAX, CHASSIS_SPEED_MAX); // rotate

  relative_yaw = INS.yaw_update - INS_top.Yaw;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  // Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  // Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  chassis[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  chassis[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);
}

/*************************** 电机电流控制 ****************************/
void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    motor_can2[i].set_current = pid_calc(&chassis[i].pid, chassis[i].target_speed, motor_can2[i].rotor_speed);
  }

  chassis_can2_cmd(motor_can2[0].set_current, motor_can2[1].set_current, motor_can2[2].set_current, motor_can2[3].set_current);
}

/**************************** CAN2发送信号 *****************************/
void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  tx_header.StdId = 0x200;
  tx_header.IDE = CAN_ID_STD;   // 标准帧
  tx_header.RTR = CAN_RTR_DATA; // 数据帧
  tx_header.DLC = 8;            // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/************************* yaw值校正 *******************************/
void yaw_correct()
{
  // 只执行一次
  if (yaw_correction_flag)
  {
    yaw_correction_flag = 0;
    INS.yaw_init = INS.Yaw;
  }
  INS.yaw_update = INS.Yaw - INS.yaw_init;
}