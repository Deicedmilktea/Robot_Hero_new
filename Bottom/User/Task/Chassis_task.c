/*
*************Chassis_task底盘任务**************
采用3508，CAN2，ID = 1234
遥控器控制：右遥杆上下→前后
           右遥杆左右→左右
           左滑轮→旋转
*/

#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "remote_control.h"
#include "Robot.h"
#include "referee_task.h"
#include "remote_control.h"
#include "video_control.h"
#include "Supercap_task.h"

motor_info_t motor_bottom[5]; // can2电机信息结构体, 0123：底盘，4：拨盘
uint8_t supercap_flag = 0;    // 是否开启超级电容

static chassis_t chassis_motor[4]; // 电机信息结构体
static int16_t Vx = 0, Vy = 0, Wz = 0;
static float rx = 0.2, ry = 0.2;
static float relative_yaw = 0;                                                        // yaw值校正标志
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz_acw, key_Wz_cw; // 键盘控制变量
static int16_t chassis_speed_max = 0;                                                 // 底盘速度，不同等级对应不同速度
static int16_t chassis_wz_max = 0;
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern int16_t top_yaw;
extern referee_hero_t referee_hero;
extern SupercapRxData_t SupercapRxData; // 超电接收数据
extern CAN_HandleTypeDef hcan2;
extern uint8_t trigger_flag;
extern uint8_t friction_mode;
extern uint8_t is_remote_online;
extern uint8_t video_mode;

// 功率限制算法的变量定义
static float Watch_Power_Max;                                         // 限制值
static float Watch_Power;                                             // 实时功率
static float Watch_Buffer;                                            // 缓冲能量值
static double Chassis_pidout;                                         // 输出值
static double Chassis_pidout_target;                                  // 目标值
static double Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0; // 比例
static float Klimit = 1;                                              // 限制值
static float Plimit = 0;                                              // 约束比例
static float Chassis_pidout_max;                                      // 输出值限制

static void Chassis_loop_Init();                                                        // 参数重置
static void chassis_mode_follow();                                                      // 底盘跟随云台模式
static void chassis_mode_stop();                                                        // 急停模式
static void read_keyboard();                                                            // 读取键鼠数据控制底盘模式
static void chassis_current_give();                                                     // 电机电流控制
static void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4);           // chassis CAN2发送信号
static int16_t Motor_Speed_limiting(volatile int16_t motor_speed, int16_t limit_speed); // 速度限制
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);                    // 功率限制
static void key_control(void);                                                          // 键鼠控制
static void detel_calc(float *angle);                                                   // 角度范围限制
static void level_judge();                                                              // 判断机器人等级，赋值最大速度
static void rc_mode_choose();                                                           // 遥控器链路选择底盘模式
static void video_mode_choose();                                                        // 图传链路选择底盘模式

void Chassis_task(void const *pvParameters)
{

  Chassis_loop_Init();

  for (;;)
  {
    level_judge(); // 等级判断，获取最大速度

    if (is_remote_online) // 遥控器链路
      rc_mode_choose();

    else // 图传链路
      video_mode_choose();

    chassis_current_give();
    osDelay(1);
  }
}

static void Chassis_loop_Init()
{
  referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

  for (uint8_t i = 0; i < 4; i++)
  {
    chassis_motor[i].pid_value[0] = 30;
    chassis_motor[i].pid_value[1] = 0.5;
    chassis_motor[i].pid_value[2] = 0;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis_motor[i].pid, chassis_motor[i].pid_value, 12000, 12000);
  }

  Vx = 0;
  Vy = 0;
  Wz = 0;
}

/****************************** 读取键鼠数据控制底盘模式 ****************************/
static void read_keyboard()
{
  // 遥控器链路
  if (is_remote_online)
  {
    // F键控制底盘模式
    switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    case 1:
      ui_data.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // normal
      break;
    default:
      ui_data.chassis_mode = CHASSIS_ZERO_FORCE; // stop
      break;
    }

    switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2)
    {
    case 1:
      ui_data.shoot_mode = SHOOT_BUFF;
      break;
    default:
      ui_data.shoot_mode = SHOOT_NORMAL;
      break;
    }
  }

  // 图传链路
  else
  {
    switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    case 1:
      ui_data.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // normal
      break;
    default:
      ui_data.chassis_mode = CHASSIS_ZERO_FORCE; // stop
      break;
    }

    switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2)
    {
    case 1:
      ui_data.shoot_mode = SHOOT_BUFF;
      break;
    default:
      ui_data.shoot_mode = SHOOT_NORMAL;
      break;
    }
  }

  // 超电开关
  switch (SupercapRxData.state)
  {
  case 3:
    supercap_flag = 1;
    ui_data.supcap_mode = SUPCAP_ON;
    break;
  default: // receive 0
    supercap_flag = 0;
    ui_data.supcap_mode = SUPCAP_OFF;
    break;
  }

  // E键切换摩擦轮速度，012分别为low，normal，high
  switch (friction_mode)
  {
  case FRICTION_NORMAL:
    ui_data.friction_mode = FRICTION_NORMAL;
    break;
  case FRICTION_LOW:
    ui_data.friction_mode = FRICTION_LOW;
    break;
  case FRICTION_HIGH:
    ui_data.friction_mode = FRICTION_HIGH;
    break;
  case FRICTION_STOP:
    ui_data.friction_mode = FRICTION_STOP;
    break;
  }

  // B键切换图传模式，normal，adaptive
  switch (video_mode)
  {
  case 1:
    ui_data.video_mode = VIDEO_ADAPTIVE;
    break;
  default:
    ui_data.video_mode = VIDEO_NORMAL;
    break;
  }
}

/***************************** 底盘跟随云台模式 *******************************/
static void chassis_mode_follow()
{
  Vx = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  Vy = rc_ctrl[TEMP].rc.rocker_r1 / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back

  // 保证切换回这个模式的时候，头在初始方向上，速度移动最快，方便逃跑(●'◡'●)
  relative_yaw = (top_yaw - INIT_YAW) / 8191.0f * 360;

  // 便于小陀螺操作
  if (key_Wz_acw)
  {
    Wz = key_Wz_acw; // rotate
  }

  else if (rc_ctrl[TEMP].rc.dial)
  {
    Wz = rc_ctrl[TEMP].rc.dial / 660.0f * chassis_wz_max; // rotate
  }

  else
  {
    // 消除静态旋转
    if (relative_yaw > -5 && relative_yaw < 5)
    {
      Wz = 0;
    }

    else
    {
      detel_calc(&relative_yaw);
      Wz = -relative_yaw * FOLLOW_WEIGHT;

      if (Wz > chassis_wz_max)
        Wz = chassis_wz_max;
      if (Wz < -chassis_wz_max)
        Wz = -chassis_wz_max;
    }
  }

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;
  Vx = cosf(-relative_yaw / 57.3f) * Temp_Vx - sinf(-relative_yaw / 57.3f) * Temp_Vy;
  Vy = sinf(-relative_yaw / 57.3f) * Temp_Vx + cosf(-relative_yaw / 57.3f) * Temp_Vy;

  chassis_motor[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis_motor[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis_motor[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  chassis_motor[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);
}

/*************************** 急停模式 ****************************/
static void chassis_mode_stop()
{
  chassis_motor[0].target_speed = 0;
  chassis_motor[1].target_speed = 0;
  chassis_motor[2].target_speed = 0;
  chassis_motor[3].target_speed = 0;
}

/*************************** 遥控器链路选择底盘模式 ****************************/
void rc_mode_choose()
{
  // 右拨杆下，遥控操作
  if (switch_is_down(rc_ctrl[TEMP].rc.switch_right))
  {
    chassis_mode_follow(); // 正常模式，便于检录小陀螺展示
  }

  // 右拨杆中，键鼠操作
  else if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right))
  {
    // 底盘模式读取
    read_keyboard();
    key_control();

    if (ui_data.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
      chassis_mode_follow();
    }
    else
    {
      chassis_mode_stop();
    }
  }

  // 右拨杆上，校正yaw
  else
  {
    chassis_mode_stop();
  }
}

void video_mode_choose()
{
  // 底盘模式读取
  read_keyboard();
  key_control();

  if (ui_data.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
  {
    chassis_mode_follow();
  }
  else
  {
    chassis_mode_stop();
  }
}

/*************************** 电机电流控制 ****************************/
static void chassis_current_give()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis_motor[i].target_speed = Motor_Speed_limiting(chassis_motor[i].target_speed, chassis_speed_max);
    motor_bottom[i].set_current = pid_calc(&chassis_motor[i].pid, chassis_motor[i].target_speed, motor_bottom[i].rotor_speed);
  }
  // 在功率限制算法中，静止状态底盘锁不住，这时取消功率限制，保证发弹稳定性
  // if (chassis_motor[0].target_speed != 0 || chassis_motor[1].target_speed != 0 || chassis_motor[2].target_speed != 0 || chassis_motor[3].target_speed != 0)
  Chassis_Power_Limit(4 * chassis_speed_max);

  chassis_can2_cmd(motor_bottom[0].set_current, motor_bottom[1].set_current, motor_bottom[2].set_current, motor_bottom[3].set_current);
}

/**************************** CAN2发送信号 *****************************/
static void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
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

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{
  // 819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
  Watch_Power_Max = Klimit;
  Watch_Power = referee_hero.chassis_power;
  Watch_Buffer = referee_hero.buffer_energy; // 限制值，功率值，缓冲能量值，初始值是1，0，0

  Chassis_pidout_max = 52428; // 16384 * 4 *0.8，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      chassis_motor[i].target_speed = Motor_Speed_limiting(chassis_motor[i].target_speed, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
    }
  }
  else
  {
    Chassis_pidout = (fabs(chassis_motor[0].target_speed - motor_bottom[0].rotor_speed) +
                      fabs(chassis_motor[1].target_speed - motor_bottom[1].rotor_speed) +
                      fabs(chassis_motor[2].target_speed - motor_bottom[2].rotor_speed) +
                      fabs(chassis_motor[3].target_speed - motor_bottom[3].rotor_speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
      Scaling1 = (chassis_motor[0].target_speed - motor_bottom[0].rotor_speed) / Chassis_pidout;
      Scaling2 = (chassis_motor[1].target_speed - motor_bottom[1].rotor_speed) / Chassis_pidout;
      Scaling3 = (chassis_motor[2].target_speed - motor_bottom[2].rotor_speed) / Chassis_pidout;
      Scaling4 = (chassis_motor[3].target_speed - motor_bottom[3].rotor_speed) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
      Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
      Klimit = 1;
    else if (Klimit < -1)
      Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    /*缓冲能量占比环，总体约束*/
    if (!supercap_flag)
    {
      if (Watch_Buffer <= 60 && Watch_Buffer >= 40)
        Plimit = 0.95; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
      else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
        Plimit = 0.75;
      else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
        Plimit = 0.5;
      else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
        Plimit = 0.25;
      else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
        Plimit = 0.125;
      else if (Watch_Buffer < 10 && Watch_Buffer > 0)
        Plimit = 0.05;
      else
        Plimit = 1;
    }

    else
      Plimit = 1;

    motor_bottom[0].set_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    motor_bottom[1].set_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    motor_bottom[2].set_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    motor_bottom[3].set_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
  }
}

static int16_t Motor_Speed_limiting(volatile int16_t motor_speed, int16_t limit_speed)
{
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;

  temp = (motor_speed > 0) ? (motor_speed) : (-motor_speed); // 求绝对值

  if (temp > max)
    max = temp;

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    motor_speed *= rate;
  }
  return motor_speed;
}

static void key_control(void)
{
  // 遥控器链路
  if (is_remote_online)
  {
    if (rc_ctrl[TEMP].key[KEY_PRESS].d)
      key_x_fast += KEY_START_OFFSET;
    else
      key_x_fast -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].a)
      key_x_slow += KEY_START_OFFSET;
    else
      key_x_slow -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].w)
      key_y_fast += KEY_START_OFFSET;
    else
      key_y_fast -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].s)
      key_y_slow += KEY_START_OFFSET;
    else
      key_y_slow -= KEY_STOP_OFFSET;

    // 正转
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift)
      key_Wz_acw += KEY_START_OFFSET;
    else
      key_Wz_acw -= KEY_STOP_OFFSET;
  }

  // 图传链路
  else
  {
    if (video_ctrl[TEMP].key[KEY_PRESS].d)
      key_x_fast += KEY_START_OFFSET;
    else
      key_x_fast -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].a)
      key_x_slow += KEY_START_OFFSET;
    else
      key_x_slow -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].w)
      key_y_fast += KEY_START_OFFSET;
    else
      key_y_fast -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].s)
      key_y_slow += KEY_START_OFFSET;
    else
      key_y_slow -= KEY_STOP_OFFSET;

    // 正转
    if (video_ctrl[TEMP].key[KEY_PRESS].shift)
      key_Wz_acw += KEY_START_OFFSET;
    else
      key_Wz_acw -= KEY_STOP_OFFSET;
  }

  if (key_x_fast > chassis_speed_max)
    key_x_fast = chassis_speed_max;
  if (key_x_fast < 0)
    key_x_fast = 0;
  if (key_x_slow > chassis_speed_max)
    key_x_slow = chassis_speed_max;
  if (key_x_slow < 0)
    key_x_slow = 0;
  if (key_y_fast > chassis_speed_max)
    key_y_fast = chassis_speed_max;
  if (key_y_fast < 0)
    key_y_fast = 0;
  if (key_y_slow > chassis_speed_max)
    key_y_slow = chassis_speed_max;
  if (key_y_slow < 0)
    key_y_slow = 0;
  if (key_Wz_acw > chassis_wz_max)
    key_Wz_acw = chassis_wz_max;
  if (key_Wz_acw < 0)
    key_Wz_acw = 0;
}

static void detel_calc(float *angle)
{
  // 如果角度大于180度，则减去360度
  if (*angle > 180)
  {
    *angle -= 360;
  }

  // 如果角度小于-180度，则加上360度
  else if (*angle < -180)
  {
    *angle += 360;
  }
}

// 判断机器人等级，赋值最大速度
static void level_judge()
{
  if (referee_hero.robot_level != 0)
  {
    switch (referee_hero.robot_level)
    {
    case 1:
      chassis_speed_max = CHASSIS_SPEED_MAX_1;
      chassis_wz_max = CHASSIS_WZ_MAX_1;
      break;
    case 2:
      chassis_speed_max = CHASSIS_SPEED_MAX_2;
      chassis_wz_max = CHASSIS_WZ_MAX_1;
      break;
    case 3:
      chassis_speed_max = CHASSIS_SPEED_MAX_3;
      chassis_wz_max = CHASSIS_WZ_MAX_1;
      break;
    case 4:
      chassis_speed_max = CHASSIS_SPEED_MAX_4;
      chassis_wz_max = CHASSIS_WZ_MAX_1;
      break;
    case 5:
      chassis_speed_max = CHASSIS_SPEED_MAX_5;
      chassis_wz_max = CHASSIS_WZ_MAX_1;
      break;
    case 6:
      chassis_speed_max = CHASSIS_SPEED_MAX_6;
      chassis_wz_max = CHASSIS_WZ_MAX_2;
      break;
    case 7:
      chassis_speed_max = CHASSIS_SPEED_MAX_7;
      chassis_wz_max = CHASSIS_WZ_MAX_2;
      break;
    case 8:
      chassis_speed_max = CHASSIS_SPEED_MAX_8;
      chassis_wz_max = CHASSIS_WZ_MAX_2;
      break;
    case 9:
      chassis_speed_max = CHASSIS_SPEED_MAX_9;
      chassis_wz_max = CHASSIS_WZ_MAX_2;
      break;
    case 10:
      chassis_speed_max = CHASSIS_SPEED_MAX_10;
      chassis_wz_max = CHASSIS_WZ_MAX_2;
      break;
    }
  }
  else
  {
    chassis_speed_max = CHASSIS_SPEED_MAX_1;
    chassis_wz_max = CHASSIS_WZ_MAX_1;
  }

  if (supercap_flag)
    chassis_speed_max += 2000;
}