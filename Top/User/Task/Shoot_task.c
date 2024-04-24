/*
**********Shoot_task射击任务**********
包含对三个摩擦轮以及开镜电机的控制
摩擦轮分别为3508, ID = 1(left) ID = 2(right) ID = 3(up) ID = 4(lens), CAN2 控制
遥控器控制：左边拨杆，拨到中间启动摩擦轮(lr)，最上面发射(up) (从上到下分别为132)
键鼠控制：默认开启摩擦轮(lr)，左键发射(up)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control.h"
#include "video_control.h"

#define FRICTION_MAX_SPEED 20000
#define FRICTION_SPEED 6200
#define FRICTION_LENS_SPEED 1000

motor_info_t motor_top[6]; //[0]-[3]:left, right, up, [4]:pitch, [5]:yaw

static shoot_t shoot_motor[4]; // 摩擦轮can2，id = 56

uint8_t friction_flag = 0; // 摩擦轮转速标志位，012分别为low, normal, high, 默认为normal

extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];

// 初始化
static void shoot_loop_init();

// 左右摩擦轮开启模式
static void shoot_start_lr();

// 三摩擦轮开启模式
static void shoot_start_all();

// 摩擦轮关闭模式
static void shoot_stop();

// 判断开关镜
static void lens_judge();

// can2发送电流
static void shoot_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

// PID计算速度并发送电流
static void shoot_current_give();

// 读取键鼠是否开启摩擦轮
static void read_keyboard();

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
#ifdef REMOTE_CONTROL
    // 右拨杆中，键鼠控制
    if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right))
    {
      shoot_start_all();
      lens_judge();
    }

    // 右拨杆下，遥控器控制
    else
    {
      // 遥控器左边拨到上和中，电机启动
      if (switch_is_up(rc_ctrl[TEMP].rc.switch_left) || switch_is_mid(rc_ctrl[TEMP].rc.switch_left))
      {
        shoot_start_all();
      }
      else
      {
        shoot_stop();
      }
    }
#endif

#ifdef VIDEO_CONTROL
    shoot_start_all();
    lens_judge();
#endif

    // shoot_current_give();
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

  // lens
  shoot_motor[3].pid_value[0] = 20;
  shoot_motor[3].pid_value[1] = 0;
  shoot_motor[3].pid_value[2] = 0;

  // 初始化目标速度
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;
  shoot_motor[3].target_speed = 0;

  // 初始化PID
  pid_init(&shoot_motor[0].pid, shoot_motor[0].pid_value, 1000, FRICTION_MAX_SPEED); // friction_right
  pid_init(&shoot_motor[1].pid, shoot_motor[1].pid_value, 1000, FRICTION_MAX_SPEED); // friction_left
  pid_init(&shoot_motor[2].pid, shoot_motor[2].pid_value, 1000, FRICTION_MAX_SPEED); // friction_up
  pid_init(&shoot_motor[3].pid, shoot_motor[3].pid_value, 1000, FRICTION_MAX_SPEED); // lens
}

/*************** 左右摩擦轮开启模式 *****************/
static void shoot_start_lr()
{
  shoot_motor[0].target_speed = FRICTION_SPEED;
  shoot_motor[1].target_speed = -FRICTION_SPEED;
  shoot_motor[2].target_speed = 0;
}

/**************** 三摩擦轮开启模式 *****************/
static void shoot_start_all()
{
  shoot_motor[0].target_speed = FRICTION_SPEED;
  shoot_motor[1].target_speed = -FRICTION_SPEED;
  shoot_motor[2].target_speed = -FRICTION_LENS_SPEED;
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
  if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 2 == 1)
    return;
}

/*************** 读取键鼠是否开启摩擦轮 **************/
static void read_keyboard()
{
  // 开启摩擦轮
  if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 3 == 0)
    friction_flag = 1; // normal
  else if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 3 == 1)
    friction_flag = 2; // high
  else
    friction_flag = 0; // low
}

/********************************摩擦轮can2发送电流***************************/
static void shoot_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = 0x200;
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
  motor_top[3].set_current = pid_calc(&shoot_motor[3].pid, shoot_motor[3].target_speed, motor_top[3].rotor_speed);

  shoot_can2_cmd(motor_top[0].set_current, motor_top[1].set_current, motor_top[2].set_current, motor_top[3].set_current);
}
