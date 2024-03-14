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

#define CHASSIS_SPEED_MAX_1 5000
#define CHASSIS_SPEED_MAX_2 3000
#define CHASSIS_SPEED_MAX_3 3000
#define CHASSIS_SPEED_MAX_4 3000
#define CHASSIS_SPEED_MAX_5 3000
#define CHASSIS_SPEED_MAX_6 3000
#define CHASSIS_SPEED_MAX_7 3000
#define CHASSIS_SPEED_MAX_8 3000
#define CHASSIS_SPEED_MAX_9 3000
#define CHASSIS_SPEED_MAX_10 3000
#define CHASSIS_SPEED_SUPERCAP 5000
#define CHASSIS_WZ_MAX 3000
#define KEY_START_OFFSET 10
#define KEY_STOP_OFFSET 20
#define FOLLOW_WEIGHT 80

motor_info_t motor_can2[6]; // can2电机信息结构体, 0123：底盘，4：拨盘, 5: 云台
chassis_t chassis[4];
volatile int16_t Vx = 0, Vy = 0, Wz = 0;
float rx = 0.2, ry = 0.2;
float relative_yaw = 0;
int yaw_correction_flag = 1;                                                          // yaw值校正标志
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz_acw, key_Wz_cw; // 键盘控制变量
uint8_t chassis_mode = 0;                                                             // 判断底盘状态，用于UI编写
uint8_t supercap_flag = 0;                                                            // 是否开启超级电容
int chassis_mode_flag = 0;
static float init_relative_yaw = 0;
static uint8_t cycle = 0; // do while循环一次的条件
float imu_err_yaw = 0;    // 记录yaw飘移的数值便于进行校正
int16_t chassis_speed_max = 0;

extern RC_ctrl_t rc_ctrl;
extern INS_t INS;
extern INS_t INS_top;
extern uint16_t shift_flag;
extern float Hero_chassis_power;
extern uint16_t Hero_chassis_power_buffer;
extern int superop; // 超电
extern uint8_t rx_buffer_c[49];
extern uint8_t rx_buffer_d[128];
extern uint8_t Hero_level;

// 功率限制算法的变量定义
float Watch_Power_Max;                                                // 限制值
float Watch_Power;                                                    // 实时功率
float Watch_Buffer;                                                   // 缓冲能量值
double Chassis_pidout;                                                // 输出值
double Chassis_pidout_target;                                         // 目标值
static double Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0; // 比例
float Klimit = 1;                                                     // 限制值
float Plimit = 0;                                                     // 约束比例
float Chassis_pidout_max;                                             // 输出值限制
// 超电
float data[6];
float vi, vo, pi, ii, io, ps;

// 读取键鼠数据控制底盘模式
static void read_keyboard(void);

// 参数重置
static void Chassis_loop_Init();

// 正常运动模式
static void chassis_mode_normal();

// 小陀螺模式
static void chassis_mode_top();

// 底盘跟随云台模式
static void chassis_mode_follow();

// 视觉运动模式
static void chassis_mode_vision();

// yaw值校正
static void yaw_correct();

// 电机电流控制
static void chassis_current_give(void);

// chassis CAN2发送信号
static void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

// 速度限制
static int16_t Motor_Speed_limiting(volatile int16_t motor_speed, int16_t limit_speed);

// 功率限制
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);

// 键鼠控制
static void key_control(void);

// 角度范围限制
static void detel_calc(fp32 *angle);

static void parse_data(char *data_string);

static void datapy();

// 判断机器人等级，赋值最大速度
static void level_judge();

// 判断是否开启超级电容
static void supercap_judge();

void Chassis_task(void const *pvParameters)
{

  Chassis_loop_Init();

  for (;;)
  {
    // 等级判断，获取最大速度
    level_judge();
    // 校正yaw值
    yaw_correct();
    // 底盘模式读取
    read_keyboard();
    // 判断是否开启超电
    supercap_judge();

    // 底盘跟随云台模式，右拨杆拨到中 || r键触发
    if (rc_ctrl.rc.s[0] == 3 || chassis_mode == 1)
    {
      key_control();
      chassis_mode_follow();
    }

    // 正常运动模式，右拨杆拨到下 || f键触发
    else if (rc_ctrl.rc.s[0] == 2 || chassis_mode == 2)
    {
      key_control();
      chassis_mode_normal();
    }

    else
    {
      key_control();
      chassis_mode_normal();
    }

    chassis_current_give();
    datapy(); // 超电数据接收
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

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis[i].pid, chassis[i].pid_value, 6000, 6000);
  }

  Vx = 0;
  Vy = 0;
  Wz = 0;
}

/****************************** 读取键鼠数据控制底盘模式 ****************************/
static void read_keyboard(void)
{
  if (r_flag)
    chassis_mode = 1;
  else if (f_flag)
    chassis_mode = 2;

  if (x_flag)
    supercap_flag = 0;
  else if (c_flag)
    supercap_flag = 1;
}

/*************************************** 正常运动模式 ************************************/
static void chassis_mode_normal()
{
  Vx = rc_ctrl.rc.ch[0] / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  Vy = rc_ctrl.rc.ch[1] / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back
  Wz = rc_ctrl.rc.ch[4] / 660.0f * chassis_speed_max + key_Wz_acw + key_Wz_cw;  // rotate

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

  cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值
}

/****************************** 小陀螺模式 *********************************/
static void chassis_mode_top()
{
  Vx = rc_ctrl.rc.ch[0] / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  Vy = rc_ctrl.rc.ch[1] / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back
  Wz = CHASSIS_WZ_MAX;

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

  // chassis[0].target_speed = 0;
  // chassis[1].target_speed = 0;
  // chassis[2].target_speed = 0;
  // chassis[3].target_speed = 0;

  cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值
}

/***************************** 底盘跟随云台模式 *******************************/
static void chassis_mode_follow()
{
  Vx = rc_ctrl.rc.ch[0] / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  Vy = rc_ctrl.rc.ch[1] / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back

  // // 切换模式的时候循环一次，计算 yaw 的差值，防止出现在切换模式的时候底盘突然一转
  // if (cycle)
  // {
  //   cycle = 0;
  //   init_relative_yaw = INS.yaw_update - INS_top.Yaw;
  // }

  // relative_yaw = INS.yaw_update - INS_top.Yaw - init_relative_yaw;

  // 保证切换回这个模式的时候，头在初始方向上，速度移动最快，方便逃跑(●'◡'●)
  relative_yaw = INS.yaw_update - INS_top.Yaw;

  // 消除静态旋转
  if (relative_yaw > -5 && relative_yaw < 5)
  {
    Wz = 0;
  }
  else
  {
    detel_calc(&relative_yaw);
    Wz = -relative_yaw * FOLLOW_WEIGHT;

    if (Wz > CHASSIS_WZ_MAX)
      Wz = CHASSIS_WZ_MAX;
    if (Wz < -CHASSIS_WZ_MAX)
      Wz = -CHASSIS_WZ_MAX;
  }

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反
  Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  chassis[0].target_speed = Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[1].target_speed = -Vy + Vx + 3 * (-Wz) * (rx + ry);
  chassis[2].target_speed = -Vy - Vx + 3 * (-Wz) * (rx + ry);
  chassis[3].target_speed = Vy - Vx + 3 * (-Wz) * (rx + ry);
}

/*************************** 视觉运动模式 ****************************/
static void chassis_mode_vision()
{
  // int16_t Temp_Vx = vision_Vx;
  // int16_t Temp_Vy = vision_Vy;
  // Wz = mapping(rc_ctrl.rc.ch[4], -660, 660, -chassis_speed_max, chassis_speed_max); // rotate

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
static void chassis_current_give()
{
  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    chassis[i].target_speed = Motor_Speed_limiting(chassis[i].target_speed, chassis_speed_max);
    motor_can2[i].set_current = pid_calc(&chassis[i].pid, chassis[i].target_speed, motor_can2[i].rotor_speed);
  }
  Chassis_Power_Limit(4 * chassis_speed_max);
  chassis_can2_cmd(motor_can2[0].set_current, motor_can2[1].set_current, motor_can2[2].set_current, motor_can2[3].set_current);
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

/************************* yaw值校正 *******************************/
static void yaw_correct()
{
  // 只执行一次
  if (yaw_correction_flag)
  {
    yaw_correction_flag = 0;
    INS.yaw_init = INS.Yaw;
  }
  // Wz为负，顺时针旋转，陀螺仪飘 60°/min（以3000为例转出的，根据速度不同调整）
  // 解决yaw偏移，完成校正
  if (Wz > 500)
    imu_err_yaw -= 0.001f;
  // imu_err_yaw -= 0.001f * chassis_speed_max / 3000.0f;
  if (Wz < -500)
    imu_err_yaw += 0.001f;
  // imu_err_yaw += 0.001f * chassis_speed_max / 3000.0f;

  INS.yaw_update = INS.Yaw - INS.yaw_init + imu_err_yaw;
}

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{
  // 819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
  Watch_Power_Max = Klimit;
  Watch_Power = Hero_chassis_power;
  Watch_Buffer = Hero_chassis_power_buffer; // 限制值，功率值，缓冲能量值，初始值是1，0，0
  // get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

  Chassis_pidout_max = 32768; // 32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      chassis[i].target_speed = Motor_Speed_limiting(chassis[i].target_speed, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
    }
  }
  else
  {
    Chassis_pidout = (fabs(chassis[0].target_speed - motor_can2[0].rotor_speed) +
                      fabs(chassis[1].target_speed - motor_can2[1].rotor_speed) +
                      fabs(chassis[2].target_speed - motor_can2[2].rotor_speed) +
                      fabs(chassis[3].target_speed - motor_can2[3].rotor_speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和

    //	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
      Scaling1 = (chassis[0].target_speed - motor_can2[0].rotor_speed) / Chassis_pidout;
      Scaling2 = (chassis[1].target_speed - motor_can2[1].rotor_speed) / Chassis_pidout;
      Scaling3 = (chassis[2].target_speed - motor_can2[2].rotor_speed) / Chassis_pidout;
      Scaling4 = (chassis[3].target_speed - motor_can2[3].rotor_speed) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
      Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    //		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
    //		else{Klimit = 0;}
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
      Klimit = 1;
    else if (Klimit < -1)
      Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    /*缓冲能量占比环，总体约束*/
    if (Watch_Buffer < 50 && Watch_Buffer >= 40)
      Plimit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
    else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
      Plimit = 0.75;
    else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
      Plimit = 0.5;
    else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
      Plimit = 0.25;
    else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
      Plimit = 0.125;
    else if (Watch_Buffer < 10 && Watch_Buffer >= 0)
      Plimit = 0.05;
    else
    {
      Plimit = 1;
    }

    motor_can2[0].set_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    motor_can2[1].set_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    motor_can2[2].set_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    motor_can2[3].set_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
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
  if (d_flag)
    key_x_fast += KEY_START_OFFSET;
  else
    key_x_fast -= KEY_STOP_OFFSET;

  if (a_flag)
    key_x_slow += KEY_START_OFFSET;
  else
    key_x_slow -= KEY_STOP_OFFSET;

  if (w_flag)
    key_y_fast += KEY_START_OFFSET;
  else
    key_y_fast -= KEY_STOP_OFFSET;

  if (s_flag)
    key_y_slow += KEY_START_OFFSET;
  else
    key_y_slow -= KEY_STOP_OFFSET;

  // 正转
  if (shift_flag)
    key_Wz_acw += KEY_START_OFFSET;
  else
    key_Wz_acw -= KEY_STOP_OFFSET;

  // 反转
  if (ctrl_flag)
    key_Wz_cw -= KEY_START_OFFSET;
  else
    key_Wz_cw += KEY_STOP_OFFSET;

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
  if (key_Wz_acw > CHASSIS_WZ_MAX)
    key_Wz_acw = CHASSIS_WZ_MAX;
  if (key_Wz_acw < 0)
    key_Wz_acw = 0;
  if (key_Wz_cw < -CHASSIS_WZ_MAX)
    key_Wz_cw = -CHASSIS_WZ_MAX;
  if (key_Wz_cw > 0)
    key_Wz_cw = 0;
}

static void detel_calc(fp32 *angle)
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

static void parse_data(char *data_string)
{
  sscanf(data_string, "Vi:%f Vo:%f Pi:%f Ii:%f Io:%f Ps:%f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
  vi = data[0]; // Vi输入电压（电池电压）
  vo = data[1]; // Vo输出电压（超级电容电压）
  pi = data[2]; // Pi输入功率
  ii = data[3]; // Ii输入电流（电池电流）
  io = data[4]; // Io输出电流（负载电流）
  ps = data[5]; // Ps参考恒功率值
}

static void datapy()
{
  int index;

  for (int i = 0; i < 128; i++)
  {
    if ((rx_buffer_d[i] == 0x56) && (rx_buffer_d[i + 1] == 'i'))
    {
      index = i;
      break;
    }
  }

  memcpy(rx_buffer_c, &rx_buffer_d[index], 49);

  parse_data(rx_buffer_c);
}

// 判断机器人等级，赋值最大速度
static void level_judge()
{
  if (Hero_level)
  {
    switch (Hero_level)
    {
    case 1:
      chassis_speed_max = CHASSIS_SPEED_MAX_1;
      break;
    case 2:
      chassis_speed_max = CHASSIS_SPEED_MAX_2;
      break;
    case 3:
      chassis_speed_max = CHASSIS_SPEED_MAX_3;
      break;
    case 4:
      chassis_speed_max = CHASSIS_SPEED_MAX_4;
      break;
    case 5:
      chassis_speed_max = CHASSIS_SPEED_MAX_5;
      break;
    case 6:
      chassis_speed_max = CHASSIS_SPEED_MAX_6;
      break;
    case 7:
      chassis_speed_max = CHASSIS_SPEED_MAX_7;
      break;
    case 8:
      chassis_speed_max = CHASSIS_SPEED_MAX_8;
      break;
    case 9:
      chassis_speed_max = CHASSIS_SPEED_MAX_9;
      break;
    case 10:
      chassis_speed_max = CHASSIS_SPEED_MAX_10;
      break;
    }
  }
  else
    chassis_speed_max = CHASSIS_SPEED_MAX_1;
}

// 判断是否开启超级电容
static void supercap_judge()
{
  if (supercap_flag)
    chassis_speed_max = CHASSIS_SPEED_SUPERCAP;
}