/*
**********Shoot_task射击任务**********
包含对拨盘的控制
拨盘为3508，ID = 5，CAN2控制, motor_can2[4]
遥控器右边拨杆控制，拨到最上面启动 (从上到下分别为132)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "cmsis_os.h"

shoot_t trigger; // 拨盘can1，id = 5
extern RC_ctrl_t rc_ctrl;
extern motor_info_t motor_can2[6];

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
    // 遥控器右边拨到上，电机启动
    if (rc_ctrl.rc.s[0] == 1)
    {
      shoot_start();
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
void shoot_loop_init()
{
  // trigger
  trigger.pid_value[0] = 50;
  trigger.pid_value[1] = 1;
  trigger.pid_value[2] = 0.05;

  // 初始化目标速度
  trigger.target_speed = 0;

  // 初始化PID
  pid_init(&trigger.pid, trigger.pid_value, 10000, 10000); // trigger
}

/***************射击模式*****************/
void shoot_start()
{
  trigger.target_speed = 250;
}

/***************停止射击模式**************/
void shoot_stop()
{
  trigger.target_speed = 0;
}

/********************************拨盘can1发送电流***************************/
void trigger_can2_cmd(int16_t v1)
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
void shoot_current_give()
{
  // trigger
  motor_can2[4].set_current = pid_calc(&trigger.pid, trigger.target_speed, -motor_can2[4].rotor_speed);
  trigger_can2_cmd(-motor_can2[4].set_current);
  // trigger_can2_cmd(1000);
}
