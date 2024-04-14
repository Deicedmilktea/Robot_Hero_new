#include "drv_can.h"
#include "ins_task.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define ECD_ANGLE_COEF 0.043945f // (360/8192),将编码器值转化为角度制

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
uint16_t can_cnt_1 = 0;
extern motor_info_t motor_can2[6];
INS_t INS_top;
float vision_yaw = 0;
// float vision_Vx = 0;
// float vision_Vy = 0;
float yaw_speed = 0;
float powerdata[4];
uint16_t pPowerdata[8];

uint8_t rx_data2[8];
uint16_t setpower = 5500;
int canerror = 0;
int error9 = 0;

void CAN1_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 0;                      // filter 0
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);                         // init can filter
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can的FIFO0中断
  HAL_CAN_Start(&hcan1);                                             // 启动can1
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 14;                     // filter 14
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &can_filter); // init can filter
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2); // 启动can2
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;

  // can1信息接收
  if (hcan->Instance == CAN1)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can1 data

    if (rx_header.StdId == 0x33) // 接收上C传来的遥控器数据
    {
      rc_ctrl.rc.ch[0] = ((rx_data[0] | (rx_data[1] << 8)) & 0x07ff);                                      //!< Channel 0  ÖÐÖµÎª1024£¬×î´óÖµ1684£¬×îÐ¡Öµ364£¬²¨¶¯·¶Î§£º660
      rc_ctrl.rc.ch[1] = ((((rx_data[1] >> 3) & 0xff) | (rx_data[2] << 5)) & 0x07ff);                      //!< Channel 1
      rc_ctrl.rc.ch[2] = ((((rx_data[2] >> 6) & 0xff) | (rx_data[3] << 2) | (rx_data[4] << 10)) & 0x07ff); //!< Channel 2
      rc_ctrl.rc.ch[3] = ((((rx_data[4] >> 1) & 0xff) | (rx_data[5] << 7)) & 0x07ff);                      //!< Channel 3
      rc_ctrl.rc.s[0] = ((rx_data[5] >> 4) & 0x0003);                                                      //!< Switch left£¡£¡£¡ÕâÄáÂêÊÇÓÒ
      rc_ctrl.rc.s[1] = ((rx_data[5] >> 4) & 0x000C) >> 2;                                                 //!< Switch right£¡£¡£¡Õâ²ÅÊÇ×ó
      rc_ctrl.mouse.x = rx_data[6] | (rx_data[7] << 8);                                                    //!< Mouse X axis
      rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
      rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
      rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
      rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    }

    if (rx_header.StdId == 0x34) // 接收上C传来的遥控器数据
    {
      rc_ctrl.mouse.y = rx_data[0] | (rx_data[1] << 8); //!< Mouse Y axis
      rc_ctrl.mouse.z = rx_data[2] | (rx_data[3] << 8); //!< Mouse Z axis
      rc_ctrl.mouse.press_l = rx_data[4];               //!< Mouse Left Is Press ?
      rc_ctrl.mouse.press_r = rx_data[5];               //!< Mouse Right Is Press ?
      rc_ctrl.key.v = rx_data[6] | (rx_data[7] << 8);   //!< KeyBoard value

      // Some flag of keyboard
      w_flag = (rx_data[6] & 0x01);
      s_flag = (rx_data[6] & 0x02);
      a_flag = (rx_data[6] & 0x04);
      d_flag = (rx_data[6] & 0x08);
      q_flag = (rx_data[6] & 0x40);
      e_flag = (rx_data[6] & 0x80);
      shift_flag = (rx_data[6] & 0x10);
      ctrl_flag = (rx_data[6] & 0x20);
      press_left = rc_ctrl.mouse.press_l;
      press_right = rc_ctrl.mouse.press_r;
      r_flag = rc_ctrl.key.v & (0x00 | 0x01 << 8);
      f_flag = rc_ctrl.key.v & (0x00 | 0x02 << 8);
      g_flag = rc_ctrl.key.v & (0x00 | 0x04 << 8);
      z_flag = rc_ctrl.key.v & (0x00 | 0x08 << 8);
      x_flag = rc_ctrl.key.v & (0x00 | 0x10 << 8);
      c_flag = rc_ctrl.key.v & (0x00 | 0x20 << 8);
      v_flag = rc_ctrl.key.v & (0x00 | 0x40 << 8);
      b_flag = rc_ctrl.key.v & (0x00 | 0x80 << 8);
    }

    if (rx_header.StdId == 0x35)
    {
      rc_ctrl.rc.ch[4] = ((rx_data[0] | (rx_data[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;
      INS_top.Yaw = ((int16_t)((rx_data[2] << 8) | rx_data[3])) / 100.0f; // yaw
      // INS_top.Roll = ((int16_t)((rx_data[4] << 8) | rx_data[5])) / 100;  // roll（roll和pitch根据c放置位置不同可能交换）
      // INS_top.Pitch = ((int16_t)((rx_data[6] << 8) | rx_data[7])) / 100; // pitch
      // vision_Vx = ((int16_t)((rx_data[4] << 8) | rx_data[5])) / 100; // 导航所需Vx
      // vision_Vy = ((int16_t)((rx_data[6] << 8) | rx_data[7])) / 100; // 导航所需Vy
    }
  }

  // can2电机信息接收
  if (hcan->Instance == CAN2)
  {
    error9++;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can2 data

    if ((rx_header.StdId >= 0x201)     // 201-205
        && (rx_header.StdId <= 0x205)) // 判断标识符，标识符为0x200+ID
    {
      uint8_t index = rx_header.StdId - 0x201; // get motor index by can_id
      motor_can2[index].last_ecd = motor_can2[index].ecd;
      motor_can2[index].ecd = ((rx_data[0] << 8) | rx_data[1]);
      motor_can2[index].angle_single_round = ECD_ANGLE_COEF * (float)motor_can2[index].ecd;
      motor_can2[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      motor_can2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      motor_can2[index].temp = rx_data[6];

      // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
      if (motor_can2[index].ecd - motor_can2[index].last_ecd > 4096)
        motor_can2[index].total_round--;
      else if (motor_can2[index].ecd - motor_can2[index].last_ecd < -4096)
        motor_can2[index].total_round++;
      motor_can2[index].total_angle = motor_can2[index].total_round * 360 + motor_can2[index].angle_single_round;
    }

    if (rx_header.StdId == 0x209) // gimbal
    {
      motor_can2[5].ecd = ((rx_data[0] << 8) | rx_data[1]);
      motor_can2[5].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      motor_can2[5].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      motor_can2[5].temp = rx_data[6];
    }

    if (rx_header.StdId == 0x211)
    {
      uint16_t *pPowerdata = (uint16_t *)rx_data;

      powerdata[0] = (float)pPowerdata[0] / 100.f; // 输入电压
      powerdata[1] = (float)pPowerdata[1] / 100.f; // 电容电压
      powerdata[2] = (float)pPowerdata[2] / 100.f; // 输入电流
      powerdata[3] = (float)pPowerdata[3] / 100.f; // P
    }
  }
}

void can_remote(uint8_t sbus_buf[], uint8_t can_send_id) // 调用can来发送遥控器数据
{
  CAN_TxHeaderTypeDef tx_header;

  tx_header.StdId = can_send_id; // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE = CAN_ID_STD;    // 标准帧
  tx_header.RTR = CAN_RTR_DATA;  // 数据帧
  tx_header.DLC = 8;             // 发送数据长度（字节）

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf, (uint32_t *)CAN_TX_MAILBOX0);
}

// CAN1发送信号（底盘+云台+pitch）
void set_motor_current_can1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  tx_header.StdId = (id_range == 0) ? (0x200) : (0x2FF); // 如果id_range==0则等于0x200,id_range==1则等于0x2ff（ID号）
  tx_header.IDE = CAN_ID_STD;                            // 标准帧
  tx_header.RTR = CAN_RTR_DATA;                          // 数据帧
  tx_header.DLC = 8;                                     // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}