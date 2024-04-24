#include "drv_can.h"
#include "ins_task.h"
#include "remote_control.h"
#include "video_control.h"
#include "Robot.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define ECD_ANGLE_COEF 0.043945f // (360/8192),将编码器值转化为角度制

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];
extern motor_info_t motor_bottom[5];

INS_t INS_top;
float powerdata[4];
uint16_t pPowerdata[8];
uint16_t setpower = 5500;
uint8_t vision_is_tracking;
uint8_t friction_mode;

static uint8_t sbus_buf[18u]; // 遥控器接收的buffer
static uint8_t video_buf[12]; // 图传接收的buffer

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

#ifdef REMOTE_CONTROL
    if (rx_header.StdId == 0x33) // 接收上C传来的遥控器数据
    {
      memcpy(sbus_buf, rx_data, 8);
    }

    if (rx_header.StdId == 0x34) // 接收上C传来的遥控器数据
    {
      memcpy(sbus_buf + 8, rx_data, 8);
    }

    if (rx_header.StdId == 0x35)
    {
      memcpy(sbus_buf + 16, rx_data, 2);
      sbus_to_rc(sbus_buf);
      // memcpy(&INS_top.Yaw, rx_data + 2, 2);
      INS_top.Yaw = ((rx_data[2] << 8) | rx_data[3]) / 100.0f;
      vision_is_tracking = rx_data[4];
      friction_mode = rx_data[5];
    }
#endif

#ifdef VIDEO_CONTROL
    if (rx_header.StdId == 0x33) // 接收上C传来的图传数据
    {
      memcpy(video_buf, rx_data, 8);
    }

    if (rx_header.StdId == 0x34) // 接收上C传来的图传数据
    {
      memcpy(video_buf + 8, rx_data, 4);
      VideoRead(video_buf);
      vision_is_tracking = rx_data[4];
      friction_mode = rx_data[5];
    }
#endif
  }

  // can2电机信息接收
  if (hcan->Instance == CAN2)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can2 data

    if ((rx_header.StdId >= 0x201)     // 201-205
        && (rx_header.StdId <= 0x205)) // 判断标识符，标识符为0x200+ID
    {
      uint8_t index = rx_header.StdId - 0x201; // get motor index by can_id
      motor_bottom[index].last_ecd = motor_bottom[index].ecd;
      motor_bottom[index].ecd = ((rx_data[0] << 8) | rx_data[1]);
      motor_bottom[index].angle_single_round = ECD_ANGLE_COEF * (float)motor_bottom[index].ecd;
      motor_bottom[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      motor_bottom[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      motor_bottom[index].temp = rx_data[6];

      // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
      if (motor_bottom[index].ecd - motor_bottom[index].last_ecd > 4096)
        motor_bottom[index].total_round--;
      else if (motor_bottom[index].ecd - motor_bottom[index].last_ecd < -4096)
        motor_bottom[index].total_round++;
      motor_bottom[index].total_angle = motor_bottom[index].total_round * 360 + motor_bottom[index].angle_single_round;
    }

    if (rx_header.StdId == 0x211) // superpower
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
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
  }

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf, (uint32_t *)CAN_TX_MAILBOX0);
}

static void motor_read(uint8_t index, uint8_t rx_data[])
{
  motor_bottom[index].last_ecd = motor_bottom[index].ecd;
  motor_bottom[index].ecd = ((rx_data[0] << 8) | rx_data[1]);
  motor_bottom[index].angle_single_round = ECD_ANGLE_COEF * (float)motor_bottom[index].ecd;
  motor_bottom[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
  motor_bottom[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
  motor_bottom[index].temp = rx_data[6];

  // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
  if (motor_bottom[index].ecd - motor_bottom[index].last_ecd > 4096)
    motor_bottom[index].total_round--;
  else if (motor_bottom[index].ecd - motor_bottom[index].last_ecd < -4096)
    motor_bottom[index].total_round++;
  motor_bottom[index].total_angle = motor_bottom[index].total_round * 360 + motor_bottom[index].angle_single_round;
}