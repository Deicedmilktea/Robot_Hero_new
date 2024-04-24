#include "drv_can.h"
#include "ins_task.h"
#include "remote_control.h"
#include "video_control.h"
#include "Robot.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define ECD_ANGLE_COEF 0.043945f // (360/8192),将编码器值转化为角度制
#define PITCH_INDEX 4
#define YAW_INDEX 5

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];
extern motor_info_t motor_top[6];

INS_t INS_bottom; // 下C板的imu数据

static void motor_read(uint8_t index, uint8_t rx_data[]);

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

    // yaw
    if ((rx_header.StdId == 0x206))
    {
      motor_read(YAW_INDEX, rx_data);
    }

    if (rx_header.StdId == 0x55)
    {
      INS_bottom.Pitch = ((rx_data[0] << 8) | rx_data[1]) / 100.0f;
    }
  }

  // can2电机信息接收
  if (hcan->Instance == CAN2)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can2 data
    if ((rx_header.StdId >= 0x201)                                 // 201-204
        && (rx_header.StdId <= 0x204))                             // 判断标识符，标识符为0x200+ID
    {
      uint8_t index = rx_header.StdId - 0x201; // get motor index by can_id
      motor_read(index, rx_data);
    }

    // pitch
    if (rx_header.StdId == 0x208)
    {
      motor_read(PITCH_INDEX, rx_data);
    }
  }
}

void can_remote(uint8_t sbus_buf[], uint32_t can_send_id) // 调用can来发送遥控器数据
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
  motor_top[index].last_ecd = motor_top[index].ecd;
  motor_top[index].ecd = ((rx_data[0] << 8) | rx_data[1]);
  motor_top[index].angle_single_round = ECD_ANGLE_COEF * (float)motor_top[index].ecd;
  motor_top[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
  motor_top[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
  motor_top[index].temp = rx_data[6];

  // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
  if (motor_top[index].ecd - motor_top[index].last_ecd > 4096)
    motor_top[index].total_round--;
  else if (motor_top[index].ecd - motor_top[index].last_ecd < -4096)
    motor_top[index].total_round++;
  motor_top[index].total_angle = motor_top[index].total_round * 360 + motor_top[index].angle_single_round;
}