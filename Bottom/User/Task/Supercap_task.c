#include "cmsis_os.h"
#include "Supercap_task.h"
#include "usart.h"
#include "string.h"
#include "rm_referee.h"
#include "remote_control.h"
#include "video_control.h"

SupercapTxData_t SupercapTxData;
uint8_t supercap_mode = 0;

extern CAN_HandleTypeDef hcan2;
extern referee_hero_t referee_hero;
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern uint8_t supercap_flag;
extern uint8_t is_remote_online;

static void supercap_can_transmit(SupercapTxData_t TxData);
static void supercap_data_set(uint16_t buffer, uint16_t power, uint8_t state);
static void read_keyboard();

void Supercap_task(void const *argument)
{
	while (1)
	{
		read_keyboard();
		supercap_data_set(referee_hero.buffer_energy, referee_hero.chassis_power_limit, supercap_mode);
		supercap_can_transmit(SupercapTxData);
		osDelay(10);
	}
}

static void supercap_can_transmit(SupercapTxData_t TxData)
{
	CAN_TxHeaderTypeDef tx_header;

	uint8_t send_buffer[5];
	tx_header.StdId = 0x302;
	tx_header.IDE = CAN_ID_STD;	  // 标准帧
	tx_header.RTR = CAN_RTR_DATA; // 数据帧

	tx_header.DLC = 5; // 发送数据长度（字节）
	send_buffer[0] = (TxData.buffer >> 8) & 0xff;
	send_buffer[1] = TxData.buffer & 0xff;
	send_buffer[2] = (TxData.power >> 8) & 0xff;
	send_buffer[3] = TxData.power & 0xff;
	send_buffer[4] = TxData.state;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) // 等待发送邮箱空闲
	{
	}
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_buffer, (uint32_t *)CAN_TX_MAILBOX0);
}

static void supercap_data_set(uint16_t buffer, uint16_t power, uint8_t state)
{
	SupercapTxData.buffer = buffer;
	SupercapTxData.power = power;
	SupercapTxData.state = state;
}

static void read_keyboard()
{
	if (is_remote_online)
	{
		switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_C] % 2)
		{
		case 1:
			supercap_mode = SUPERCAP_STATE_OFF;
			break;
		default:
			supercap_mode = SUPERCAP_STATE_AUTO;
			break;
		}
	}

	else
	{
		switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_C] % 2)
		{
		case 1:
			supercap_mode = SUPERCAP_STATE_OFF;
			break;
		default:
			supercap_mode = SUPERCAP_STATE_AUTO;
			break;
		}
	}
}