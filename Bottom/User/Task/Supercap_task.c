#include "cmsis_os.h"
#include "Supercap_task.h"
#include "usart.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern uint16_t Hero_chassis_power_limit;
extern uint16_t Hero_chassis_power_buffer;
extern uint8_t Hero_level;

static void supercap_can_transmit(int16_t poooower);

// int superop = 0;

void Supercap_task(void const *argument)
{
	while (1)
	{
		// uint8_t iuy[7] = "P055P\r\n";
		// int power = (int)Hero_chassis_power_limit;
		// if (power == 55)
		// 	strcpy(iuy, "P055P\r\n");
		// else if (power == 60)
		// 	strcpy(iuy, "P060P\r\n");
		// else if (power == 65)
		// 	strcpy(iuy, "P065P\r\n");
		// else if (power == 70)
		// 	strcpy(iuy, "P070P\r\n");
		// else if (power == 75)
		// 	strcpy(iuy, "P075P\r\n");
		// else if (power == 80)
		// 	strcpy(iuy, "P080P\r\n");
		// else if (power == 85)
		// 	strcpy(iuy, "P085P\r\n");
		// else if (power == 90)
		// 	strcpy(iuy, "P090P\r\n");
		// else if (power == 100 || power == 120)
		// 	strcpy(iuy, "P100P\r\n");
		// else
		// 	strcpy(iuy, "P055P\r\n");

		// uint8_t scon[7] = "PVONP\r\n";
		// uint8_t scoff[7] = "PVOFP\r\n";

		// HAL_UART_Transmit(&huart1, (uint8_t *)iuy, 7, 0xff);

		// osDelay(10);

		switch (Hero_level)
		{
		case 1:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(5500);
			else
				supercap_can_transmit(6500);
		case 2:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(6000);
			else
				supercap_can_transmit(7000);
		case 3:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(6500);
			else
				supercap_can_transmit(7500);
		case 4:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(7000);
			else
				supercap_can_transmit(8000);
		case 5:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(7500);
			else
				supercap_can_transmit(9000);
		case 6:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(8000);
			else
				supercap_can_transmit(9500);
		case 7:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(8500);
			else
				supercap_can_transmit(10500);
		case 8:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(9000);
			else
				supercap_can_transmit(11000);
		case 9:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(10000);
			else
				supercap_can_transmit(13000);
		case 10:
			if (Hero_chassis_power_buffer < 10)
				supercap_can_transmit(12000);
			else
				supercap_can_transmit(13000);
		}

		osDelay(1);
	}
}

static void supercap_can_transmit(int16_t poooower)
{
	uint32_t CAN_TX_MAILBOX01;
	CAN_TxHeaderTypeDef tx_header;

	uint8_t power[2];
	tx_header.StdId = 0x210;
	tx_header.IDE = CAN_ID_STD;	  // 标准帧
	tx_header.RTR = CAN_RTR_DATA; // 数据帧

	tx_header.DLC = 2; // 发送数据长度（字节）
	power[0] = poooower >> 8;
	power[1] = poooower;
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, power, (uint32_t *)CAN_TX_MAILBOX0);
}