#include "cmsis_os.h"
#include "Supercap_task.h"
#include "usart.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern uint16_t Hero_chassis_power_limit;
extern uint16_t Hero_chassis_power_buffer;

int superop = 0;

void Supercap_task(void const *argument)
{
	while (1)
	{
		uint8_t iuy[7] = "P060P\r\n";
		int power = (int)Hero_chassis_power_limit;
		if (power == 55)
			strcpy(iuy, "P055P\r\n");
		else if (power == 60)
			strcpy(iuy, "P060P\r\n");
		else if (power == 65)
			strcpy(iuy, "P065P\r\n");
		else if (power == 70)
			strcpy(iuy, "P070P\r\n");
		else if (power == 75)
			strcpy(iuy, "P075P\r\n");
		else if (power == 80)
			strcpy(iuy, "P080P\r\n");
		else if (power == 85)
			strcpy(iuy, "P085P\r\n");
		else if (power == 90)
			strcpy(iuy, "P090P\r\n");
		else if (power == 100 || power == 120)
			strcpy(iuy, "P100P\r\n");
		else
			strcpy(iuy, "P055P\r\n");

		uint8_t scon[7] = "PVONP\r\n";
		uint8_t scoff[7] = "PVOFP\r\n";
		char buffer[20];

		if (Hero_chassis_power_buffer < 5)
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)scon, 7, 0xff);
			superop = 1;
		}
		else
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)scoff, 7, 0xff);
			superop = 0;
		}

		HAL_UART_Transmit(&huart1, (uint8_t *)iuy, 7, 0xff);

		osDelay(10);
	}
}