#include "cmsis_os.h"
#include "Supercap_task.h"
#include "usart.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

void Supercap_task(void const *argument)
{
	for (;;)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)"P055P\r\n", strlen("P055P\r\n"), HAL_MAX_DELAY);
		osDelay(1000);
		// osDelay(1);
	}
}