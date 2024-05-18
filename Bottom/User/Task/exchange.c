#include "struct_typedef.h"
#include "cmsis_os.h"
#include "exchange.h"
#include "ins_task.h"
#include "drv_can.h"
#include "referee_task.h"

extern INS_t INS;

static uint8_t send_buffer[8];
static int16_t ins_pitch = 0;

void exchange_task()
{
	while (1)
	{
		ins_pitch = 50 * INS.Roll;
		send_buffer[0] = (ins_pitch >> 8) & 0xff;
		send_buffer[1] = ins_pitch & 0xff;
		send_buffer[2] = 0;
		send_buffer[3] = 0;
		send_buffer[4] = 0;
		send_buffer[5] = 0;
		send_buffer[6] = 0;
		send_buffer[7] = 0;
		can_remote(send_buffer, 0x55);

		osDelay(2);
	}
}