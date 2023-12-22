#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern INS_t INS;

ins_data_t ins_data;

void exchange_task()
{
	while(1){
	ins_data.angle[0]=INS.Yaw;
	ins_data.angle[1]=INS.Roll;
	ins_data.angle[2]=INS.Pitch;
	
	osDelay(1);
	}
}