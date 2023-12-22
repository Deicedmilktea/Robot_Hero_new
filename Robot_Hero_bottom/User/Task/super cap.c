#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"-
#include "super_cap.h"
#include  "drv_can.h"
#include "judge.h"
extern uint8_t Hero_level;
#include "Chassis_task.h"
extern int flag[1];
extern float powerdata[4];

void super_cap(void const *pvParameters)//≥¨º∂µÁ»›
	

{
for(;;)
    {  
if(Hero_level==1)
{
	supercap(5500);
}
if(Hero_level==2)
{
	supercap(6000);
}
if(Hero_level==3)
{
	supercap(6500);
}
else
{

	supercap(5500);
}

		//can_remote(sbuss_buf,0x33);
		//CAN_rc_forward(power,power1);
			osDelay(1);
		}

}
