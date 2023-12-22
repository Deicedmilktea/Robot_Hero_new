#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "rc_potocal.h"
extern UART_HandleTypeDef huart6;
JUDGE_MODULE_DATA Judge_Hero;
ext_power_heat_data_t powerd;
float newpower;
extern uint8_t seq;
uint8_t Hero_level;
uint8_t Hero_id;
uint16_t Hero_42mm_speed_limit;
uint16_t Hero_chassis_power_limit;
fp32 Hero_42mm_speed;

extern float powerdata[4];
#define LEN_CMD_ID 2
#define LEN_FRAME_HEAD 5
uint8_t first_line[30] = {"spin:"};
uint8_t CliendTxBuffer[200];


void Update_data();//定义一些需要用到的变量并实时更新数值方便其他文件调用
void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length)
{
    uint8_t pos=0;
    uint16_t data_length=0;
    uint16_t CMD_ID =0;
    
     while(pos<length)
     {
        if(databuffer[pos]==0xA5)
        {
            if(Verify_CRC8_Check_Sum(&databuffer[pos],5))
            {
                data_length = (databuffer[pos+1]&0xff)|((databuffer[pos+2]<<8)&0xff00);
                if(pos+data_length+9>length)
                {
                    continue;
                }
            if(Verify_CRC16_Check_Sum(&databuffer[pos],data_length+9))
            {
              
             
                CMD_ID = (databuffer[pos+5]&0xff)|((databuffer[pos+6]<<8)&0xff00);
                switch(CMD_ID)
                { 
                    case 0x0001:
                        data_length = 11;
                        memcpy((void*)(&Judge_Hero.status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Hero.robot_hp), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0005:
                        data_length = 13;
                          memcpy((void*)(&Judge_Hero.zone), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.event_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Hero.supply_status), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0104 :
                        data_length = 2;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 27;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length);                    //底盘功率限制上限在这
                     
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length);        //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.aerial_energy), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Hero.robot_hurt), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Hero.shoot_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.bullet_remain), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    default:break;
                }
                pos+=(data_length+9);
                continue;

            }
          }
        }
        pos++;
     
     }
		 Update_data();
}


void Update_data()
{
	Hero_id = Judge_Hero.robot_status.robot_id;//ID号
	Hero_level = Judge_Hero.robot_status.robot_level;//等级
	Hero_42mm_speed_limit = Judge_Hero.robot_status.shooter_id1_42mm_speed_limit;
	Hero_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;
	newpower = powerd.chassis_power;
	if(Judge_Hero.shoot_data.bullet_speed)
	{
		Hero_42mm_speed = Judge_Hero.shoot_data.bullet_speed;
	}
}


