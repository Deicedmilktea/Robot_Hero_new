#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"

JUDGE_MODULE_DATA Judge_Hero;

uint8_t Hero_level;
uint8_t Hero_id;
uint16_t Hero_42mm_speed_limit;
uint16_t Hero_chassis_power_limit;
uint16_t Hero_chassis_power_buffer;
float Hero_chassis_power;
float Hero_42mm_speed;
uint8_t Flag_progress;
uint8_t Flag_judge = 0;
uint8_t Flag_first = 0;
// extern uint8_t first_x;
// extern uint8_t first_y;
extern CAN_HandleTypeDef hcan1;

void Update_data(); // 定义一些需要用到的变量并实时更新数值方便其他文件调用

void JUDGE_Receive(volatile uint8_t *databuffer, uint8_t length)
{
    uint8_t pos = 0;
    uint16_t data_length = 0;
    uint16_t CMD_ID = 0;

    while (pos < length)
    {
        if (databuffer[pos] == 0xA5)
        {
            if (Verify_CRC8_Check_Sum(&databuffer[pos], 5))
            {
                data_length = (databuffer[pos + 1] & 0xff) | ((databuffer[pos + 2] << 8) & 0xff00);
                if (pos + data_length + 9 > length)
                {
                    continue;
                }
                if (Verify_CRC16_Check_Sum(&databuffer[pos], data_length + 9))
                {

                    CMD_ID = (databuffer[pos + 5] & 0xff) | ((databuffer[pos + 6] << 8) & 0xff00);
                    switch (CMD_ID)
                    {
                    case 0x0001:
                        data_length = 11;
                        memcpy((void *)(&Judge_Hero.status), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                        memcpy((void *)(&Judge_Hero.robot_hp), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0005:
                        data_length = 13;
                        memcpy((void *)(&Judge_Hero.zone), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void *)(&Judge_Hero.event_data), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                        memcpy((void *)(&Judge_Hero.supply_status), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0104:
                        data_length = 2;
                        memcpy((void *)(&Judge_Hero.warning), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0105:
                        data_length = 1;
                        memcpy((void *)(&Judge_Hero.dart_remaining_time), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0201:
                        data_length = 27;
                        memcpy((void *)(&Judge_Hero.robot_status), (const void *)(&databuffer[pos + 7]), data_length); // 底盘功率限制上限在这
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void *)(&Judge_Hero.power_heat), (const void *)(&databuffer[pos + 7]), data_length); // 含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                        memcpy((void *)(&Judge_Hero.robot_pos), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void *)(&Judge_Hero.buff), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void *)(&Judge_Hero.aerial_energy), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0206:
                        data_length = 1;
                        memcpy((void *)(&Judge_Hero.robot_hurt), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void *)(&Judge_Hero.shoot_data), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void *)(&Judge_Hero.bullet_remain), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void *)(&Judge_Hero.rfid_status), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void *)(&Judge_Hero.rfid_status), (const void *)(&databuffer[pos + 7]), data_length);
                        break;
                    default:
                        break;
                    }
                    pos += (data_length + 9);
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
    Hero_id = Judge_Hero.robot_status.robot_id;                                   // ID号
    Hero_level = Judge_Hero.robot_status.robot_level;                             // 等级
    Hero_42mm_speed_limit = Judge_Hero.robot_status.shooter_id1_42mm_speed_limit; // 42mm弹丸射速限制
    Hero_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;       // 功率限制
    Hero_chassis_power_buffer = Judge_Hero.power_heat.chassis_power_buffer;       // 缓冲能量
    Hero_chassis_power = Judge_Hero.power_heat.chassis_power;                     // 实时功率
    if (Judge_Hero.shoot_data.bullet_speed)
    {
        Hero_42mm_speed = Judge_Hero.shoot_data.bullet_speed;
    }
    // 比赛进程
    Flag_progress = Judge_Hero.status.game_progress;
    if (Flag_progress == 4 && Flag_first == 0) // 比赛开始
    {
        //		first_x = 1;
        //		first_y = 1;
        Flag_first = 1;
        //		HAL_TIM_Base_Start_IT(&htim8);
    }
    // 判断我方是红方还是蓝方
    if (Hero_id == 1) // 红色方
    {
        Flag_judge = 1;
    }
    else if (Hero_id == 101)
    {
        Flag_judge = 2;
    }

    // 判断是红方哨兵还是蓝方,掉血强制开启旋转模式
    if (Flag_judge == 1) // 红色方
    {
        if (Judge_Hero.robot_hp.red_1_robot_HP != 0 && Judge_Hero.robot_hp.red_1_robot_HP != 600)
        {
            Flag_first = 2;
        }
    }

    else if (Flag_judge == 2) // 蓝色方
    {
        if (Judge_Hero.robot_hp.blue_1_robot_HP != 0 && Judge_Hero.robot_hp.blue_1_robot_HP != 600)
        {
            Flag_first = 2;
        }
    }

    // // 发送给上C板
    // uint8_t temp_remote[2];
    // temp_remote[0] = Flag_progress;
    // temp_remote[1] = Flag_judge;

    // CAN_TxHeaderTypeDef tx_header;

    // tx_header.StdId = 0x10;
    // tx_header.IDE = CAN_ID_STD;   // 标准帧
    // tx_header.RTR = CAN_RTR_DATA; // 数据帧
    // tx_header.DLC = 2;            // 发送数据长度（字节）

    // HAL_CAN_AddTxMessage(&hcan1, &tx_header, temp_remote, (uint32_t *)CAN_TX_MAILBOX0);
}