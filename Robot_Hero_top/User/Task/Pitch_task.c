/*
*************pitch轴任务**************
采用3508，ID = 7，CAN2，motor_can2[2]
遥控器控制：右拨杆上下
*/

#include "Pitch_task.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "ins_task.h"

extern motor_info_t motor_can2[3];
extern RC_ctrl_t rc_ctrl;
pitch_t pitch;
float relative_pitch = 0;
extern INS_t INS_bottom;

void Pitch_task(void const *argument)
{
    pitch_loop_init();

    for (;;)
    {
        // pitch_position_limit();
        pitch.target_speed = pitch_speed_map(rc_ctrl.rc.ch[1], -660, 660, -pitch.speed_max, pitch.speed_max);
        pitch_position_limit();
        pitch_current_give();
        osDelay(1);
    }
}

/****************初始化****************/
void pitch_loop_init()
{
    pitch.speed_pid_value[0] = 3;
    pitch.speed_pid_value[1] = 0;
    pitch.speed_pid_value[2] = 0;

    pitch.target_speed = 0;
    pitch.speed_max = 4000;

    pid_init(&pitch.pid_speed, pitch.speed_pid_value, 2000, 5000);
}

/********************************can1发送电流***************************/
void pitch_can2_cmd(int16_t v3)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = 0x1FF;
    tx_header.IDE = CAN_ID_STD;   // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧

    tx_header.DLC = 8; // 发送数据长度（字节）

    tx_data[0] = NULL;
    tx_data[1] = NULL;
    tx_data[2] = NULL;
    tx_data[3] = NULL;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3) & 0xff;
    tx_data[6] = NULL;
    tx_data[7] = NULL;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/****************PID计算速度并发送电流***************/
void pitch_current_give()
{
    motor_can2[2].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, -motor_can2[2].rotor_speed);
    pitch_can2_cmd(-motor_can2[2].set_current);
}

/*************pitch速度映射********************/
int16_t pitch_speed_map(int value, int from_min, int from_max, int to_min, int to_max)
{
    // 首先将输入值从 [a, b] 映射到 [0, 1] 范围内
    double normalized_value = (value * 1.0 - from_min) / (from_max - from_min);

    // 然后将标准化后的值映射到 [C, D] 范围内
    int16_t mapped_value = (int16_t)(to_min + (to_max - to_min) * normalized_value);

    return mapped_value;
}

/***************判断pitch位置******************/
void pitch_position_limit()
{
    relative_pitch = INS.Roll - INS_bottom.Roll;
    if (relative_pitch > 20 && pitch.target_speed > 0)
    {
        pitch.target_speed = 0;
    }
    if (relative_pitch < -15 && pitch.target_speed < 0)
    {
        pitch.target_speed = 0;
    }
}