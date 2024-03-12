/*
*************pitch轴任务**************
采用3508，ID = 7，CAN2，motor_can2[2]
遥控器控制：左遥杆上下
*/

#include "Pitch_task.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "ins_task.h"

extern motor_info_t motor_can2[4];
extern RC_ctrl_t rc_ctrl;
pitch_t pitch;
float relative_pitch = 0;
extern INS_t INS_bottom;
extern float vision_pitch;

// 初始化
static void pitch_loop_init();

/*can1发送电流*/
static void pitch_can2_cmd(int16_t v3);

// PID计算速度并发送电流
static void pitch_current_give();

// 判断pitch位置
static void pitch_position_limit();

void Pitch_task(void const *argument)
{
    pitch_loop_init();

    for (;;)
    {
        relative_pitch = INS.Roll - INS_bottom.Roll;

        // 视觉识别，右拨杆上/鼠标右键
        if (rc_ctrl.rc.s[0] == 1 || press_right == 1)
        {
            // 视觉模式下的遥控器微调
            pitch.vision_remote_pitch += (rc_ctrl.rc.ch[3] / 660.0f - rc_ctrl.mouse.y / 16384.0f * 50) * 0.1;
            pitch.vision_target_pitch = pitch.vision_remote_pitch + vision_pitch;

            if (pitch.vision_target_pitch > 20)
            {
                pitch.vision_target_pitch = 20;
            }
            if (pitch.vision_target_pitch < -15)
            {
                pitch.vision_target_pitch = -15;
            }

            pitch.target_speed = -pid_calc(&pitch.vision_pid_angle, pitch.vision_target_pitch, INS.Roll);

            // target_speed 的计算必须加上负号（想要符合给正值抬头，负值低头的话），与3508的旋转方向相关，否则pitch会疯转
        }

        else
        {
            pitch.target_speed = -(rc_ctrl.rc.ch[3] / 660.0f * pitch.speed_max - 200 * rc_ctrl.mouse.y / 16384.0f * pitch.speed_max);
            pitch_position_limit();
        }

        pitch_current_give();
        osDelay(1);
    }
}

/****************初始化****************/
static void pitch_loop_init()
{
    pitch.speed_pid_value[0] = 3;
    pitch.speed_pid_value[1] = 0;
    pitch.speed_pid_value[2] = 0;

    pitch.angle_pid_value[0] = 3;
    pitch.angle_pid_value[1] = 0;
    pitch.angle_pid_value[2] = 0;

    pitch.vision_speed_pid_value[0] = 20;
    pitch.vision_speed_pid_value[1] = 0;
    pitch.vision_speed_pid_value[2] = 0;

    pitch.vision_angle_pid_value[0] = 400;
    pitch.vision_angle_pid_value[1] = 0;
    pitch.vision_angle_pid_value[2] = 0;

    pitch.target_speed = 0;
    pitch.speed_max = 4000;

    pid_init(&pitch.pid_speed, pitch.speed_pid_value, 1000, 4000);
    pid_init(&pitch.pid_angle, pitch.angle_pid_value, 1000, 4000);
    pid_init(&pitch.vision_pid_speed, pitch.vision_speed_pid_value, 1000, 4000);
    pid_init(&pitch.vision_pid_angle, pitch.vision_angle_pid_value, 1000, 4000);
}

/********************************can1发送电流***************************/
static void pitch_can2_cmd(int16_t v3)
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
static void pitch_current_give()
{
    if (rc_ctrl.rc.s[1] == 2)
    {
        motor_can2[2].set_current = pid_calc(&pitch.vision_pid_speed, pitch.target_speed, motor_can2[2].rotor_speed);
    }

    else
    {
        motor_can2[2].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, motor_can2[2].rotor_speed);
    }

    pitch_can2_cmd(motor_can2[2].set_current);
}

/***************判断pitch位置******************/
static void pitch_position_limit()
{
    if (relative_pitch > 20 && pitch.target_speed < 0) // pitch.target_speed正负与3508旋转方向有关
    {
        pitch.target_speed = 0;
    }
    if (relative_pitch < -15 && pitch.target_speed > 0)
    {
        pitch.target_speed = 0;
    }
}