/*
*************pitch轴任务**************
采用3508，ID = 7，CAN2，motor_top[2]
遥控器控制：左遥杆上下
*/

#include "Pitch_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "remote_control.h"

#define PITCH_MAX 40
#define PITCH_MIN 0

pitch_t pitch;

extern motor_info_t motor_top[4];
extern RC_ctrl_t rc_ctrl[2];
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
        // 在非平地起作用，使用相对角度保证软件限位依旧有效
        pitch.relative_pitch = INS.Roll - INS_bottom.Roll;

        // 视觉识别，右拨杆上/鼠标右键
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r == 1)
        {
            // 视觉模式下的手动微调
            pitch.vision_manual_pitch = (rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
            pitch.vision_target_pitch = pitch.vision_manual_pitch + vision_pitch;

            if (pitch.vision_target_pitch > PITCH_MAX)
            {
                pitch.vision_target_pitch = PITCH_MAX;
            }
            if (pitch.vision_target_pitch < PITCH_MIN)
            {
                pitch.vision_target_pitch = PITCH_MIN;
            }

            pitch.target_speed = -pid_calc(&pitch.vision_pid_angle, pitch.vision_target_pitch, INS.Roll);

            // target_speed 的计算必须加上负号（想要符合给正值抬头，负值低头的话），与3508的旋转方向相关，否则pitch会疯转
        }

        else
        {
            pitch.target_speed = -(rc_ctrl[TEMP].rc.rocker_l1 / 660.0f * pitch.speed_max - 200 * rc_ctrl[TEMP].mouse.y / 16384.0f * pitch.speed_max);
            pitch_position_limit();
        }

        // pitch_current_give();
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
    if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r == 1)
    {
        motor_top[2].set_current = pid_calc(&pitch.vision_pid_speed, pitch.target_speed, motor_top[2].rotor_speed);
    }

    else
    {
        motor_top[2].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, motor_top[2].rotor_speed);
    }

    pitch_can2_cmd(motor_top[2].set_current);
}

/***************判断pitch位置******************/
static void pitch_position_limit()
{
    if (pitch.relative_pitch > PITCH_MAX && pitch.target_speed < 0) // pitch.target_speed正负与3508旋转方向有关
    {
        pitch.target_speed = 0;
    }
    if (pitch.relative_pitch < PITCH_MIN && pitch.target_speed > 0)
    {
        pitch.target_speed = 0;
    }
}