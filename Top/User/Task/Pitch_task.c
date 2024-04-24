/*
*************pitch轴任务**************
* 采用6020，ID = 1
* motor_top[4] CAN2
* 遥控器控制：左遥杆上下
* 键鼠控制：鼠标上下滑动
*/

#include "Pitch_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "remote_control.h"
#include "video_control.h"
#include "stdbool.h"

#define PITCH_MAX 40
#define PITCH_MIN 0

pitch_t pitch;

extern CAN_HandleTypeDef hcan2;
extern motor_info_t motor_top[6];
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern INS_t INS_bottom;
extern float vision_pitch;
extern bool vision_is_tracking;

// 初始化
static void pitch_loop_init();

/*can1发送电流*/
static void pitch_can2_cmd(int16_t voltage);

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
        pitch.relative_pitch = INS.Roll - INS_bottom.Pitch;

#ifdef REMOTE_CONTROL
        // 视觉识别，右拨杆上/鼠标右键
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r == 1)
        {
            if (vision_is_tracking)
            {
                // 视觉模式下的手动微调
                float normalized_input = (rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
                pitch.target_angle = vision_pitch + normalized_input;
            }
        }

        else
        {
            // 使用非线性映射函数调整灵敏度
            float normalized_input = (rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
            pitch.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
        }
#endif

#ifdef VIDEO_CONTROL
        // 视觉识别，鼠标右键
        if (video_ctrl[TEMP].key_data.right_button_down == 1)
        {
            if (vision_is_tracking)
            {
                // 视觉模式下的手动微调
                float normalized_input = (video_ctrl[TEMP].key_data.mouse_y / 16384.0f) * 100.0f;
                pitch.target_angle = vision_pitch + normalized_input;
            }
        }

        else
        {
            // 使用非线性映射函数调整灵敏度
            float normalized_input = (rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
            pitch.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
        }
#endif

        pitch_position_limit();
        // pitch_current_give();
        osDelay(1);
    }
}

/*************************** 初始化 *************************/
static void pitch_loop_init()
{
    pitch.speed_pid_value[0] = 3;
    pitch.speed_pid_value[1] = 0;
    pitch.speed_pid_value[2] = 0;

    pitch.angle_pid_value[0] = 3;
    pitch.angle_pid_value[1] = 0;
    pitch.angle_pid_value[2] = 0;

    pid_init(&pitch.pid_speed, pitch.speed_pid_value, 10000, 30000);
    pid_init(&pitch.pid_angle, pitch.angle_pid_value, 10000, 30000);
}

/************************** can1发送电流 ***************************/
static void pitch_can2_cmd(int16_t voltage)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = 0x1FF;
    tx_header.IDE = CAN_ID_STD;   // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧

    tx_header.DLC = 8; // 发送数据长度（字节）

    tx_data[0] = (voltage >> 8) & 0xff;
    tx_data[1] = (voltage) & 0xff;
    tx_data[2] = NULL;
    tx_data[3] = NULL;
    tx_data[4] = NULL;
    tx_data[5] = NULL;
    tx_data[6] = NULL;
    tx_data[7] = NULL;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/**************** PID计算速度并发送电流 ***************/
static void pitch_current_give()
{
    pitch.pid_angle_out = pid_calc(&pitch.pid_angle, pitch.target_angle, INS.Roll);
    pitch.pid_speed_out = pid_calc(&pitch.pid_speed, pitch.pid_angle_out, INS.Gyro[1] * 57.3f); // 此处速度需具体测量

    pitch_can2_cmd(pitch.pid_speed_out);
}

/*************** 判断pitch位置 ******************/
static void pitch_position_limit()
{
    if (pitch.relative_pitch > PITCH_MAX)
    {
        pitch.target_angle = PITCH_MAX + INS_bottom.Pitch;
    }
    if (pitch.relative_pitch < PITCH_MIN)
    {
        pitch.target_angle = PITCH_MIN + INS_bottom.Pitch;
    }
}