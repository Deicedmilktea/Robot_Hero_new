/*
*************pitch轴任务**************
* 采用6020，ID = 2
* motor_top[5] CAN2
* 遥控器控制：左遥杆上下
* 键鼠控制：鼠标上下滑动
*/

#include "Pitch_task.h"
#include "drv_can.h"
#include "remote_control.h"
#include "video_control.h"
#include "ins_task.h"
#include "stdbool.h"
#include "user_lib.h"

#define PITCH_MAX 38
#define PITCH_MIN -8

static pitch_t pitch;

extern CAN_HandleTypeDef hcan2;
extern motor_info_t motor_top[7];
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern INS_t INS_bottom;
extern float vision_pitch;
extern bool vision_is_tracking;
extern uint8_t is_remote_online;
extern uint8_t is_gimbal_on;

static void pitch_loop_init();               // 初始化
static void pitch_can2_cmd(int16_t voltage); // can1发送电流
static void pitch_current_give();            // PID计算速度并发送电流
static void pitch_position_limit();          // 限制pitch位置

void Pitch_task(void const *argument)
{
    pitch_loop_init();

    for (;;)
    {
        // 在非平地起作用，使用相对角度保证软件限位依旧有效
        pitch.relative_pitch = INS.Roll - INS_bottom.Pitch;

        // 遥控器链路
        if (is_remote_online)
        {
            // 视觉识别，右拨杆上/鼠标右键
            if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r == 1)
            {
                if (vision_is_tracking)
                {
                    // 视觉模式下的手动微调
                    float normalized_input = (rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f * 10.0f) * 10.0f; // 微调幅度10°
                    pitch.target_angle = vision_pitch + normalized_input;
                }
                else
                {
                    // 使用非线性映射函数调整灵敏度
                    float normalized_input = rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f * 100.0f;
                    pitch.target_angle += pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.1f;
                }
            }

            else
            {
                // 使用非线性映射函数调整灵敏度
                float normalized_input = rc_ctrl[TEMP].rc.rocker_l1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f * 100.0f;
                pitch.target_angle += pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.1f;
            }
        }

        // 图传链路
        else
        {
            // 视觉识别，鼠标右键
            if (video_ctrl[TEMP].key_data.right_button_down == 1)
            {
                if (vision_is_tracking)
                {
                    // 视觉模式下的手动微调
                    float normalized_input = video_ctrl[TEMP].key_data.mouse_y / 16384.0f * 100.0f;
                    pitch.target_angle = vision_pitch + normalized_input;
                }

                else
                {
                    // 使用非线性映射函数调整灵敏度
                    float normalized_input = video_ctrl[TEMP].key_data.mouse_y / 16384.0f * 10.0f;
                    pitch.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input);
                }
            }

            else
            {
                // 使用非线性映射函数调整灵敏度
                float normalized_input = video_ctrl[TEMP].key_data.mouse_y / 16384.0f * 10.0f;
                pitch.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input);
            }
        }

        pitch_position_limit();
        pitch_current_give();
        osDelay(1);
    }
}

/*************************** 初始化 *************************/
static void pitch_loop_init()
{
    pitch.angle_pid_value[0] = 50;
    pitch.angle_pid_value[1] = 0;
    pitch.angle_pid_value[2] = 100;

    pitch.speed_pid_value[0] = 500;
    pitch.speed_pid_value[1] = 0.05;
    pitch.speed_pid_value[2] = 0;

    pitch.target_angle = 0;
    pitch.target_speed = 0;

    pid_init(&pitch.pid_angle, pitch.angle_pid_value, 16384, 16384);
    pid_init(&pitch.pid_speed, pitch.speed_pid_value, 16384, 16384);
}

/**************** PID计算速度并发送电流 ***************/
static void pitch_current_give()
{
    pitch.target_speed = pid_calc(&pitch.pid_angle, pitch.target_angle, INS.Roll);
    motor_top[5].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, INS.Gyro[1] * 57.3f); // 此处速度需具体测量
    // 在shoot_task.c中发送电流，防止一下发多个电流给电机
}

/*************** 判断pitch位置 ******************/
static void pitch_position_limit()
{
    if ((pitch.target_angle > PITCH_MAX + INS_bottom.Pitch) && (pitch.target_speed > 0))
    {
        pitch.target_angle = PITCH_MAX + INS_bottom.Pitch;
        pitch.target_speed = 0;
    }
    if ((pitch.target_angle < PITCH_MIN + INS_bottom.Pitch) && (pitch.target_speed < 0))
    {
        pitch.target_angle = PITCH_MIN + INS_bottom.Pitch;
        pitch.target_speed = 0;
    }
}