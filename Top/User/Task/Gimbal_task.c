/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 2
* 云台电机为motor_top[5] CAN1控制
* 遥控器控制：左拨杆左右
* 键鼠控制：鼠标左右滑动
*/

#include "Gimbal_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "drv_can.h"
#include "user_lib.h"
#include "stdbool.h"
#include "remote_control.h"
#include "video_control.h"

static gimbal_t gimbal_gyro; // gimbal gyro

// yaw_correct
static uint8_t Update_yaw_flag = 1;
static float imu_err_yaw = 0; // 记录yaw飘移的数值便于进行校正

extern CAN_HandleTypeDef hcan1;
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern INS_t INS;
extern bool vision_is_tracking;
extern float vision_yaw;

static void Gimbal_loop_init();				  // 初始化
static void angle_over_zero(float err);		  // 角度过零处理
static void gimbal_control();				  // 控制云台旋转
static void detel_calc(float *angle);		  // 角度范围限制
static void gimbal_can2_cmd(int16_t voltage); // can1发送电流
static void gimbal_mode_vision();			  // 视觉控制
static void gimbal_mode_normal();			  // 锁yaw

// 云台运动task
void Gimbal_task(void const *pvParameters)
{
	Gimbal_loop_init();

	for (;;)
	{
		// 遥控链路
		if (rc_ctrl[TEMP].rc.switch_left)
		{
			// 视觉控制
			if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r) // 左拨杆上 || 按住右键
			{
				gimbal_mode_vision();
			}

			// 锁yaw模式
			else // 左拨杆中或下
			{
				gimbal_mode_normal();
			}
		}

		// 图传链路
		else
		{
			// 视觉控制
			if (video_ctrl[TEMP].key_data.right_button_down) // 按住右键
			{
				gimbal_mode_vision();
			}

			// 锁yaw模式
			else // 左拨杆中或下
			{
				gimbal_mode_normal();
			}
		}

		osDelay(1);
	}
}

static void Gimbal_loop_init()
{
	gimbal_gyro.pid_angle_value[0] = 40;
	gimbal_gyro.pid_angle_value[1] = 0;
	gimbal_gyro.pid_angle_value[2] = 0;

	gimbal_gyro.pid_speed_value[0] = 100;
	gimbal_gyro.pid_speed_value[1] = 0;
	gimbal_gyro.pid_speed_value[2] = 0;

	// 视觉使用版本，防止yaw抖动过大
	gimbal_gyro.pid_angle_vision_value[0] = 150;
	gimbal_gyro.pid_angle_vision_value[1] = 0.01;
	gimbal_gyro.pid_angle_vision_value[2] = 800;

	gimbal_gyro.pid_speed_vision_value[0] = 10;
	gimbal_gyro.pid_speed_vision_value[1] = 0;
	gimbal_gyro.pid_speed_vision_value[2] = 0;

	gimbal_gyro.target_angle = 0;

	gimbal_gyro.pid_angle_out = 0;
	gimbal_gyro.pid_speed_out = 0;

	pid_init(&gimbal_gyro.pid_angle, gimbal_gyro.pid_angle_value, 30000, 30000);
	pid_init(&gimbal_gyro.pid_speed, gimbal_gyro.pid_speed_value, 30000, 30000);
	pid_init(&gimbal_gyro.pid_angle_vision, gimbal_gyro.pid_angle_vision_value, 30000, 30000);
	pid_init(&gimbal_gyro.pid_speed_vision, gimbal_gyro.pid_speed_vision_value, 30000, 30000);
}

/************************************ 角度过零处理 ********************************/
static void angle_over_zero(float err)
{
	if (err > 180) // 180 ：半圈机械角度
	{
		gimbal_gyro.target_angle -= 360;
	}
	else if (err < -180)
	{
		gimbal_gyro.target_angle += 360;
	}
}

/************************************读取yaw轴imu数据**************************************/
static void Yaw_read_imu()
{
	// 记录初始位置
	if (Update_yaw_flag)
	{
		Update_yaw_flag = 0; // 只进入一次
		INS.yaw_init = INS.Yaw - 0.0f;
		gimbal_gyro.target_angle = INS.yaw_init;
	}

	// // 顺时针旋转，陀螺仪飘 -90°/min
	// // 解决yaw偏移，完成校正
	// if (rc_ctrl[TEMP].rc.rocker_l_ > 50 || rc_ctrl[TEMP].mouse.x > 1500)
	// 	imu_err_yaw += 0.0015f;
	// if ((rc_ctrl[TEMP].rc.rocker_l_ < -50 || rc_ctrl[TEMP].mouse.x < -1500))
	// 	imu_err_yaw -= 0.0015f;

	// 校正
	INS.yaw_update = INS.Yaw - INS.yaw_init + imu_err_yaw;
}

/**************************** 视觉控制 **********************************/
static void gimbal_mode_vision()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// 遥控器链路
	if (rc_ctrl[TEMP].rc.switch_left)
	{
		// 如果追踪到目标
		if (vision_is_tracking)
		{
			// 视觉模式中加入手动微调
			float normalized_input = (rc_ctrl[TEMP].rc.rocker_l_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f) * 10.0f; // 最大微调角度限制为10°
			gimbal_gyro.target_angle = vision_yaw - normalized_input;
		}

		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = rc_ctrl[TEMP].rc.rocker_l_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f * 100.0f;
			gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	// 图传链路
	else
	{
		// 如果追踪到目标
		if (vision_is_tracking)
		{
			// 视觉模式中加入手动微调
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 10.0f; // 最大微调角度限制为10°
			gimbal_gyro.target_angle = vision_yaw - normalized_input;
		}

		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 100.0f;
			gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	detel_calc(&gimbal_gyro.target_angle);

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle_vision, gimbal_gyro.target_angle, INS.yaw_update);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed_vision, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);

	// 给电流
	gimbal_can2_cmd(gimbal_gyro.pid_speed_out); // 给电流
}

/**************************** 锁yaw **********************************/
static void gimbal_mode_normal()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// 遥控器链路
	if (rc_ctrl[TEMP].rc.switch_left)
	{
		// 使用非线性映射函数调整灵敏度
		float normalized_input = rc_ctrl[TEMP].rc.rocker_l_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f * 100;
		gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.97) * sign(normalized_input) * 0.3;
	}

	// 图传链路
	else
	{
		// 使用非线性映射函数调整灵敏度
		float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 50;
		gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
	}

	detel_calc(&gimbal_gyro.target_angle);

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle, gimbal_gyro.target_angle, INS.yaw_update);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);

	// 给电流
	gimbal_can2_cmd(gimbal_gyro.pid_speed_out);
}

/*****************************角度范围限制**********************************/
static void detel_calc(float *angle)
{
	// 如果角度大于180度，则减去360度
	if (*angle > 180)
	{
		*angle -= 360;
	}

	// 如果角度小于-180度，则加上360度
	else if (*angle < -180)
	{
		*angle += 360;
	}
}

/********************************can2发送电流***************************/
static void gimbal_can2_cmd(int16_t voltage)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];

	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;	  // 标准帧
	tx_header.RTR = CAN_RTR_DATA; // 数据帧

	tx_header.DLC = 8; // 发送数据长度（字节）

	// tx_data[0] = NULL;
	// tx_data[1] = NULL;
	// tx_data[2] = (voltage >> 8) & 0xff;
	// tx_data[3] = (voltage) & 0xff;
	// tx_data[4] = NULL;
	// tx_data[5] = NULL;
	// tx_data[6] = NULL;
	// tx_data[7] = NULL;
	tx_data[0] = NULL;
	tx_data[1] = NULL;
	tx_data[2] = NULL;
	tx_data[3] = NULL;
	tx_data[4] = NULL;
	tx_data[5] = NULL;
	tx_data[6] = (voltage >> 8) & 0xff;
	tx_data[7] = (voltage) & 0xff;

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}