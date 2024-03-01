/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 4
* 云台电机为motor_can2[3] // CAN1控制
* 遥控器控制：右拨杆左右
*/

#include "Gimbal_task.h"
#include "rc_potocal.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "drv_can.h"
#include "user_lib.h"
#include "stdbool.h"

// motor data read
#define get_motor_measure(ptr, data)                                   \
	{                                                                  \
		(ptr)->last_ecd = (ptr)->ecd;                                  \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->temperate = (data)[6];                                  \
	}

gimbal_t gimbal_encoder; // gimbal encoder
gimbal_t gimbal_gyro;	 // gimbal gyro
fp32 err_yaw_angle;		 // yaw angle error
extern RC_ctrl_t rc_ctrl;
extern INS_t INS;
extern motor_info_t motor_can2[4];
extern bool vision_is_tracking;

int gimbal_mode = 0; // 记录模式，0为编码器，1为陀螺仪
int error6 = 0;

fp32 ins_yaw;
fp32 ins_yaw_update = 0;
fp32 Driftring_yaw = 0;
fp32 ins_pitch;
fp32 ins_roll;
fp32 init_yaw; // 记录yaw初始量
int Update_yaw_flag = 1;

extern float vision_yaw;

/********************云台运动task*********************/
void Gimbal_task(void const *pvParameters)
{
	Gimbal_loop_init();
	gimbal_mode = 1;

	if (gimbal_mode == 0)
	{
		gimbal_encoder.target_angle = motor_can2[3].rotor_angle;
	}

	for (;;)
	{
		error6++;
		gimbal_control();
		osDelay(1);
	}
}

void Gimbal_loop_init()
{
	// Kp, Ki, Kd
	gimbal_encoder.pid_angle_value[0] = 5;
	gimbal_encoder.pid_angle_value[1] = 0.5;
	gimbal_encoder.pid_angle_value[2] = 0;

	gimbal_encoder.pid_speed_value[0] = 5;
	gimbal_encoder.pid_speed_value[1] = 0.5;
	gimbal_encoder.pid_speed_value[2] = 0;

	// // normal
	// gimbal_gyro.pid_angle_value[0] = 260;
	// gimbal_gyro.pid_angle_value[1] = 0.05;
	// gimbal_gyro.pid_angle_value[2] = 800;

	// gimbal_gyro.pid_speed_value[0] = 1;
	// gimbal_gyro.pid_speed_value[1] = 0;
	// gimbal_gyro.pid_speed_value[2] = 2;

	// // normal
	// gimbal_gyro.pid_angle_value[0] = 350;
	// gimbal_gyro.pid_angle_value[1] = 0.05;
	// gimbal_gyro.pid_angle_value[2] = 800;

	// gimbal_gyro.pid_speed_value[0] = 2.5;
	// gimbal_gyro.pid_speed_value[1] = 0;
	// gimbal_gyro.pid_speed_value[2] = 2;

	// normal
	gimbal_gyro.pid_angle_value[0] = 1050;
	gimbal_gyro.pid_angle_value[1] = 0.05;
	gimbal_gyro.pid_angle_value[2] = 800;

	gimbal_gyro.pid_speed_value[0] = 2.5;
	gimbal_gyro.pid_speed_value[1] = 0;
	gimbal_gyro.pid_speed_value[2] = 2;

	// // shoot
	// gimbal_gyro.pid_angle_value[0] = 40;
	// gimbal_gyro.pid_angle_value[1] = 0.001;
	// gimbal_gyro.pid_angle_value[2] = 30;

	// gimbal_gyro.pid_speed_value[0] = 25;
	// gimbal_gyro.pid_speed_value[1] = 0.001;
	// gimbal_gyro.pid_speed_value[2] = 0;

	gimbal_encoder.target_angle = 0;
	gimbal_gyro.target_angle = 0;

	gimbal_encoder.pid_angle_out = 0;
	gimbal_encoder.pid_speed_out = 0;
	gimbal_gyro.pid_angle_out = 0;
	gimbal_gyro.pid_speed_out = 0;

	pid_init(&gimbal_encoder.pid_angle, gimbal_encoder.pid_angle_value, 500, 8191);
	pid_init(&gimbal_encoder.pid_speed, gimbal_encoder.pid_speed_value, 100, 3000);
	pid_init(&gimbal_gyro.pid_angle, gimbal_gyro.pid_angle_value, 10000, 10000);
	pid_init(&gimbal_gyro.pid_speed, gimbal_gyro.pid_speed_value, 10000, 10000);
}

// /**********************将目标角度从（-pi, pi）映射到（0, 8091）*************************/
// float angle_map(float cur_angle)
// {
// 	return (cur_angle + pi) * 8191/(2 * pi);
// }

/************************************ 角度过零处理 ********************************/
void angle_over_zero(float err)
{
	if (gimbal_mode == 0)
	{
		if (err > 4096) // 4096 ：半圈机械角度
		{
			gimbal_encoder.target_angle -= 8191;
		}
		else if (err < -4096)
		{
			gimbal_encoder.target_angle += 8191;
		}
	}

	if (gimbal_mode == 1)
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
}

/************************************读取yaw轴imu数据**************************************/
static void Yaw_read_imu()
{
	// 三个角度值读取
	ins_yaw = INS.Yaw;
	ins_pitch = INS.Pitch;
	ins_roll = INS.Roll;

	// 记录初始位置
	if (Update_yaw_flag)
	{
		Update_yaw_flag = 0; // 只进入一次
		init_yaw = ins_yaw - 0.0f;
		gimbal_gyro.target_angle = init_yaw;
	}

	// 校正
	ins_yaw_update = ins_yaw - init_yaw;
}

/************************判断数值正负*******************************************/
// template <typename T>
// int sign(T val)
// {
// 	return (val > T(0)) - (val < T(0));
// }

// int sign(float val)
// {
// 	return (val > 0) - (val < 0);
// }

/***************************** 处理接收遥控器数据控制云台旋转 *********************************/
void gimbal_control()
{
	if (gimbal_mode == 0)
	{
		gimbal_encoder.target_angle += 0.02 * rc_ctrl.rc.ch[0];
		detel_calc(&gimbal_encoder.target_angle);
		err_yaw_angle = gimbal_gyro.target_angle - motor_can2[3].rotor_angle;
		angle_over_zero(err_yaw_angle);
		gimbal_encoder.pid_angle_out = pid_calc(&gimbal_encoder.pid_angle, gimbal_encoder.target_angle, motor_can2[4].rotor_angle);	 // 计算出云台角度
		gimbal_encoder.pid_speed_out = pid_calc(&gimbal_encoder.pid_speed, gimbal_encoder.pid_angle_out, motor_can2[4].rotor_speed); // 计算出云台速度

		gimbal_can2_cmd(gimbal_encoder.pid_speed_out); // 给电流
	}

	if (gimbal_mode == 1)
	{
		// 视觉控制
		if (rc_ctrl.rc.s[1] == 2 || press_right == 1) // 左拨杆下
		{
			gimbal_mode_vision();
		}

		// 锁yaw模式
		else // 左拨杆上或中
		{
			gimbal_mode_normal();
		}
	}
}

/**************************** 视觉控制 **********************************/
void gimbal_mode_vision()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// 如果追踪到目标
	if (vision_is_tracking)
	{
		// 视觉模式中加入遥控器的微调
		float normalized_input = (rc_ctrl.rc.ch[0] / 660.0 + rc_ctrl.mouse.x / 16384) * 5; // 最大微调角度限制为5°
		gimbal_gyro.target_angle = vision_yaw - normalized_input;
	}

	else
	{
		// 使用非线性映射函数调整灵敏度
		float normalized_input = rc_ctrl.rc.ch[0] / 660.0 + rc_ctrl.mouse.x / 16384;
		gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
	}

	detel_calc(&gimbal_gyro.target_angle);

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle, gimbal_gyro.target_angle, INS.Yaw);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);

	// 给电流
	gimbal_can2_cmd(gimbal_gyro.pid_speed_out); // 给电流
}

/**************************** 锁yaw **********************************/
void gimbal_mode_normal()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// // 接收遥控器数值
	// gimbal_gyro.target_angle -= rc_ctrl.rc.ch[0] / 660 * 0.3; // 遥控器右边左右控制yaw轴电机

	// 使用非线性映射函数调整灵敏度
	float normalized_input = rc_ctrl.rc.ch[0] / 660.0 + rc_ctrl.mouse.x / 16384.0f * 20;
	gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;

	// if (rc_ctrl.rc.ch[0] > 0)
	// {
	// 	gimbal_gyro.target_angle -= 0.2;
	// }
	// if (rc_ctrl.rc.ch[0] < 0)
	// {
	// 	gimbal_gyro.target_angle += 0.2;
	// }

	detel_calc(&gimbal_gyro.target_angle);

	// // 在范围内置零，消除抖动
	// if (err_yaw_angle > -err_yaw_range && err_yaw_angle < err_yaw_range)
	// {
	// 	err_yaw_angle = 0;
	// }

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle, gimbal_gyro.target_angle, ins_yaw_update);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);
	// gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, motor_can2[5].rotor_speed);

	// 给电流
	gimbal_can2_cmd(gimbal_gyro.pid_speed_out);
}

/*****************************角度范围限制**********************************/
static void detel_calc(fp32 *angle)
{
	if (gimbal_mode == 0)
	{
		if (*angle > 8191)
		{
			*angle -= 8191;
		}
		else if (*angle < 0)
		{
			*angle += 8191;
		}
	}

	if (gimbal_mode == 1)
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
}

/********************************can2发送电流***************************/
void gimbal_can2_cmd(int16_t v4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];

	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;	  // 标准帧
	tx_header.RTR = CAN_RTR_DATA; // 数据帧

	tx_header.DLC = 8; // 发送数据长度（字节）

	tx_data[0] = NULL;
	tx_data[1] = NULL;
	tx_data[2] = NULL;
	tx_data[3] = NULL;
	tx_data[4] = NULL;
	tx_data[5] = NULL;
	tx_data[6] = (v4 >> 8) & 0xff;
	tx_data[7] = (v4) & 0xff;

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}