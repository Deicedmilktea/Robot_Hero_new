/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 4
* 云台电机为motor_can2[3] // CAN1控制
* 遥控器控制：左拨杆左右
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
uint8_t gimbal_mode = 0; // 记录模式，0为编码器，1为陀螺仪

// yaw_correct
int Update_yaw_flag = 1;
float imu_err_yaw = 0; // 记录yaw飘移的数值便于进行校正

extern RC_ctrl_t rc_ctrl;
extern INS_t INS;
extern motor_info_t motor_can2[4];
extern bool vision_is_tracking;
extern float vision_yaw;

// 初始化
static void Gimbal_loop_init();

// 角度过零处理
static void angle_over_zero(float err);

// 控制云台旋转
static void gimbal_control();

// 角度范围限制
static void detel_calc(fp32 *angle);

// can1发送电流
static void gimbal_can2_cmd(int16_t v4);

// 视觉控制
static void gimbal_mode_vision();

// 锁yaw
static void gimbal_mode_normal();

// 云台运动task
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
		gimbal_control();
		osDelay(1);
	}
}

static void Gimbal_loop_init()
{
	// Kp, Ki, Kd
	gimbal_encoder.pid_angle_value[0] = 5;
	gimbal_encoder.pid_angle_value[1] = 0.5;
	gimbal_encoder.pid_angle_value[2] = 0;

	gimbal_encoder.pid_speed_value[0] = 5;
	gimbal_encoder.pid_speed_value[1] = 0.5;
	gimbal_encoder.pid_speed_value[2] = 0;

	// // normal不压榨电机，效果一般，超调
	// gimbal_gyro.pid_angle_value[0] = 200;
	// gimbal_gyro.pid_angle_value[1] = 0.05;
	// gimbal_gyro.pid_angle_value[2] = 2000;

	// gimbal_gyro.pid_speed_value[0] = 10;
	// gimbal_gyro.pid_speed_value[1] = 0;
	// gimbal_gyro.pid_speed_value[2] = 0;

	// normal日常使用版本，压榨一点点电机，效果不错
	gimbal_gyro.pid_angle_value[0] = 150;
	gimbal_gyro.pid_angle_value[1] = 0.01;
	gimbal_gyro.pid_angle_value[2] = 800;

	gimbal_gyro.pid_speed_value[0] = 20;
	gimbal_gyro.pid_speed_value[1] = 0;
	gimbal_gyro.pid_speed_value[2] = 100;

	// // normal效果很好，但是6020发热很严重，能感觉电机微小抖动
	// gimbal_gyro.pid_angle_value[0] = 150;
	// gimbal_gyro.pid_angle_value[1] = 0.05;
	// gimbal_gyro.pid_angle_value[2] = 0;

	// gimbal_gyro.pid_speed_value[0] = 20;
	// gimbal_gyro.pid_speed_value[1] = 0;
	// gimbal_gyro.pid_speed_value[2] = 800;

	gimbal_encoder.target_angle = 0;
	gimbal_gyro.target_angle = 0;

	gimbal_encoder.pid_angle_out = 0;
	gimbal_encoder.pid_speed_out = 0;
	gimbal_gyro.pid_angle_out = 0;
	gimbal_gyro.pid_speed_out = 0;

	pid_init(&gimbal_encoder.pid_angle, gimbal_encoder.pid_angle_value, 500, 8191);
	pid_init(&gimbal_encoder.pid_speed, gimbal_encoder.pid_speed_value, 100, 3000);
	pid_init(&gimbal_gyro.pid_angle, gimbal_gyro.pid_angle_value, 30000, 30000);
	pid_init(&gimbal_gyro.pid_speed, gimbal_gyro.pid_speed_value, 30000, 30000);
}

/************************************ 角度过零处理 ********************************/
static void angle_over_zero(float err)
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
	// 记录初始位置
	if (Update_yaw_flag)
	{
		Update_yaw_flag = 0; // 只进入一次
		INS.yaw_init = INS.Yaw - 0.0f;
		gimbal_gyro.target_angle = INS.yaw_init;
	}

	// // 顺时针旋转，陀螺仪飘 -90°/min
	// // 解决yaw偏移，完成校正
	// if (rc_ctrl.rc.ch[2] > 50 || rc_ctrl.mouse.x > 1500)
	// 	imu_err_yaw += 0.0015f;
	// if ((rc_ctrl.rc.ch[2] < -50 || rc_ctrl.mouse.x < -1500))
	// 	imu_err_yaw -= 0.0015f;

	// 校正
	INS.yaw_update = INS.Yaw - INS.yaw_init + imu_err_yaw;
}

/***************************** 处理接收遥控器数据控制云台旋转 *********************************/
static void gimbal_control()
{
	if (gimbal_mode == 0)
	{
		gimbal_encoder.target_angle += 0.02 * rc_ctrl.rc.ch[2];
		detel_calc(&gimbal_encoder.target_angle);
		err_yaw_angle = gimbal_gyro.target_angle - motor_can2[3].rotor_angle;
		angle_over_zero(err_yaw_angle);
		gimbal_encoder.pid_angle_out = pid_calc(&gimbal_encoder.pid_angle, gimbal_encoder.target_angle, motor_can2[3].rotor_angle);	 // 计算出云台角度
		gimbal_encoder.pid_speed_out = pid_calc(&gimbal_encoder.pid_speed, gimbal_encoder.pid_angle_out, motor_can2[3].rotor_speed); // 计算出云台速度

		gimbal_can2_cmd(gimbal_encoder.pid_speed_out); // 给电流
	}

	if (gimbal_mode == 1)
	{
		// 视觉控制
		if (rc_ctrl.rc.s[0] == 1 || press_right == 1) // 左拨杆上 || 按住右键
		{
			gimbal_mode_vision();
		}

		// 锁yaw模式
		else // 左拨杆中或下
		{
			gimbal_mode_normal();
		}
	}
}

/**************************** 视觉控制 **********************************/
static void gimbal_mode_vision()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// 如果追踪到目标
	if (vision_is_tracking)
	{
		// 视觉模式中加入手动微调
		float normalized_input = (rc_ctrl.rc.ch[2] / 660.0f + rc_ctrl.mouse.x / 16384.0f) * 10; // 最大微调角度限制为5°
		gimbal_gyro.target_angle = vision_yaw - normalized_input;
	}

	else
	{
		// 使用非线性映射函数调整灵敏度
		float normalized_input = rc_ctrl.rc.ch[2] / 660.0f + rc_ctrl.mouse.x / 16384.0f * 100;
		gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
	}

	detel_calc(&gimbal_gyro.target_angle);

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle, gimbal_gyro.target_angle, INS.yaw_update);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);

	// 给电流
	gimbal_can2_cmd(gimbal_gyro.pid_speed_out); // 给电流
}

/**************************** 锁yaw **********************************/
static void gimbal_mode_normal()
{
	// 接收Yaw轴imu数据
	Yaw_read_imu();

	// 使用非线性映射函数调整灵敏度
	float normalized_input = rc_ctrl.rc.ch[2] / 660.0f + rc_ctrl.mouse.x / 16384.0f * 100;
	gimbal_gyro.target_angle -= pow(fabs(normalized_input), 0.97) * sign(normalized_input) * 0.3;

	detel_calc(&gimbal_gyro.target_angle);

	// // 在范围内置零，消除抖动
	// if (err_yaw_angle > -err_yaw_range && err_yaw_angle < err_yaw_range)
	// {
	// 	err_yaw_angle = 0;
	// }

	// 云台角度输出
	gimbal_gyro.pid_angle_out = pid_calc_a(&gimbal_gyro.pid_angle, gimbal_gyro.target_angle, INS.yaw_update);

	// 云台速度输出
	gimbal_gyro.pid_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.pid_angle_out, INS.Gyro[2] * 57.3f);

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
static void gimbal_can2_cmd(int16_t v4)
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