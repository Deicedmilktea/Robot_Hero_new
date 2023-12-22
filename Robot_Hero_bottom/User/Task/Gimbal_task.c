/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 5
* 云台电机为motor_can1[5]
* 遥控器控制：右拨杆左右
*/

#include "Gimbal_task.h"
#include "rc_potocal.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "drv_can.h"

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		

gimbal_t gimbal_encoder; //gimbal encoder
gimbal_t gimbal_gyro; //gimbal gyro
fp32 err_yaw_angle;//yaw angle error
extern RC_ctrl_t rc_ctrl;
extern INS_t INS_top;
extern motor_info_t  motor_can1[5];

float gimbal_angle_out = 0;
float gimbal_speed_out = 0;
int gimbal_mode = 0; //记录模式，0为编码器，1为陀螺仪
int error6 = 0;

fp32 ins_yaw;
fp32 ins_yaw_update = 0;
fp32 Driftring_yaw = 0;
fp32 ins_pitch;
fp32 ins_roll;
fp32 init_yaw;	//记录yaw初始量
int  Update_yaw_flag = 1;
fp32 err_yaw_range = 1;

/********************云台运动task*********************/
void Gimbal_task(void const *pvParameters)
{
	Gimbal_loop_init();
	gimbal_mode = 1;
	if(gimbal_mode == 0){
		gimbal_encoder.target_angle = motor_can1[5].rotor_angle;
	}
	
	for(;;)
	{
		error6++;
		//remote_gimbal_control();  //加上遥控器的控制
		osDelay(1);
	}
}

void Gimbal_loop_init()
{
	//Kp, Ki, Kd
	gimbal_encoder.pid_angle_value[0] = 5;
	gimbal_encoder.pid_angle_value[1] = 0.5;
	gimbal_encoder.pid_angle_value[2] = 0;

	gimbal_encoder.pid_speed_value[0] = 5;
	gimbal_encoder.pid_speed_value[1] = 0.5;
	gimbal_encoder.pid_speed_value[2] = 0;

	gimbal_gyro.pid_angle_value[0] = 1;
	gimbal_gyro.pid_angle_value[1] = 0;
	gimbal_gyro.pid_angle_value[2] = 0;

	gimbal_gyro.pid_speed_value[0] = 300;
	gimbal_gyro.pid_speed_value[1] = 30;
	gimbal_gyro.pid_speed_value[2] = 1;

	gimbal_encoder.target_angle = 0;
	gimbal_encoder.target_speed = 0;

	pid_init(&gimbal_encoder.pid_angle, gimbal_encoder.pid_angle_value, 500, 8191);
	pid_init(&gimbal_encoder.pid_speed, gimbal_encoder.pid_speed_value, 100, 3000);
	pid_init(&gimbal_gyro.pid_angle, gimbal_gyro.pid_angle_value, 15000, 180);
	pid_init(&gimbal_gyro.pid_speed, gimbal_gyro.pid_speed_value, 1000, 15000);
}

// /**********************将目标角度从（-pi, pi）映射到（0, 8091）*************************/
// float angle_map(float cur_angle)
// {
// 	return (cur_angle + pi) * 8191/(2 * pi);
// }

/******************************************角度过零处理********************************/
void angle_over_zero(float err)
{
	if(gimbal_mode == 0)
	{
		if(err > 4096)    //4096 ：半圈机械角度
		{
			gimbal_encoder.target_angle -= 8191;
		}
		else if(err < -4096)
		{
			gimbal_encoder.target_angle += 8191;
		}
	}
	
	if(gimbal_mode == 1)
	{
		if(err > 180)    //180 ：半圈机械角度
		{
			gimbal_gyro.target_angle -= 361;
		}
		else if(err < -180)
		{
			gimbal_gyro.target_angle += 361;
		}
	}
}

/************************************读取yaw轴imu数据**************************************/
static void Yaw_read_imu()
{
		//三个角度值读取
		ins_yaw = INS_top.Yaw;
		ins_pitch = INS_top.Pitch;
		ins_roll = INS_top.Roll;
	
		//记录初始位置
		if(Update_yaw_flag)
		{
			Update_yaw_flag = 0;//只进入一次
			init_yaw = ins_yaw - 0.0f;
			gimbal_gyro.target_angle = init_yaw;
		}
		
		//校正
		ins_yaw_update = ins_yaw - init_yaw;
}

/*****************************处理接收遥控器数据控制云台旋转*********************************/
void remote_gimbal_control()
{
		if(gimbal_mode == 0)
		{
			gimbal_encoder.target_angle += 0.02*rc_ctrl.rc.ch[0];
			detel_calc(&gimbal_encoder.target_angle);
			err_yaw_angle = gimbal_gyro.target_angle - motor_can1[4].rotor_angle;
			angle_over_zero(err_yaw_angle);
			gimbal_angle_out = pid_calc(&gimbal_encoder.pid_angle, gimbal_encoder.target_angle, motor_can1[4].rotor_angle); //计算出云台角度
			gimbal_speed_out = pid_calc(&gimbal_encoder.pid_speed, gimbal_encoder.target_speed, gimbal_angle_out); //计算出云台速度

    		gimbal_can1_cmd(gimbal_speed_out);//给电流
		}
		
		if(gimbal_mode == 1)
		{
			//接收Yaw轴imu数据
			Yaw_read_imu();
			
			//接收遥控器数值
			gimbal_gyro.target_angle -= rc_ctrl.rc.ch[0] / 660 * 0.3; //遥控器右边左右控制yaw轴电机

			detel_calc(&gimbal_gyro.target_angle);

			// 计算偏移量
			err_yaw_angle = gimbal_gyro.target_angle - ins_yaw_update;

			detel_calc(&err_yaw_angle);

			//在范围内置零，消除抖动
			if(err_yaw_angle > -err_yaw_range && err_yaw_angle < err_yaw_range){
				err_yaw_angle = 0;
			}

			// 云台角度输出
			gimbal_angle_out = pid_calc(&gimbal_gyro.pid_angle, 0, err_yaw_angle);

			// 云台速度输出
			gimbal_speed_out = pid_calc(&gimbal_gyro.pid_speed, gimbal_gyro.target_speed, gimbal_angle_out);

			// 给电流
			gimbal_can1_cmd(gimbal_speed_out);//给电流
			// gimbal_can1_cmd(5000);//给电流
//			CAN_cmd_gimbal(10000);//给电流
		}
}

/*****************************角度范围限制**********************************/
static void detel_calc(fp32 *angle)
{
	if(gimbal_mode == 0){
		if(*angle > 8191)
		{
			*angle -= 8191;
		}
		else if(*angle < 0)
		{
			*angle += 8191;
		}
	}

	if(gimbal_mode == 1){
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

/********************************can1发送电流***************************/
void gimbal_can1_cmd(int16_t v1)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x2FF;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
  tx_header.DLC   = 8;		//发送数据长度（字节）

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] =         NULL;
  tx_data[3] =         NULL;
  tx_data[4] =         NULL;
  tx_data[5] =         NULL;
  tx_data[6] =         NULL;
  tx_data[7] =         NULL;

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}