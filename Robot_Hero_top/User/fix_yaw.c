#include "fix_yaw.h"

//	思路：利用速度环作基础PID反馈给云台，用云台陀螺仪的yaw值不变作角度环进行补偿
//  注意：云台往左转是负数
//  这一版本是针对遥控器控制的版本

//	定义一些全局变量

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_row;
fp32 init_yaw;	//记住init_yaw初值
int yaw_model_flag = 1;
fp32 err_yaw;
fp32 angle_weight = 1;	//角度环->速度环，映射的权重

#define valve 100		//阈值
#define base 1024		//遥控器的回中值
#define base_max 1684		
#define base_min 364
#define angle_valve 10		//角度阈值，在这个范围内就不去抖动了

void FIX_Task(void const *pvParameters)
{
  //初始化设置
	
	//yaw轴电机初始化，id为can1的5号
	pid_init(&motor_pid[5],60,10,5,30000,30000);
	
	
	//循环任务运行
  for(;;)
  {
		
		//三个角度值读取
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2];
		
		//模式判断,左上角开关开到最下方
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			
		
				//遥感回中锁云台
				if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve)
				{
					
					
					
					if(yaw_model_flag == 1)		//移动云台后重新记住云台的初始位置的值
					{
						init_yaw = ins_yaw;
						yaw_model_flag = 0;
					}
					
					
					err_yaw = ins_yaw - init_yaw;		//用实时数据减初始数据
					
					//阈值判断
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[5] = err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[5] = 0;
					}

				}
				
				
				//不回中的时候可以移动云台
				else if(rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max)
				{
					target_speed[5] = 60;
					yaw_model_flag = 1;
				}
				else if(rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve)
				{
					target_speed[5] = -60;
					yaw_model_flag = 1;
				}
				
				//计算pid，发送函数在其他的任务里面
				motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_speed[5], motor_info[5].rotor_speed);
			
				
		}
		
    osDelay(1);
  }

}