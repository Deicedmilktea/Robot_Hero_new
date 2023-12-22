#ifndef STARTTASK04_H
#define STARTTASK04_H

#include "INS_task.h"
#include "pid.h"
#include "struct_typedef.h"
#include "rc_potocal.h"

//声明一些其他文件的全局变量
extern ins_data_t ins_data;		//这是陀螺仪解算出来的数据
extern RC_ctrl_t rc_ctrl;		//遥控器数据
extern uint16_t Up_ins_yaw;

//定义一些宏定义


//定义函数
void StartTask04(void const * argument);


#endif
