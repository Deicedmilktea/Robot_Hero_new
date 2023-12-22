#include "time_user.h"
void TIM1_UP_TIM10_IRQHandler(void)//定时器1的中断，一般CubeMx会配置在系统文件里面，我们需要删除系统文件中的那个
{
	if(motor_info[4].rotor_angle)//根据角度有无来判断是否接收到了电机的响应
	{
		flag_shoot=1;//置1开始转动
		time_count++;//配置的TIM1是100Hz，也就是0.01s进一次中断
	}
	if(time_count==time)//这里用来调整时间，目前是采取的1s，必须赋予一个time初值，不然定时器初始化尚未赋值直接会停止定时器(bug)
	{
		flag_shoot=0;//置0停止转动
		time_count=0;//计数器归0
		HAL_TIM_Base_Stop_IT(&htim1);//停止定时器
	}
  HAL_TIM_IRQHandler(&htim1);//这个是系统函数，里面会有标志位归0
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//中断回调函数，为啥没进来
{

  UNUSED(htim);


}