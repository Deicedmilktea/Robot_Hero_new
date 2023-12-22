
#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


#include "RTE_Components.h"             // Component selection
void pid_init(pid_struct_t *pid, float value[3], float i_max, float out_max)
{
  pid->kp      = value[0];
  pid->ki      = value[1];
  pid->kd      = value[2];
  pid->i_max   = i_max;
  pid->out_max = out_max;
}


float pid_calc(pid_struct_t *pid, float ref, float fdb)//ref是目标值,fdb是电机解码的速度返回值
{	
	pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb;//err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
	LimitMax(pid->i_out, pid->i_max);//防止越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LimitMax(pid->output, pid->out_max);//防止越界
  return pid->output;//电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)
}


