#include "Robot.h"
#include "bsp_usart.h"
#include "remote_control.h"
#include "referee_task.h"

// static RC_ctrl_t *rc_data;                 // 遥控器数据,初始化时返回
// static referee_info_t *referee_data;       // 用于获取裁判系统的数据
// static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

void Robot_init()
{
    // rc_data = RemoteControlInit(&huart3);         // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
}