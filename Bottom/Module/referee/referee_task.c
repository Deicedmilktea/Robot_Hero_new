/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "video_control.h"

#define X1 960 // 落风坡 yellow
#define Y1 370
#define X2 950 // 高地 pink
#define Y2 400
#define X3 950 // 吊射点 oriange
#define Y3 350
#define LINE_LENGTH 25

Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;     // 接收到的裁判系统数据
uint8_t UI_Seq;                               // 包序号，供整个referee文件使用

extern RC_ctrl_t rc_ctrl[2];       // 遥控器控制数据
extern Video_ctrl_t video_ctrl[2]; // 视频控制数据
extern INS_t INS_top;              // 上C板imu数据
extern uint8_t vision_is_tracking; // 视觉是否识别到目标
extern uint8_t friction_mode;      // 摩擦轮转速模式

// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    // 按下V键，重新初始化UI
    if (rc_ctrl[TEMP].key[KEY_PRESS].x || video_ctrl[TEMP].key[KEY_PRESS].x)
        MyUIInit();
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[11]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static Graph_Data_t UI_pitch;          // pitch角度
static String_Data_t UI_State_sta[7];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 565, 515, 490};

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 860, shoot_line_location[2], 1060, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 885, shoot_line_location[3], 1035, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 910, shoot_line_location[4], 1010, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // 绘制发射点，落风坡
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 2, X1 - LINE_LENGTH, Y1, X1 + LINE_LENGTH, Y1);
    UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 2, X1, Y1 - LINE_LENGTH, X1, Y1 + LINE_LENGTH);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[5], UI_shoot_line[6]);

    // 绘制发射点，高地
    UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Pink, 2, X2 - LINE_LENGTH, Y2, X2 + LINE_LENGTH, Y2);
    UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Pink, 2, X2, Y2 - LINE_LENGTH, X2, Y2 + LINE_LENGTH);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[7], UI_shoot_line[8]);

    // 绘制发射点，吊射点
    UILineDraw(&UI_shoot_line[9], "sl9", UI_Graph_ADD, 7, UI_Color_Orange, 2, X3 - LINE_LENGTH, Y3, X3 + LINE_LENGTH, Y3);
    UILineDraw(&UI_shoot_line[10], "sll", UI_Graph_ADD, 7, UI_Color_Orange, 2, X3, Y3 - LINE_LENGTH, X3, Y3 + LINE_LENGTH);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[9], UI_shoot_line[10]);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 150, 750, "chassis-f:"); // 底盘模式
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 150, 700, "supcap-c:"); // 超电开关
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 150, 650, "frict-e:"); // 摩擦轮转速
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 150, 600, "shoot-q:"); // 拨盘模式
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 150, 550, "top-r:"); // 小陀螺转速
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 390, 750, "stop");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 360, 700, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 340, 650, "normal");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 340, 600, "normal");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 4, 290, 550, "low");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 底盘功率显示，静态
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 3, 870, 210, "Power:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);

    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 超电电压显示,动态
    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 3, 970, 210, 24000);

    // 能量条初始状态
    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 36, 722, 160, 1218, 160);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    // pitch角度显示
    UICharDraw(&UI_State_sta[6], "ss7", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 1050, 630, "Pitch:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);
    UIFloatDraw(&UI_pitch, "sd7", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 1150, 630, 0); // pitch角度
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch);
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 390, 750, "stop  ");
            break;
        case CHASSIS_ROTATE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 390, 750, "rotate");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_NO_FOLLOW:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 390, 750, "normal");
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 390, 750, "follow");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // supercap
    if (_Interactive_data->Referee_Interactive_Flag.supcap_flag == 1)
    {
        UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 360, 700, _Interactive_data->supcap_mode == SUPCAP_OFF ? "off" : "on ");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.supcap_flag = 0;
    }

    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        switch (_Interactive_data->friction_mode)
        {
        case FRICTION_NORMAL:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 340, 650, "normal");
            break;
        }
        case FRICTION_LOW:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 340, 650, "low   ");
            break;
        }
        case FRICTION_HIGH:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 340, 650, "high  ");
            break;
        }
        case FRICTION_STOP:
        {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 340, 650, "stop  ");
            break;
        }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }

    // trigger
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 340, 600, _Interactive_data->shoot_mode == SHOOT_NORMAL ? "normal" : "buff  ");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }

    // top
    if (_Interactive_data->Referee_Interactive_Flag.top_flag == 1)
    {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Yellow, 25, 4, 290, 550, _Interactive_data->top_mode == TOP_LOW ? "low " : "high");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.top_flag = 0;
    }

    // supercap_power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 722, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }

    // pitch
    if (_Interactive_data->Referee_Interactive_Flag.pitch_flag == 1)
    {
        UIFloatDraw(&UI_pitch, "sd7", UI_Graph_Change, 8, UI_Color_Yellow, 18, 2, 2, 1150, 630, _Interactive_data->pitch * 1000);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch);
        _Interactive_data->Referee_Interactive_Flag.pitch_flag = 0;
    }

    // is_tracking
    if (_Interactive_data->Referee_Interactive_Flag.tracking_flag == 1)
    {
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 8, _Interactive_data->is_tracking == 1 ? UI_Color_Pink : UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 8, _Interactive_data->is_tracking == 1 ? UI_Color_Pink : UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // chassis_mode
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    // supercap开关
    if (_Interactive_data->supcap_mode != _Interactive_data->supcap_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.supcap_flag = 1;
        _Interactive_data->supcap_last_mode = _Interactive_data->supcap_mode;
    }
    // friction_mode转速
    _Interactive_data->friction_mode = friction_mode;
    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    // 拨盘模式
    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    // 小陀螺转速
    if (_Interactive_data->top_mode != _Interactive_data->top_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.top_flag = 1;
        _Interactive_data->top_last_mode = _Interactive_data->top_mode;
    }

    // supercap数据
    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    // pitch
    _Interactive_data->pitch = INS_top.Pitch;
    if (_Interactive_data->pitch != _Interactive_data->pitch_last)
    {
        _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
        _Interactive_data->pitch_last = _Interactive_data->pitch;
    }

    // is_tracking
    _Interactive_data->is_tracking = vision_is_tracking;
    if (_Interactive_data->is_tracking != _Interactive_data->is_tracking_last)
    {
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 1;
        _Interactive_data->is_tracking_last = _Interactive_data->is_tracking;
    }
}
