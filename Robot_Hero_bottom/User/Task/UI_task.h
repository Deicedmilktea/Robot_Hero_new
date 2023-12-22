#ifndef __RM_CILENT_UI__
#define __RM_CILENT_UI__
#include "main.h"
#include "stm32f4xx.h"
#include "stdarg.h"
#include "struct_typedef.h"
#include "rc_potocal.h"

#define Robot_ID UI_Data_RobotID_RStandard1
#define Cilent_ID UI_Data_CilentID_RStandard1        //红步兵1

#pragma pack(1)                           //按1字节对齐

#define NULL 0
#define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据，这部分为自定义********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101				//画一个图
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104				//画7个图
#define UI_Data_ID_DrawChar 0x110			//写字符
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8



typedef unsigned char Uint8_t;
typedef unsigned char uint8_t;



//帧头部分
typedef __packed struct
{
    uint8_t SOF;                    //起始字节,固定0xA5
    uint16_t Data_Length;           //帧数据长度
    uint8_t Seq;                    //包序号
    uint8_t CRC8;                   //CRC8校验值
    uint16_t CMD_ID;                //命令ID
} UI_Packhead;

//针对0x0301命令的 自定义ID
typedef __packed struct
{
    uint16_t Data_ID;               //内容ID ，自定义
    uint16_t Sender_ID;             //发送者ID
    uint16_t Receiver_ID;           //接收者ID
} UI_Data_Operate;

//客户端图形操作
typedef __packed struct
{
    uint8_t Delete_Operate;         //删除操作
    uint8_t Layer;                  //删除图层
} UI_Data_Delete;          //删除图层帧


typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    float graph_Float;              //浮点数据
} Float_Data;


typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;              //图形数据
} Graph_Data;


typedef __packed struct
{
    Graph_Data Graph_Control;
    uint8_t show_Data[30];
} String_Data;                  //打印字符串数据




/**
 *  @brief          删除图层
 *  @param[in]      操作类型  0：空操作   1：删除指定图层   2：删除所有图层
 *  @param[in]      图层数 [0, 9]
 */
 void UI_delete(uint8_t operate, uint8_t layer);

/**
 *  @brief          使用自定义UI绘制直线
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始X坐标
 *  @param[in]      起始Y坐标
 *  @param[in]      结束X坐标
 *  @param[in]      结束Y坐标
 *  @retval         none
 */
void Line_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/**
 *  @brief          使用自定义UI绘制正圆
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      圆心X坐标
 *  @param[in]      圆心Y坐标
 *  @param[in]      半径
 *  @retval         none
 */
 void Circle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

/**
 *  @brief          使用自定义UI绘制圆弧
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始角度
 *  @param[in]      中止角度
 *  @param[in]      圆心X坐标
 *  @param[in]      圆心Y坐标
 *  @param[in]      X方向轴长
 *  @param[in]      Y方向轴长，参考椭圆的使用方法
 *  @retval         none
 */
 void Arc_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);

/**
 *  @brief          使用自定义UI绘制矩形
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始X坐标
 *  @param[in]      起始Y坐标
 *  @param[in]      结束X坐标
 *  @param[in]      结束Y坐标
 *  @return         none
 */
 void Rectangle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void Float_Draw(Float_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);

/**
 *  @brief          绘制字符型数据
 *  @param[in]      String_Data* 字符串结构体的指针
 *  @param[in]      图片名称
 *  @param[in]      图形操作
 *  @param[in]      图层 [0,9]
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      字号，建议与线宽的比为 10:1
 *  @param[in]      开始坐标X
 *  @param[in]      开始坐标Y
 *  @param[in]      待发送的字符串，一次最多发送30个字符
 *  @retval         none
 */
 void string_Draw(String_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Start_x, uint32_t Start_y, char* Char_Data);
/**
 *  @brief          使用自定义UI绘制一个图形，刷新出来在屏幕上显示
 *  @param[in]      图形
 *  @return         none
 */
 void ui_display_1_graph(Graph_Data* graph1);

/**
 *  @brief          使用自定义UI绘制两个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @return         none
 */
 void ui_display_2_graph(Graph_Data* graph1, Graph_Data* graph2);

/**
 *  @brief          使用自定义UI绘制五个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @param[in]      图形3
 *  @param[in]      图形4
 *  @param[in]      图形5
 *  @return         none
 */
 void ui_display_5_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5);

/**
 *  @brief          使用自定义UI绘制七个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @param[in]      图形3
 *  @param[in]      图形4
 *  @param[in]      图形5
 *  @param[in]      图形6
 *  @param[in]      图形7
 *  @return         none
 */
 void ui_display_7_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5, Graph_Data* graph6, Graph_Data* graph7);

/**
 *  @brief          使用自定义UI绘制字符串
 *  @param[in]      字符串数据结构体指针
 *  @return         none
 */
 void ui_display_string(String_Data* str_data);



#define USERUI_TASK_CONTROL_TIME_MS 10

//按下手动刷新键一段时间，UI全部重绘（当UI发生异常时使用）
#define USERUI_MANUAL_REFRESH_CONTROL_TIME  1000
#define USERUI_MANUAL_REFRESH_KEY           KEY_PRESSED_OFFSET_Z

//每隔一段时间，所有UI控件（不包括背景）重绘一次
#define USERUI_MODULE_REFRESH_TIME          1000






typedef struct  
{
    const RC_ctrl_t* rc;
    uint16_t all_refresh_key_press_time;    //全屏刷新按键按下的时长
    uint8_t all_refresh_flag;               //全屏刷新标志位
    uint16_t module_refresh_timer;          //每隔一段时间，所有UI控件（不包括背景）重绘一次
    uint8_t module_extern_flag;             //UI元素已存在标志
} userUI_control_t;

void UI_Task(void const* argument);
uint8_t get_usart6_tx_dma_busy_flag(void);
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void clear_usart6_tx_dma_busy_sign(void);

uint8_t get_robot_id(void);
uint16_t get_client_id(void);

#endif