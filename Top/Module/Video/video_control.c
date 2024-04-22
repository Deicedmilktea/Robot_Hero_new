#include "video_control.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "string.h"
#include "crc_ref.h"
#include "referee_protocol.h"
#include "drv_can.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

Video_ctrl_t video_ctrl[2]; // 用于存储图传链路的控制数据,[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t is_init;
static uint8_t send_buff[8]; // 发送数据缓冲区
// 图传拥有的串口实例,因为图传是单例,所以这里只有一个,就不封装了
static USART_Instance *video_usart_instance;
static DaemonInstance *video_daemon_instance;

static void VideoDataContorl()
{
    if (video_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (video_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = video_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = video_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = video_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = video_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
    video_ctrl[LAST] = video_ctrl[TEMP];
}

/**
 * @brief 图传数据解析函数
 *
 * @param buff 图传数据
 */
static void VideoRead(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)      // 空数据包，则不作任何处理
        return;
    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&video_ctrl[TEMP].FrameHeader, buff, LEN_HEADER);
    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                // 2个8位拼成16位int
                video_ctrl[TEMP].CmdID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (video_ctrl[TEMP].CmdID)
                {
                case ID_remote_control_data: // 图传链路键鼠数据
                    memcpy(&video_ctrl[TEMP].key_data, (buff + DATA_Offset), LEN_remote_control_data);
                    // 发送给下C板
                    memcpy(send_buff, buff + DATA_Offset, 8);
                    can_remote(send_buff, 0x33);

                    memcpy(send_buff, buff + DATA_Offset + 8, 4);
                    can_remote(send_buff, 0x34);

                    *(uint16_t *)&video_ctrl[TEMP].key[KEY_PRESS] = video_ctrl[TEMP].key_data.keyboard_value;
                    VideoDataContorl();
                    break;
                default:
                    break;
                }
            }
        }
    }
}

/**
 * @brief  图传数据接收回调函数
 *
 */
static void VideoTransmitterCallback()
{
    DaemonReload(video_daemon_instance);
    VideoRead(video_usart_instance->recv_buff);
}

static void VideoTransmitterLostCallback()
{
    USARTServiceInit(video_usart_instance);
}

/**
 * @brief
 *
 * @param vedeo_usart_handle
 * @return Video_ctrl_t*
 */
Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle)
{
    if (is_init)
        return video_ctrl;
    USART_Init_Config_s conf;
    conf.module_callback = VideoTransmitterCallback;
    conf.usart_handle = video_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE;
    video_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VideoTransmitterLostCallback,
        .owner_id = video_usart_instance,
        .reload_count = 30, // 0.3s
    };
    video_daemon_instance = DaemonRegister(&daemon_conf);

    is_init = 1;
    return video_ctrl;
}