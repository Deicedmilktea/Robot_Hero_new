#include "video_control.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "string.h"
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
void VideoRead(uint8_t *buff)
{
    memcpy(&video_ctrl[TEMP].key_data, buff, 12);
    *(uint16_t *)&video_ctrl[TEMP].key[KEY_PRESS] = video_ctrl[TEMP].key_data.keyboard_value;
    VideoDataContorl();
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