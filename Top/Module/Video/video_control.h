#ifndef VIDEO_CONTROL_H
#define VIDEO_CONTROL_H

#include "stdint.h"
#include "main.h"
#include "usart.h"
#include "remote_control.h"
#include "referee_protocol.h"

#pragma pack(1)

typedef struct
{
    xFrameHeader FrameHeader;  // 接收到的帧头信息
    uint16_t CmdID;            // 命令码
    remote_control_t key_data; // 遥控器数据

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} Video_ctrl_t;

#pragma pack()

Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);

#endif // !VIDEO_CONTROL_H