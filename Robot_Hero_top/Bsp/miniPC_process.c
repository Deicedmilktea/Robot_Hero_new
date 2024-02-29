#include "miniPC_process.h"
#include "string.h"

float vision_yaw = 0;
float vision_pitch = 0;
bool vision_is_tracking = false;

static Vision_Instance *vision_instance; // 用于和视觉通信的串口实例

/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 * 此处根据接收结构体的不同修改
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* 使用memcpy接收浮点型小数 */
    memcpy(&recv->is_tracking, &rx_buff[1], 1);
    memcpy(&recv->yaw, &rx_buff[2], 4);
    memcpy(&recv->pitch, &rx_buff[6], 4);

    /* 接收校验位 */
    memcpy(&recv->checksum, &rx_buff[10], 2);
}

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision(void)
{
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header)
    {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);

        vision_is_tracking = vision_instance->recv_data->is_tracking;
        vision_yaw = vision_instance->recv_data->yaw;
        vision_pitch = vision_instance->recv_data->pitch;
    }
    else
    {
    }
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header = send_config->header;
    send_data->detect_color = send_config->detect_color;
    send_data->reset_tracker = send_config->reset_tracker;
    send_data->is_shoot = send_config->is_shoot;
    send_data->tail = send_config->tail;
    return send_data;
}

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(Vision_Init_Config_s *init_config)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));

    init_config->usart_config.module_callback = DecodeVision;

    vision_instance->usart = USARTRegister(&init_config->usart_config);
    vision_instance->recv_data = VisionRecvRegister(&init_config->recv_config);
    vision_instance->send_data = VisionSendRegister(&init_config->send_config);
    return vision_instance->recv_data;
}

/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* 发送帧头，目标颜色，是否重置等数据 */
    tx_buff[0] = send->header;
    tx_buff[1] = send->detect_color;
    tx_buff[2] = send->reset_tracker;
    tx_buff[3] = send->is_shoot;

    /* 使用memcpy发送浮点型小数 */
    memcpy(&tx_buff[4], &send->roll, 4);
    memcpy(&tx_buff[8], &send->yaw, 4);
    memcpy(&tx_buff[12], &send->pitch, 4);

    /* 发送校验位 */
    memcpy(&tx_buff[16], &send->checksum, 2);
    memcpy(&tx_buff[18], &send->tail, 1);
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

/**
 * @brief 设置发送给视觉的IMU数据
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    vision_instance->send_data->yaw = yaw;
    vision_instance->send_data->pitch = pitch;
    vision_instance->send_data->roll = roll;
}