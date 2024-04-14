/**
 * @file bsp_usart.c
 * @author neozng,Bi Kaixiang (wexhicy@gmail.com)
 * @brief  串口bsp层的实现
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "bsp_usart.h"
#include "stdlib.h"
#include "string.h"

/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USART_Instance *usart_instances[USART_DEVICE_MAX_NUM] = {NULL};

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USART_Instance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USART_Instance *USARTRegister(USART_Init_Config_s *init_config)
{
    USART_Instance *usart = (USART_Instance *)malloc(sizeof(USART_Instance));
    memset(usart, 0, sizeof(USART_Instance));

    usart->usart_handle    = init_config->usart_handle;
    usart->recv_buff_size  = init_config->recv_buff_size;
    usart->module_callback = init_config->module_callback;

    usart_instances[idx++] = usart;
    USARTServiceInit(usart);

    return usart;
}

/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USART_Instance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE_e mode)
{
    switch (mode) {
        case USART_TRANSFER_BLOCKING:
            HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
            break;
        case USART_TRANSFER_IT:
            HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
            break;
        case USART_TRANSFER_DMA:
            HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
            break;
        default:
            while (1)
                ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
            break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USART_Instance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i) {                 // find the instance which is being handled
        if (huart == usart_instances[i]->usart_handle) { // call the callback function if it is not NULL
            if (usart_instances[i]->module_callback != NULL) {
                usart_instances[i]->module_callback();
                memset(usart_instances[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle, usart_instances[i]->recv_buff, usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == usart_instances[i]->usart_handle) {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle, usart_instances[i]->recv_buff, usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}