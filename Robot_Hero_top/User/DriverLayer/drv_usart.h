#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "main.h"

void DRV_USART3_IRQHandler(UART_HandleTypeDef *huart);
void DRV_USART6_IRQHandler(UART_HandleTypeDef *huart);
void USART3_Init(void);
void USART6_Init(void);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

#endif