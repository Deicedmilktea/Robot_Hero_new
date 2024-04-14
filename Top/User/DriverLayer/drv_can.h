#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "main.h"
void CAN1_Init(void);
void CAN2_Init(void);
void can_remote(uint8_t sbus_buf[], uint32_t can_send_id);
void can_vision(uint8_t sbus_buf[], uint8_t can_send_id); // 调用can来发送vision数据

// CAN2发送信号（摩擦轮+拨盘+抬头）
void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

// CAN1发送信号（底盘+云台）
void set_motor_current_can1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

// CAN2发送信号（摩擦轮+拨盘+抬头）
void set_motor_current_can22(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif