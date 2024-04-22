#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "main.h"

void CAN1_Init(void);
void CAN2_Init(void);
void can_remote(uint8_t sbus_buf[], uint32_t can_send_id);

#endif