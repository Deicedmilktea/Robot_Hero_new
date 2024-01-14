#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern INS_t INS;
extern UART_HandleTypeDef huart1;

ins_data_t ins_data;
uint8_t temp[4];

void exchange_task()
{
    while (1) {
        ins_data.angle[0] = INS.Yaw;
        ins_data.angle[1] = INS.Roll;
        ins_data.angle[2] = INS.Pitch;

        temp[0] = ((int)INS.Yaw >> 8) & 0xff;
        temp[1] = ((int)INS.Yaw) & 0xff;
        temp[2] = ((int)INS.Roll >> 8) & 0xff;
        temp[3] = ((int)INS.Roll) & 0xff;
        // for (int i = 0; i < 4; i++) {
        //     HAL_UART_Transmit(&huart1, temp[0], sizeof(temp[0]), HAL_MAX_DELAY);
        // }

        osDelay(1);
    }
}