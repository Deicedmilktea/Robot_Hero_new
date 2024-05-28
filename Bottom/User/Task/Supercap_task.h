#ifndef SUPERCAP_TASK_H
#define SUPERCAP_TASK_H

// 超电接收数据
typedef struct
{
    uint16_t voltage; // 超电电压
    uint16_t power;   // 超电功率
    uint8_t state;    // 超电状态
} SupercapRxData_t;

// 超电发送数据
typedef struct
{
    uint16_t buffer; // 缓冲能量
    uint16_t power;  // 底盘功率
    uint8_t state;   // 超电状态
} SupercapTxData_t;

typedef enum
{
    SUPERCAP_STATE_OFF = 2,
    SUPERCAP_STATE_ON,
} supercap_state_e;

#endif