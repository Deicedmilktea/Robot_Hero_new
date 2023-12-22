#include "rc_potocal.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
//底盘电机结构体
extern motor_info_t  motor_can1[6];
int16_t Rotate_w;

//IMU
extern ins_data_t ins_data;
// flag for keyboard
uint16_t w_flag;
uint16_t s_flag;	
uint16_t a_flag;
uint16_t d_flag;
uint16_t q_flag;
uint16_t e_flag;
uint16_t shift_flag;
uint16_t ctrl_flag;
uint8_t press_left;
uint8_t press_right;
uint16_t r_flag;
uint16_t f_flag;
uint16_t g_flag;
uint16_t z_flag;
uint16_t x_flag;
uint16_t c_flag;
uint16_t v_flag;
uint16_t b_flag;

RC_ctrl_t rc_ctrl;
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

void USART3_rxDataHandler(uint8_t *rxBuf)
{

}