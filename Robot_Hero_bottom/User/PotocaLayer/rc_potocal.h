#ifndef RC_POTOCAL
#define RC_POTOCAL
  #include "struct_typedef.h"
  #include "drv_can.h"
  #include "main.h"

void USART3_rxDataHandler(uint8_t *rxBuf);


typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

extern uint16_t w_flag;
extern uint16_t s_flag;
extern uint16_t a_flag;
extern uint16_t d_flag;
extern uint16_t q_flag;
extern uint16_t e_flag;
extern uint16_t shift_flag;
extern uint16_t ctrl_flag;
extern uint8_t press_left;
extern uint8_t press_right;
extern uint16_t r_flag;
extern uint16_t f_flag;
extern uint16_t g_flag;
extern uint16_t z_flag;
extern uint16_t x_flag;
extern uint16_t c_flag;
extern uint16_t v_flag;
extern uint16_t b_flag;




#endif