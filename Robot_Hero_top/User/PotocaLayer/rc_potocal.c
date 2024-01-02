#include "rc_potocal.h"
#include "INS_task.h"
#include "exchange.h"
// 底盘电机结构体
extern motor_info_t motor_can1[6];
int16_t Rotate_w;

// IMU
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

uint8_t temp_remote[8];
uint8_t temp_imu_angle[3]; // 发送imu数据
RC_ctrl_t rc_ctrl;
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	rc_ctrl.rc.ch[0] = (rxBuf[0] | (rxBuf[1] << 8)) & 0x07ff;				  //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
	rc_ctrl.rc.ch[1] = (((rxBuf[1] >> 3) & 0xff) | (rxBuf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl.rc.ch[2] = (((rxBuf[2] >> 6) & 0xff) | (rxBuf[3] << 2) |		  //!< Channel 2
						(rxBuf[4] << 10)) &
					   0x07ff;
	rc_ctrl.rc.ch[3] = (((rxBuf[4] >> 1) & 0xff) | (rxBuf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl.rc.s[0] = ((rxBuf[5] >> 4) & 0x0003);							  //!< Switch left！！！这尼玛是右
	rc_ctrl.rc.s[1] = ((rxBuf[5] >> 4) & 0x000C) >> 2;						  //!< Switch right！！！这才是左
	rc_ctrl.mouse.x = rxBuf[6] | (rxBuf[7] << 8);							  //!< Mouse X axis
	rc_ctrl.mouse.y = rxBuf[8] | (rxBuf[9] << 8);							  //!< Mouse Y axis
	rc_ctrl.mouse.z = rxBuf[10] | (rxBuf[11] << 8);							  //!< Mouse Z axis
	rc_ctrl.mouse.press_l = rxBuf[12];										  //!< Mouse Left Is Press ?
	rc_ctrl.mouse.press_r = rxBuf[13];										  //!< Mouse Right Is Press ?
	rc_ctrl.key.v = rxBuf[14] | (rxBuf[15] << 8);							  //!< KeyBoard value
	rc_ctrl.rc.ch[4] = rxBuf[16] | (rxBuf[17] << 8);						  // NULL

	rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

	for (int i = 0; i <= 7; i++)
	{
		temp_remote[i] = rxBuf[i]; // volatile const uint8_t和uint8_t不一样不能直接带入can_remote这个函数
	}
	can_remote(temp_remote, 0x33);

	for (int i = 8; i <= 15; i++)
	{
		temp_remote[i - 8] = rxBuf[i]; // volatile const uint8_t和uint8_t不一样不能直接带入can_remote这个函数
	}
	can_remote(temp_remote, 0x34);

	temp_remote[0] = rxBuf[16];
	temp_remote[1] = rxBuf[17];

	INS.Yaw = 100 * INS.Yaw; // 使之接收带上小数点
	INS.Roll = 100 * INS.Roll;
	INS.Pitch = 100 * INS.Pitch;

	temp_remote[2] = ((int)INS.Yaw >> 8) & 0xff;
	temp_remote[3] = (int)INS.Yaw & 0xff;
	temp_remote[4] = ((int)INS.Roll >> 8) & 0xff;
	temp_remote[5] = ((int)INS.Roll) & 0xff;
	temp_remote[6] = ((int)INS.Pitch >> 8) & 0xff;
	temp_remote[7] = (int)INS.Pitch & 0xff;

	can_remote(temp_remote, 0x35);

	// temp_imu_angle[0]=ins_data.angle[0]; // yaw
	// temp_imu_angle[1]=ins_data.angle[1]; // roll
	// temp_imu_angle[2]=ins_data.angle[2]; // pitch

	// can_imu_angle(temp_imu_angle,0x56); // 向下C板发送imu数据

	// Some flag of keyboard
	w_flag = (rxBuf[14] & 0x01);
	s_flag = (rxBuf[14] & 0x02);
	a_flag = (rxBuf[14] & 0x04);
	d_flag = (rxBuf[14] & 0x08);
	q_flag = (rxBuf[14] & 0x40);
	e_flag = (rxBuf[14] & 0x80);
	shift_flag = (rxBuf[14] & 0x10);
	ctrl_flag = (rxBuf[14] & 0x20);
	press_left = rc_ctrl.mouse.press_l;
	press_right = rc_ctrl.mouse.press_r;
	// HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_11);
	r_flag = rc_ctrl.key.v & (0x00 | 0x01 << 8);
	f_flag = rc_ctrl.key.v & (0x00 | 0x02 << 8);
	g_flag = rc_ctrl.key.v & (0x00 | 0x04 << 8);
	z_flag = rc_ctrl.key.v & (0x00 | 0x08 << 8);
	x_flag = rc_ctrl.key.v & (0x00 | 0x10 << 8);
	c_flag = rc_ctrl.key.v & (0x00 | 0x20 << 8);
	v_flag = rc_ctrl.key.v & (0x00 | 0x40 << 8);
	b_flag = rc_ctrl.key.v & (0x00 | 0x80 << 8);
}