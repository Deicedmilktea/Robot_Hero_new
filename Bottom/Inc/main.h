/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_voltage;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�?
}moto_info_t;


//�궨��
#define MOTOR_MAX_NUM 7		//��������ֽ���?
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//Խ���򸳱߽�ֵ
#define FEEDBACK_ID_BASE      0x201
#define FEEDBACK_ID_BASE_6020 0x205
#define CAN_CONTROL_ID_BASE   0x200
#define CAN_CONTROL_ID_EXTEND 0x1ff
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//ȫ�ֱ���
extern uint16_t can_cnt_1;
extern uint16_t can_cnt_2;
extern float target_speed[7];//ʵ��������ת��320rpm
extern float target_speed_can_2[7];
extern moto_info_t motor_info[MOTOR_MAX_NUM];		//��������7���ֽ�
extern moto_info_t motor_info_can_2[MOTOR_MAX_NUM];
extern uint8_t can_flag;
extern double step; 
extern double r;
extern double target_v;
extern int16_t target_int1;
extern int16_t target_int2;//���ڵ�����ת��ֱ��
extern double target_curl;
extern float yuntai_step;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//����
extern float time;
extern float time_count;
extern uint8_t flag_shoot;
extern float round_shoot;
extern float down;
extern float up;
//��ʱ������
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
//Yaw��
#define yaw_front 4096
#define yaw_L 30.0f
#define tyro 3.1415f*7.5f
extern int16_t target_angle;
extern int16_t err_angle;
extern int16_t max_yaw_speed;
extern float small;
extern float angle_limit;

extern uint8_t rx_data[8];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_SENSOR_PIN2_Pin GPIO_PIN_7
#define HALL_SENSOR_PIN2_GPIO_Port GPIOI
#define HALL_SENSOR_PIN2_EXTI_IRQn EXTI9_5_IRQn
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_HEAT_Pin GPIO_PIN_6
#define IMU_HEAT_GPIO_Port GPIOF
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define door2_Pin GPIO_PIN_2
#define door2_GPIO_Port GPIOC
#define door2_EXTI_IRQn EXTI2_IRQn
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define door1_Pin GPIO_PIN_15
#define door1_GPIO_Port GPIOB
#define door1_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
