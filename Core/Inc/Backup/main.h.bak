/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
int spi_reg_write(uint8_t reg,uint8_t*data,int len);
int spi_reg_receive(uint8_t reg,uint8_t*data,int len);
void spi_dummy();
int spi4_reg_write(uint8_t reg,uint8_t*data,int len);
int spi4_reg_receive(uint8_t reg,uint8_t*data,int len);
int i2c4_reg_write(uint8_t addr,uint8_t reg,uint8_t*data,int len);
int i2c4_reg_receive(uint8_t addr,uint8_t reg,uint8_t*data,int len);


void UART7_IT(void);
void UART4_IT(void);
void UART2_IT(void);
void UART8_IT();
void uart2_it();
void delay_us(int us);
void main_task();

extern float target_yaw;
extern float target_depth;
extern int timer;
extern uint8_t uart8_rx_buffer[64];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
