/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "my_lis2dw12.h"
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
void send_2_dbg ( uint8_t* buff ) ;
int32_t my_lis2dw12_platform_write ( void* , uint8_t , const uint8_t* , uint16_t ) ;
int32_t my_lis2dw12_platform_read ( void* , uint8_t , uint8_t* , uint16_t ) ;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LDG_Pin GPIO_PIN_5
#define LDG_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_14
#define SPI1_CS_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ACC_INT1_Pin GPIO_PIN_8
#define ACC_INT1_GPIO_Port GPIOB
#define ACC_INT1_EXTI_IRQn EXTI4_15_IRQn
#define ACC_INT2_Pin GPIO_PIN_9
#define ACC_INT2_GPIO_Port GPIOB
#define ACC_INT2_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
#define HUART_DBG						&huart2
#define HSPI1							&hspi1
#define UART_TIMEOUT 					1000
#define UART_TX_MAX_BUFF_SIZE			250
#define UART_TX_TIMEOUT					100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
