/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_VTEC_Pin GPIO_PIN_0
#define ADC_VTEC_GPIO_Port GPIOC
#define ADC_ITEC_Pin GPIO_PIN_1
#define ADC_ITEC_GPIO_Port GPIOC
#define ADC_NTC_Pin GPIO_PIN_0
#define ADC_NTC_GPIO_Port GPIOA
#define SW_VCC1_Pin GPIO_PIN_1
#define SW_VCC1_GPIO_Port GPIOA
#define SW_VCC2_Pin GPIO_PIN_2
#define SW_VCC2_GPIO_Port GPIOA
#define SW_VCC3_Pin GPIO_PIN_3
#define SW_VCC3_GPIO_Port GPIOA
#define TEC_SET_Pin GPIO_PIN_4
#define TEC_SET_GPIO_Port GPIOA
#define TEC_EN_Pin GPIO_PIN_5
#define TEC_EN_GPIO_Port GPIOA
#define DUT_nRST_Pin GPIO_PIN_6
#define DUT_nRST_GPIO_Port GPIOA
#define DUT_nTXDIS_Pin GPIO_PIN_7
#define DUT_nTXDIS_GPIO_Port GPIOA
#define VCC1_STA_Pin GPIO_PIN_0
#define VCC1_STA_GPIO_Port GPIOB
#define VCC2_STA_Pin GPIO_PIN_1
#define VCC2_STA_GPIO_Port GPIOB
#define VCC3_STA_Pin GPIO_PIN_2
#define VCC3_STA_GPIO_Port GPIOB
#define ALM_VCC1_Pin GPIO_PIN_12
#define ALM_VCC1_GPIO_Port GPIOB
#define ALM_VCC2_Pin GPIO_PIN_13
#define ALM_VCC2_GPIO_Port GPIOB
#define ALM_VCC3_Pin GPIO_PIN_14
#define ALM_VCC3_GPIO_Port GPIOB
#define ALM_TEC_PWR_Pin GPIO_PIN_15
#define ALM_TEC_PWR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define USART1_DE_Pin GPIO_PIN_11
#define USART1_DE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
