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
#include "stm32g4xx_hal.h"

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
#define EN_PSFB_Pin GPIO_PIN_2
#define EN_PSFB_GPIO_Port GPIOC
#define EN_PFC_Pin GPIO_PIN_3
#define EN_PFC_GPIO_Port GPIOC
#define ADC_PFC_V_Pin GPIO_PIN_0
#define ADC_PFC_V_GPIO_Port GPIOA
#define ADC_OUT_V_Pin GPIO_PIN_1
#define ADC_OUT_V_GPIO_Port GPIOA
#define ADC_OUT_I_Pin GPIO_PIN_2
#define ADC_OUT_I_GPIO_Port GPIOA
#define ADC_BAT_V_Pin GPIO_PIN_3
#define ADC_BAT_V_GPIO_Port GPIOA
#define DAC_CC_Pin GPIO_PIN_4
#define DAC_CC_GPIO_Port GPIOA
#define DAC_CV_Pin GPIO_PIN_5
#define DAC_CV_GPIO_Port GPIOA
#define ADC_TEMP1_Pin GPIO_PIN_6
#define ADC_TEMP1_GPIO_Port GPIOA
#define ADC_TEMP2_Pin GPIO_PIN_7
#define ADC_TEMP2_GPIO_Port GPIOA
#define ADC_TEMP3_Pin GPIO_PIN_4
#define ADC_TEMP3_GPIO_Port GPIOC
#define ADC_PFC_I_Pin GPIO_PIN_0
#define ADC_PFC_I_GPIO_Port GPIOB
#define PWM_FAN2_Pin GPIO_PIN_6
#define PWM_FAN2_GPIO_Port GPIOC
#define PWM_FAN1_Pin GPIO_PIN_8
#define PWM_FAN1_GPIO_Port GPIOA
#define RELAY_CTRL_Pin GPIO_PIN_9
#define RELAY_CTRL_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
