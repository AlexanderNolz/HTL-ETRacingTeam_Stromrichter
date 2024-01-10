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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define sin_in_Pin GPIO_PIN_1
#define sin_in_GPIO_Port GPIOC
#define cos_in_Pin GPIO_PIN_2
#define cos_in_GPIO_Port GPIOC
#define Uzk_mess_Pin GPIO_PIN_3
#define Uzk_mess_GPIO_Port GPIOC
#define Strom_V_Pin GPIO_PIN_0
#define Strom_V_GPIO_Port GPIOA
#define Strom_U_Pin GPIO_PIN_1
#define Strom_U_GPIO_Port GPIOA
#define Strom_W_Pin GPIO_PIN_5
#define Strom_W_GPIO_Port GPIOA
#define Gaspedal_Pin GPIO_PIN_4
#define Gaspedal_GPIO_Port GPIOC
#define GATE_Treiber_RST_ENA_Pin GPIO_PIN_2
#define GATE_Treiber_RST_ENA_GPIO_Port GPIOB
#define Trenner_Pin GPIO_PIN_10
#define Trenner_GPIO_Port GPIOB
#define Strom_filter_clock_Pin GPIO_PIN_8
#define Strom_filter_clock_GPIO_Port GPIOC
#define cos_sin_clock_Pin GPIO_PIN_9
#define cos_sin_clock_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
