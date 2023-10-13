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
#include "stm32wbxx_hal.h"

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
#define NB_Button_Pin GPIO_PIN_0
#define NB_Button_GPIO_Port GPIOC
#define SB_Button_Pin GPIO_PIN_1
#define SB_Button_GPIO_Port GPIOC
#define WB_Button_Pin GPIO_PIN_2
#define WB_Button_GPIO_Port GPIOC
#define EB_Button_Pin GPIO_PIN_3
#define EB_Button_GPIO_Port GPIOC
#define WB_Y_Pin GPIO_PIN_2
#define WB_Y_GPIO_Port GPIOA
#define WB_G_Pin GPIO_PIN_3
#define WB_G_GPIO_Port GPIOA
#define SB_Y_Pin GPIO_PIN_5
#define SB_Y_GPIO_Port GPIOA
#define NB_R_Pin GPIO_PIN_6
#define NB_R_GPIO_Port GPIOA
#define NB_Y_Pin GPIO_PIN_7
#define NB_Y_GPIO_Port GPIOA
#define SB_G_Pin GPIO_PIN_8
#define SB_G_GPIO_Port GPIOA
#define NB_G_Pin GPIO_PIN_9
#define NB_G_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_0
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define WB_R_Pin GPIO_PIN_6
#define WB_R_GPIO_Port GPIOC
#define EB_G_Pin GPIO_PIN_10
#define EB_G_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define EB_R_Pin GPIO_PIN_15
#define EB_R_GPIO_Port GPIOA
#define EB_Y_Pin GPIO_PIN_10
#define EB_Y_GPIO_Port GPIOC
#define SB_R_Pin GPIO_PIN_12
#define SB_R_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_6
#define STLINK_RX_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
