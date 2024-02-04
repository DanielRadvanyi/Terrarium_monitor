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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "z_displ_ILI9XXX.h"
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
#define DISPL_RST_Pin GPIO_PIN_3
#define DISPL_RST_GPIO_Port GPIOF
#define DISPL_LED_Pin GPIO_PIN_0
#define DISPL_LED_GPIO_Port GPIOC
#define DISPL_SCK_Pin GPIO_PIN_5
#define DISPL_SCK_GPIO_Port GPIOA
#define TOUCH_MISO_Pin GPIO_PIN_6
#define TOUCH_MISO_GPIO_Port GPIOA
#define Green_Pin GPIO_PIN_0
#define Green_GPIO_Port GPIOB
#define TOUCH_CS_Pin GPIO_PIN_2
#define TOUCH_CS_GPIO_Port GPIOB
#define Red_Pin GPIO_PIN_14
#define Red_GPIO_Port GPIOB
#define DISPL_CS_Pin GPIO_PIN_14
#define DISPL_CS_GPIO_Port GPIOD
#define DISPL_DC_Pin GPIO_PIN_15
#define DISPL_DC_GPIO_Port GPIOD
#define TOUCH_INT_Pin GPIO_PIN_6
#define TOUCH_INT_GPIO_Port GPIOG
#define TOUCH_INT_EXTI_IRQn EXTI9_5_IRQn
#define DISPL_MOSI_Pin GPIO_PIN_5
#define DISPL_MOSI_GPIO_Port GPIOB
#define Button_Interrupt_Pin GPIO_PIN_7
#define Button_Interrupt_GPIO_Port GPIOB
#define Button_Interrupt_EXTI_IRQn EXTI9_5_IRQn
#define Yellow_Pin GPIO_PIN_1
#define Yellow_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
