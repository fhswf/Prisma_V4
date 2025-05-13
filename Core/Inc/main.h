/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SO_A_Pin GPIO_PIN_0
#define SO_A_GPIO_Port GPIOC
#define SO_B_Pin GPIO_PIN_1
#define SO_B_GPIO_Port GPIOC
#define SO_C_Pin GPIO_PIN_2
#define SO_C_GPIO_Port GPIOC
#define TOUCHREF_Pin GPIO_PIN_3
#define TOUCHREF_GPIO_Port GPIOC
#define BTN0_Pin GPIO_PIN_0
#define BTN0_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOA
#define DISP_CS_Pin GPIO_PIN_4
#define DISP_CS_GPIO_Port GPIOA
#define BMI088_INT1_Pin GPIO_PIN_5
#define BMI088_INT1_GPIO_Port GPIOA
#define BMI088_INT1_EXTI_IRQn EXTI9_5_IRQn
#define BMI088_INT3_Pin GPIO_PIN_6
#define BMI088_INT3_GPIO_Port GPIOA
#define BMI088_INT3_EXTI_IRQn EXTI9_5_IRQn
#define DISP_ON_Pin GPIO_PIN_7
#define DISP_ON_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_4
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_5
#define BT_RX_GPIO_Port GPIOC
#define BT_KEY_Pin GPIO_PIN_0
#define BT_KEY_GPIO_Port GPIOB
#define BT_PWRN_Pin GPIO_PIN_1
#define BT_PWRN_GPIO_Port GPIOB
#define DISP_RST_Pin GPIO_PIN_2
#define DISP_RST_GPIO_Port GPIOB
#define MOT_PGOOD_Pin GPIO_PIN_10
#define MOT_PGOOD_GPIO_Port GPIOB
#define MOT_PGOOD_EXTI_IRQn EXTI15_10_IRQn
#define NFAULT_Pin GPIO_PIN_11
#define NFAULT_GPIO_Port GPIOB
#define NSLEEP_Pin GPIO_PIN_12
#define NSLEEP_GPIO_Port GPIOB
#define HALL_A_Pin GPIO_PIN_13
#define HALL_A_GPIO_Port GPIOB
#define HALL_A_EXTI_IRQn EXTI15_10_IRQn
#define HALL_B_Pin GPIO_PIN_14
#define HALL_B_GPIO_Port GPIOB
#define HALL_B_EXTI_IRQn EXTI15_10_IRQn
#define HALL_C_Pin GPIO_PIN_15
#define HALL_C_GPIO_Port GPIOB
#define HALL_C_EXTI_IRQn EXTI15_10_IRQn
#define HS_A_Pin GPIO_PIN_6
#define HS_A_GPIO_Port GPIOC
#define HS_B_Pin GPIO_PIN_7
#define HS_B_GPIO_Port GPIOC
#define HS_C_Pin GPIO_PIN_8
#define HS_C_GPIO_Port GPIOC
#define USB_PWRD_Pin GPIO_PIN_9
#define USB_PWRD_GPIO_Port GPIOC
#define LS_A_Pin GPIO_PIN_8
#define LS_A_GPIO_Port GPIOA
#define LS_B_Pin GPIO_PIN_9
#define LS_B_GPIO_Port GPIOA
#define LS_C_Pin GPIO_PIN_10
#define LS_C_GPIO_Port GPIOA
#define DISP_SCK_Pin GPIO_PIN_10
#define DISP_SCK_GPIO_Port GPIOC
#define DISP_MOSI_Pin GPIO_PIN_12
#define DISP_MOSI_GPIO_Port GPIOC
#define DISP_DC_Pin GPIO_PIN_2
#define DISP_DC_GPIO_Port GPIOD
#define NEOPXL1_Pin GPIO_PIN_3
#define NEOPXL1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define NEOPXL2_Pin GPIO_PIN_6
#define NEOPXL2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
