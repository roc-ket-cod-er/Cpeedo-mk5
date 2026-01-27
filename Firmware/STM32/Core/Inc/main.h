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
#include "stm32u5xx_hal.h"

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
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define INA226_POWER_Pin GPIO_PIN_3
#define INA226_POWER_GPIO_Port GPIOE
#define TP2_RST_Pin GPIO_PIN_2
#define TP2_RST_GPIO_Port GPIOC
#define TP2_INT_Pin GPIO_PIN_3
#define TP2_INT_GPIO_Port GPIOC
#define POWER_GOOD_Pin GPIO_PIN_3
#define POWER_GOOD_GPIO_Port GPIOA
#define BMM150_PWR_Pin GPIO_PIN_4
#define BMM150_PWR_GPIO_Port GPIOA
#define E_INK_POWER_Pin GPIO_PIN_5
#define E_INK_POWER_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOC
#define ESP_POWER_EN_Pin GPIO_PIN_5
#define ESP_POWER_EN_GPIO_Port GPIOC
#define INK_BUSY_Pin GPIO_PIN_10
#define INK_BUSY_GPIO_Port GPIOE
#define INK_RST_Pin GPIO_PIN_11
#define INK_RST_GPIO_Port GPIOE
#define INK_DC_Pin GPIO_PIN_12
#define INK_DC_GPIO_Port GPIOE
#define SPI2_CS3_Pin GPIO_PIN_13
#define SPI2_CS3_GPIO_Port GPIOE
#define SD_CD_Pin GPIO_PIN_14
#define SD_CD_GPIO_Port GPIOE
#define TP1_RST_Pin GPIO_PIN_15
#define TP1_RST_GPIO_Port GPIOE
#define SPEED_SENSE_Pin GPIO_PIN_15
#define SPEED_SENSE_GPIO_Port GPIOB
#define TP1_INT_Pin GPIO_PIN_8
#define TP1_INT_GPIO_Port GPIOD
#define SPEED_SENSED9_Pin GPIO_PIN_9
#define SPEED_SENSED9_GPIO_Port GPIOD
#define SPI2_CS1_Pin GPIO_PIN_0
#define SPI2_CS1_GPIO_Port GPIOD
#define SPI2_CS2_Pin GPIO_PIN_5
#define SPI2_CS2_GPIO_Port GPIOD
#define IMU_POWER_Pin GPIO_PIN_7
#define IMU_POWER_GPIO_Port GPIOD
#define ICM_INT2_Pin GPIO_PIN_0
#define ICM_INT2_GPIO_Port GPIOE
#define ICM_INT2_EXTI_IRQn EXTI0_IRQn
#define ICM_INT1_Pin GPIO_PIN_1
#define ICM_INT1_GPIO_Port GPIOE
#define ICM_INT1_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
