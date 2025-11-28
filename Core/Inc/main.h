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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Soil_Sensor_PA0_Pin GPIO_PIN_0
#define Soil_Sensor_PA0_GPIO_Port GPIOA
#define LDR_PA1_Pin GPIO_PIN_1
#define LDR_PA1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Servo_PWM_PA5_Pin GPIO_PIN_5
#define Servo_PWM_PA5_GPIO_Port GPIOA
#define RGB_LED_PWM_PB0_Pin GPIO_PIN_0
#define RGB_LED_PWM_PB0_GPIO_Port GPIOB
#define DHT22_PB10_Pin GPIO_PIN_10
#define DHT22_PB10_GPIO_Port GPIOB
#define TRIG_FOR_HCSRO4_Pin GPIO_PIN_8
#define TRIG_FOR_HCSRO4_GPIO_Port GPIOA
#define ECHO_for_HCSRO4_Pin GPIO_PIN_9
#define ECHO_for_HCSRO4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RGB_LED_PWM_PB4_Pin GPIO_PIN_4
#define RGB_LED_PWM_PB4_GPIO_Port GPIOB
#define RGB_LED_PWM_PB5_Pin GPIO_PIN_5
#define RGB_LED_PWM_PB5_GPIO_Port GPIOB
#define Buzzer_PWM_PB6_Pin GPIO_PIN_6
#define Buzzer_PWM_PB6_GPIO_Port GPIOB
#define LCD_PB8_Pin GPIO_PIN_8
#define LCD_PB8_GPIO_Port GPIOB
#define LCD_PB9_Pin GPIO_PIN_9
#define LCD_PB9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
