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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define USB_RX_SIZE 2048

typedef struct usb_data{
  uint32_t len;
  uint8_t is_new_message;
  uint8_t usb_buff[USB_RX_SIZE];
} usb_data_t;

extern usb_data_t usb_buff;
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
#define TEMP_ALERT_Pin GPIO_PIN_13
#define TEMP_ALERT_GPIO_Port GPIOF
#define INT_SCL_Pin GPIO_PIN_14
#define INT_SCL_GPIO_Port GPIOF
#define INT_SDA_Pin GPIO_PIN_15
#define INT_SDA_GPIO_Port GPIOF
#define RTC_INT_Pin GPIO_PIN_0
#define RTC_INT_GPIO_Port GPIOG
#define SD_MUX_Pin GPIO_PIN_15
#define SD_MUX_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_2
#define SD_SCK_GPIO_Port GPIOG
#define SD_MISO_Pin GPIO_PIN_3
#define SD_MISO_GPIO_Port GPIOG
#define SD_MOSI_Pin GPIO_PIN_4
#define SD_MOSI_GPIO_Port GPIOG
#define SD_CS_Pin GPIO_PIN_5
#define SD_CS_GPIO_Port GPIOG
#define DBG_TX_Pin GPIO_PIN_12
#define DBG_TX_GPIO_Port GPIOC
#define NOR_RST_Pin GPIO_PIN_0
#define NOR_RST_GPIO_Port GPIOD
#define NOR_SCK_Pin GPIO_PIN_1
#define NOR_SCK_GPIO_Port GPIOD
#define DBG_RX_Pin GPIO_PIN_2
#define DBG_RX_GPIO_Port GPIOD
#define NOR_MISO_Pin GPIO_PIN_3
#define NOR_MISO_GPIO_Port GPIOD
#define NOR_MOSI_Pin GPIO_PIN_4
#define NOR_MOSI_GPIO_Port GPIOD
#define NOR_CS_Pin GPIO_PIN_5
#define NOR_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
