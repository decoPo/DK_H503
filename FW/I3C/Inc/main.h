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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
  char *        TARGET_NAME;          /*!< Marketing Target reference */
  uint32_t      TARGET_ID;            /*!< Target Identifier on the Bus */
  uint64_t      TARGET_BCR_DCR_PID;   /*!< Concatenation value of PID, BCR and DCR of the target */
  uint8_t       STATIC_ADDR;          /*!< Static Address of the target, value found in the datasheet of the device */
  uint8_t       DYNAMIC_ADDR;         /*!< Dynamic Address of the target preset by software/application */
} TargetDesc_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

#define DEVICE_ID1      0U
#define DEVICE_ID2      1U
#define DEVICE_ID3      2U
#define DEVICE_ID4      3U
#define DEVICE_ID5      4U
#define DEVICE_ID6      5U
#define DEVICE_ID7      6U
#define DEVICE_ID8      7U
#define DEVICE_ID9      8U
#define DEVICE_ID10     9U

#define TARGET1_DYN_ADDR        0x32
#define TARGET2_DYN_ADDR        0x33
#define TARGET3_DYN_ADDR        0x34
#define TARGET4_DYN_ADDR        0x35
#define TARGET5_DYN_ADDR        0x36
#define TARGET6_DYN_ADDR        0x37
#define TARGET7_DYN_ADDR        0x38
#define TARGET8_DYN_ADDR        0x39
#define TARGET9_DYN_ADDR        0x3a
#define TARGET10_DYN_ADDR       0x3b

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
