/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // IMPORTANT: For STM32 HAL, you MUST call GetTime before GetDate to unlock the RTC registers.
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  // Pack the time and date into the FatFs 32-bit format
  return (DWORD)(
    // STM32 year is 0-99 representing 2000-2099. 
    // FatFs year is an offset from 1980. So, we add 20 to the STM32 year.
    ((DWORD)(sDate.Year + 20) << 25) | 
    ((DWORD)sDate.Month       << 21) | 
    ((DWORD)sDate.Date        << 16) | 
    ((DWORD)sTime.Hours       << 11) | 
    ((DWORD)sTime.Minutes     << 5)  | 
    ((DWORD)sTime.Seconds / 2)         // FatFs expects seconds divided by 2
  );
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
