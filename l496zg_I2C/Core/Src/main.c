/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LM73_ADDR (0x4D << 1)
#define MCP79411_ADDR (0x6F << 1)
#define LM73_REG_CTRL 0x04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_Scan(void)
{
    char msg[64];
    HAL_StatusTypeDef res;
    int devices = 0;

    printf("Scanning I2C bus...\r\n");

    for (uint16_t addr = 1; addr < 128; addr++)
    {
        // HAL expects 8-bit address, so shift left by 1
        res = HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 10);

        if (res == HAL_OK)
        {
            sprintf(msg, "I2C device found at 0x%02X\r\n", addr);
            printf("%s", msg);
            devices++;
        }
        else if (res == HAL_ERROR)
        {
            // ไม่ตอบสนอง -> ข้าม
        }
        else if (res == HAL_TIMEOUT)
        {
            // Timeout -> ข้าม
        }
    }

    if (devices == 0)
    {
        printf("No I2C devices found.\r\n");
    }
    else
    {
        printf("Scan done. %d device(s) found.\r\n", devices);
    }
}

// void lm73Init(void)
// {
//     uint8_t data;

//     // อ่านค่าเดิมจาก CTRL register
//     if (HAL_I2C_Mem_Read(&hi2c1, LM73_ADDR, LM73_REG_CTRL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
//         printf("LM73 init read failed!\r\n");
//         return;
//     }

//     printf("CTRL before: 0x%02X\r\n", data);

//     // set bit[7:6] = 11
//     data |= (0b11 << 6);

//     if (HAL_I2C_Mem_Write(&hi2c1, LM73_ADDR, LM73_REG_CTRL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
//         printf("LM73 init write failed!\r\n");
//     }

//     printf("CTRL after : 0x%02X\r\n", data);
// }

uint8_t lm73GetStatus(void)
{
    uint8_t data;
    if (HAL_I2C_Mem_Read(&hi2c1, LM73_ADDR, LM73_REG_CTRL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("LM73 status read fail\r\n");
        return 0xFF;
    }
    return data;
}


float lm73GetTemperature()
{
    uint8_t reg = 0x00;          // Register to read temperature
    uint8_t tempBuff[2] = {0};

    // Send register pointer (write)
    if (HAL_I2C_Master_Transmit(&hi2c1, LM73_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle error (optional)
        return -1000.0f;
    }

    // Read 2 bytes from the LM73
    if (HAL_I2C_Master_Receive(&hi2c1, LM73_ADDR, tempBuff, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle error (optional)
        return -1000.0f;
    }

    // Temperature decoding
    float tempHigh = (tempBuff[0] & 0x7F) * 2.0f;
    float tempLow = (tempBuff[1] >> 2) * 0.03125f;

    return tempHigh + tempLow;
}

/* Simple BCD -> binary conversion for MCP79411 time fields */
static uint8_t bcd2bin(uint8_t b)
{
  return (uint8_t)(((b >> 4) * 10U) + (b & 0x0F));
}

float mcp79411GetTime(void)
{
  // MCP79411 time registers are BCD encoded. Register 0 has ST bit in bit7.
  uint8_t timeBuff[7];
  if (HAL_I2C_Mem_Read(&hi2c1, MCP79411_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, timeBuff, 7, HAL_MAX_DELAY) != HAL_OK)
  {
    printf("MCP79411 read fail\r\n");
    return -1000.0f;
  }

  // If oscillator stop bit not set? Bit7 = ST (1 = running). If 0, start it.
  if ((timeBuff[0] & 0x80) == 0)
  {
    uint8_t sec = timeBuff[0] | 0x80; // set ST bit
    if (HAL_I2C_Mem_Write(&hi2c1, MCP79411_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      printf("MCP79411 oscillator started\r\n");
    }
    else
    {
      printf("MCP79411 start fail\r\n");
    }
    return 0.0f; // not valid time yet
  }

  uint8_t sec_bcd  = (uint8_t)(timeBuff[0] & 0x7F);
  uint8_t min_bcd  = (uint8_t)(timeBuff[1] & 0x7F);
  uint8_t hour_bcd = (uint8_t)(timeBuff[2] & 0x3F); // 24h mode assumed (bit6 = 0)

  uint8_t sec  = bcd2bin(sec_bcd);
  uint8_t min  = bcd2bin(min_bcd);
  uint8_t hour = bcd2bin(hour_bcd);

  if (sec > 59 || min > 59 || hour > 23)
  {
    printf("MCP79411 invalid BCD time (%u:%u:%u)\r\n", hour, min, sec);
    return -1000.0f;
  }

  return (float)hour + (float)min / 60.0f + (float)sec / 3600.0f;
}

// New: read Hours/Minutes/Seconds separately (returns HAL status-style int)
int mcp79411GetHMS(uint8_t *h, uint8_t *m, uint8_t *s)
{
  uint8_t buf[7];
  if (HAL_I2C_Mem_Read(&hi2c1, MCP79411_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7, HAL_MAX_DELAY) != HAL_OK)
    return -1; // I2C error
  if ((buf[0] & 0x80) == 0)
  {
    // Start oscillator if stopped
    uint8_t sec = buf[0] | 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MCP79411_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec, 1, HAL_MAX_DELAY);
    return -2; // oscillator was stopped
  }
  uint8_t sec  = bcd2bin((uint8_t)(buf[0] & 0x7F));
  uint8_t min  = bcd2bin((uint8_t)(buf[1] & 0x7F));
  uint8_t hour = bcd2bin((uint8_t)(buf[2] & 0x3F));
  if (sec > 59 || min > 59 || hour > 23)
    return -3; // invalid data
  *s = sec; *m = min; *h = hour;
  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  I2C_Scan();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("-----LM73----\r\n");
    printf("Get Status read : 0x%02X\r\n",lm73GetStatus());
    printf("Get Temperature read : %.2f\r\n",lm73GetTemperature());
    printf("-----end-----\r\n");
    HAL_Delay(500);
    printf("---MCP79411--\r\n");
    uint8_t hh, mm, ss; int tstat = mcp79411GetHMS(&hh,&mm,&ss);
    if (tstat == 0)
      printf("Time: %02u:%02u:%02u\r\n", hh, mm, ss);
    else if (tstat == -2)
      printf("RTC oscillator started, time not valid yet\r\n");
    else
      printf("RTC read error (%d)\r\n", tstat);
    printf("-----end-----\r\n");
    HAL_Delay(1500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00805B85;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init 1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
