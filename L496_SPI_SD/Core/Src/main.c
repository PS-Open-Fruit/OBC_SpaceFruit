/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
#define DB_SIZE 4096
uint8_t rxBufferA[DB_SIZE];
uint8_t rxBufferB[DB_SIZE];
volatile uint32_t rxIndex = 0;
volatile uint8_t activeBufferIdx = 0; // 0 = A, 1 = B
volatile uint8_t bufferA_Ready = 0;
volatile uint8_t bufferB_Ready = 0;
volatile uint8_t bufferOverrun = 0;
uint8_t rxTempByte;

volatile uint32_t lastByteTime = 0;
volatile uint32_t totalBytesReceived = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //some variables for FatFs
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations

    //Buffer for f_mkfs work area and file reading
    //Must be at least the size of one sector (512 bytes) for f_mkfs
    BYTE workBuffer[512];

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
  	    printf("f_mount error (%i)\r\n", fres);
        if (fres == FR_NO_FILESYSTEM) {
            printf("No valid FAT filesystem found. Attempting to format to FAT32...\r\n");
            //Format the card. FM_ANY will choose FAT32 for 64GB usually.
            fres = f_mkfs("", FM_ANY, 0, workBuffer, sizeof(workBuffer));
            if (fres == FR_OK) {
                printf("Format successful! Remounting...\r\n");
                fres = f_mount(&FatFs, "", 1);
                if (fres == FR_OK) {
                    printf("Remount successful!\r\n");
                } else {
                    printf("Remount failed error (%i)\r\n", fres);
                    while(1);
                }
            } else {
                printf("Format failed error (%i)\r\n", fres);
                while(1);
            }
        } else {
  	        while(1);
        }
    }

    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
  	printf("f_getfree error (%i)\r\n", fres);
  	while(1);
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    //Now let's try to open file "test.txt"
    fres = f_open(&fil, "test.txt", FA_READ);
    if (fres != FR_OK) {
  	    //If file doesn't exist (e.g. after format), just print and continue
        printf("Could not open 'test.txt' for reading (Error %i). Skipping read test.\r\n", fres);
    } else {
        printf("I was able to open 'test.txt' for reading!\r\n");
    
        //We can either use f_read OR f_gets to get data out of files
        //f_gets is a wrapper on f_read that does some string formatting for us
        TCHAR* rres = f_gets((TCHAR*)workBuffer, 512, &fil);
    if(rres != 0) {
  	printf("Read string from 'test.txt' contents: %s\r\n", workBuffer);
    } else {
  	printf("f_gets error (%i)\r\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);
    }

    //Now let's try and write a file "write.txt"
    fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(fres == FR_OK) {
  	printf("I was able to open 'write.txt' for writing\r\n");
    } else {
  	printf("f_open error (%i)\r\n", fres);
    }

    //Copy in a string
    char text[] = "this is test for STM32L496ZG use FatFs library with SPI SD Card reader to save the text.";
    snprintf((char*)workBuffer, 512, text);
    UINT bytesWrote;
    fres = f_write(&fil, workBuffer, strlen(text), &bytesWrote);
    if(fres == FR_OK) {
  	printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
    } else {
  	printf("f_write error (%i)\r\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);
    printf("close complete\r\n");

    // --- Verify Write ---
    printf("Verifying write...\r\n");
    fres = f_open(&fil, "write.txt", FA_READ);
    if (fres == FR_OK) {
        memset(workBuffer, 0, sizeof(workBuffer)); // Clear buffer
        TCHAR* rres = f_gets((TCHAR*)workBuffer, 512, &fil);
        if(rres != 0) {
            printf("Read back from 'write.txt': %s\r\n", workBuffer);
        } else {
            printf("f_gets on write.txt failed\r\n");
        }
        f_close(&fil);
    } else {
        printf("Failed to open 'write.txt' for verification (Error %i)\r\n", fres);
    }
    // --------------------

    // --- USB CDC Image Receive Experiment ---
    printf("Starting Image Rx (Protocol: START:<filename> -> Data)...\r\n");
    
    // Externs from usbd_cdc_if.c
    extern uint8_t UserRxBufferFS_A[];
    extern uint8_t UserRxBufferFS_B[];
    extern volatile uint8_t Buffer_A_Ready;
    extern volatile uint8_t Buffer_B_Ready;
    extern volatile uint32_t Buffer_A_Length;
    extern volatile uint32_t Buffer_B_Length;
    extern void CDC_Buffer_Processed(uint8_t buffer_id);
    extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len); // Extern Transmit

    // Reset Globals
    totalBytesReceived = 0;
    uint32_t lastHeartbeat = 0;
    lastByteTime = HAL_GetTick();
    
    // Ensure flags are clear
    Buffer_A_Ready = 0;
    Buffer_B_Ready = 0;

    // State Machine
    // 0: Waiting for Start Command
    // 1: Receiving Data
    uint8_t receiveState = 0; 
    char currentFilename[64] = {0};
    uint32_t startTime = 0;
    
    printf("Waiting for command...\r\n");

    while(1) {
         // --- Heartbeat (Red LED) ---
         if((HAL_GetTick() - lastHeartbeat) > 500) {
             HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
             lastHeartbeat = HAL_GetTick();
         }

         // check exit condition (Timeout while receiving)
         if (receiveState == 1 && (HAL_GetTick() - lastByteTime) > 3000) {
             printf("\r\nEnd of transmission (Timeout). Closing file.\r\n");
             f_close(&fil);
             
              // Calc Speed
              uint32_t endTime = HAL_GetTick();
              uint32_t duration_ms = endTime - startTime;
              if (duration_ms == 0) duration_ms = 1;
              uint64_t speed_calc = ((uint64_t)totalBytesReceived * 1000 * 100) / ((uint64_t)1024 * duration_ms);
              printf("Duration: %lu ms\r\n", duration_ms);
              printf("Received: %lu bytes\r\n", totalBytesReceived);
              printf("Speed: %lu.%02lu KB/s\r\n", (uint32_t)(speed_calc/100), (uint32_t)(speed_calc%100));
              
              // --- Save Log to SD Card ---
              FIL logFile;
              if(f_open(&logFile, "log.txt", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
                  char logBuf[128];
                  int len = snprintf(logBuf, sizeof(logBuf), "Tick:%lu, File:%s, Size:%lu B, Time:%lu ms, Speed:%lu.%02lu KB/s\r\n", 
                                     HAL_GetTick(), currentFilename, totalBytesReceived, duration_ms, (uint32_t)(speed_calc/100), (uint32_t)(speed_calc%100));
                  
                  UINT bw_log;
                  f_write(&logFile, logBuf, len, &bw_log);
                  f_close(&logFile);
                  printf("Log saved to log.txt\r\n");
              } else {
                  printf("Failed to open log.txt\r\n");
              }

              break; // Exit Experiment Loop
          }
         
         // Process Buffer A
         if(Buffer_A_Ready) {
             lastByteTime = HAL_GetTick();
             HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Blue LED Activity

             if (receiveState == 0) {
                 // Check for START: command
                 if (strncmp((char*)UserRxBufferFS_A, "START:", 6) == 0 && Buffer_A_Length > 6) {
                     memset(currentFilename, 0, sizeof(currentFilename));
                     uint32_t nameLen = Buffer_A_Length - 6;
                     if(nameLen > 63) nameLen = 63;
                     memcpy(currentFilename, UserRxBufferFS_A + 6, nameLen);
                     
                     // Sanitize filename (remove non-printable)
                     for(int i=0; i<nameLen; i++) {
                         if(currentFilename[i] < 32 || currentFilename[i] > 126) currentFilename[i] = 0;
                     }

                     printf("CMD Received. File: %s\r\n", currentFilename);
                     fres = f_open(&fil, currentFilename, FA_WRITE | FA_CREATE_ALWAYS);
                     if(fres == FR_OK) {
                         receiveState = 1;
                         startTime = HAL_GetTick();
                         totalBytesReceived = 0;
                         CDC_Transmit_FS((uint8_t*)"OK", 2); // Send ACK
                         printf("File Open. Sending ACK. Receiving...\r\n");
                     } else {
                         printf("Create File Failed: %d\r\n", fres);
                         CDC_Transmit_FS((uint8_t*)"ERR", 3);
                     }
                 } else {
                     printf("Ignored Invalid Start Packet (Len: %lu)\r\n", Buffer_A_Length);
                 }
             } 
             else if (receiveState == 1) {
                 // Write Data
                 UINT bw;
                 fres = f_write(&fil, UserRxBufferFS_A, Buffer_A_Length, &bw);
                 if(fres != FR_OK) printf("Write Error: %d\r\n", fres);
                 totalBytesReceived += Buffer_A_Length;
             }
             
             CDC_Buffer_Processed(0); // Release A
         }

         // Process Buffer B
         if(Buffer_B_Ready) {
             lastByteTime = HAL_GetTick();
             HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); 

             if (receiveState == 0) {
                 // Usually commands come in A if single packet, but handle B just in case
                  if (strncmp((char*)UserRxBufferFS_B, "START:", 6) == 0 && Buffer_B_Length > 6) {
                     memset(currentFilename, 0, sizeof(currentFilename));
                     uint32_t nameLen = Buffer_B_Length - 6;
                     if(nameLen > 63) nameLen = 63;
                     memcpy(currentFilename, UserRxBufferFS_B + 6, nameLen);
                     
                     for(int i=0; i<nameLen; i++) {
                         if(currentFilename[i] < 32 || currentFilename[i] > 126) currentFilename[i] = 0;
                     }

                     printf("CMD Received (B). File: %s\r\n", currentFilename);
                     fres = f_open(&fil, currentFilename, FA_WRITE | FA_CREATE_ALWAYS);
                     if(fres == FR_OK) {
                         receiveState = 1;
                         startTime = HAL_GetTick();
                         totalBytesReceived = 0;
                         CDC_Transmit_FS((uint8_t*)"OK", 2);
                         printf("File Open. Sending ACK. Receiving...\r\n");
                     } else {
                         printf("Create File Failed: %d\r\n", fres);
                         CDC_Transmit_FS((uint8_t*)"ERR", 3);
                     }
                 }
             } 
             else if (receiveState == 1) {
                 UINT bw;
                 fres = f_write(&fil, UserRxBufferFS_B, Buffer_B_Length, &bw);
                 if(fres != FR_OK) printf("Write Error: %d\r\n", fres);
                 totalBytesReceived += Buffer_B_Length;
             }
             
             CDC_Buffer_Processed(1); // Release B
         }
    }
    // ----------------------

    // ------------------

    //We're done, so de-mount the drive
    HAL_Delay(100);
    f_mount(NULL, "", 0);
    printf("unmount complete\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
