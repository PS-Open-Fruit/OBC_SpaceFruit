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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdarg.h>
#include "../../OBC_Software/Common/obc_packet.h"
#include "../../OBC_Software/Subsystems/VR/subsystem_vr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
// --- Buffer A: ISR Accumulator (High Priority) ---
// Used exclusively by OBC_On_Receive() to build up incoming frames byte-by-byte.
uint8_t isr_rx_buffer[MAX_FRAME_SIZE];
uint8_t isr_rx_idx = 0;
uint8_t uart_rx_char; // for single-byte UART interrupt

// --- Buffer B: Application Processor (Low Priority) ---
// Safe copy of the frame for the main loop to process without blocking interrupts.
uint8_t app_process_buffer[MAX_FRAME_SIZE];
uint16_t app_process_len = 0;

// Flag: 1 = Buffer B has data pending, 0 = Buffer B is empty/processed
volatile uint8_t is_frame_ready = 0;

// --- Ground Station Buffers (LPUART1) ---
uint8_t gs_isr_buffer[MAX_FRAME_SIZE];
uint8_t gs_isr_idx = 0;
uint8_t gs_app_buffer[MAX_FRAME_SIZE];
uint16_t gs_app_len = 0;
volatile uint8_t is_gs_frame_ready = 0;
// --- LED Timers ---
uint32_t gs_led_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void OBC_Process_Loop(void);
void OBC_On_Receive(uint8_t* Buf, uint32_t *Len);
void OBC_Log(const char *fmt, ...);
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
  setvbuf(stdout, NULL, _IONBF, 0); // Disable buffering for printf
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  OBC_Packet_Init(&hcrc);
  VR_Init();
  OBC_Log("[OBC] System Booted. VCP Active.");
  HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OBC_Process_Loop();
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hlpuart1.Init.BaudRate = 9600;
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
  // Manually disable FIFO and set baudrate again to be sure (since regeneration might have messed flags)
  hlpuart1.Init.BaudRate = 9600;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END MX_GPIO_Init_1 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// For printf() via STLink UART
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  
  // Blink GS LED
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  gs_led_timer = HAL_GetTick();

  return ch;
}

// --- OBC Logging (Framed) ---
void OBC_Log(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uint16_t msg_len = strlen(buf);
    // Frame Format: [0x01] [ASCII Message] [CRC32]
    uint8_t payload[300];
    payload[0] = 0x01; // Log Command ID
    if (msg_len > 250) msg_len = 250; 
    memcpy(&payload[1], buf, msg_len);
    
    // Calculate CRC (CMD + Content)
    uint32_t crc = OBC_Calculate_CRC(payload, msg_len + 1);
    
    uint16_t p_idx = msg_len + 1;
    memcpy(&payload[p_idx], &crc, 4);
    p_idx += 4;

    uint8_t tx_frame[600];
    uint16_t enc_len = SLIP_Encode(payload, p_idx, tx_frame);
    HAL_UART_Transmit(&hlpuart1, tx_frame, enc_len, 1000);

    // Blink GS LED
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    gs_led_timer = HAL_GetTick();
}



// --- Main Process Loop (Runs in while(1)) ---
void OBC_Process_Loop(void) {
    static uint32_t last_tick = 0;
    
    // 1. Process Received Packets (VR Simulator - Buffer B)
    if (is_frame_ready) {
        uint8_t decoded[512]; 
        uint16_t dec_len = SLIP_Decode(app_process_buffer, app_process_len, decoded);
        
        // Pass to VR Subsystem
        VR_Handle_Packet(decoded, dec_len);

        // Mark Buffer B as empty
        is_frame_ready = 0; 
    }

    // 1b. Process Ground Station Packets (LPUART1)
    if (is_gs_frame_ready) {
        uint8_t decoded_gs[512];
        uint16_t dec_len = SLIP_Decode(gs_app_buffer, gs_app_len, decoded_gs);
        
        // Validate CRC (GS -> OBC)
        uint8_t gs_crc_ok = 0;
        if (dec_len >= 5) {
            //  [KISS_CMD] + [CRC:4] = 5 bytes (Empty payload)
             uint32_t rx_crc;
             memcpy(&rx_crc, &decoded_gs[dec_len-4], 4);
             
             uint32_t calc_crc = OBC_Calculate_CRC(decoded_gs, dec_len-4);
             
             if (calc_crc == rx_crc) gs_crc_ok = 1;
             else OBC_Log("[OBC] GS CRC Fail: Rx %08X vs Calc %08X", rx_crc, calc_crc);
        } else if (dec_len > 0) {
             OBC_Log("[OBC] GS Error: Payload too short (%d)", dec_len);
        }

        if (gs_crc_ok) {
             // Index 0 is KISS CMD (should be 0x00), Index 1 is APP CMD
             uint8_t kiss_cmd = decoded_gs[0];
             uint8_t cmd_id = decoded_gs[1];
             
             if (kiss_cmd == 0x00) {
                 if (cmd_id == 0x12) { 
                     OBC_Log("[OBC] GS CMD: Capture!");
                     if (VR_IsOnline()) VR_SendCmd(VR_CMD_CAPTURE_RES);
                     else OBC_Log("[OBC] VR Offline! Cannot Capture.");
                 }
                 else if (cmd_id == 0x10) { 
                     OBC_Log("[OBC] GS CMD: Ping!");
                     if (VR_IsOnline()) VR_RequestGSPing();
                     else OBC_Log("[OBC] VR Offline! Pong failed.");
                 }
                 else if (cmd_id == 0x20) { 
                     OBC_Log("[OBC] GS CMD: Status Req!");
                     if (VR_IsOnline()) VR_SendCmd(VR_CMD_STATUS_RES); 
                     else OBC_Log("[OBC] VR Offline! No Status.");
                 }
             }
        }
    
        is_gs_frame_ready = 0;
    }

    // LED Off Logic (Blink effect)
    if (gs_led_timer > 0 && (HAL_GetTick() - gs_led_timer > 50)) {
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
        gs_led_timer = 0;
    }

    // 2. Periodic Tasks
    if (HAL_GetTick() - last_tick > 1000) {
        last_tick = HAL_GetTick();
        
        // Heartbeat LED (Red)
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    
    // Run VR Update frequently to handle 20ms LED timing and timeouts
    VR_Update();
}

// --- UART Error Callback ---
// Restart reception if an error (Overrun, Noise, Framing) occurs
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        // Clear Error Flags (handled by HAL, but good to be explicit about intent)
        // Restart Reception
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    }
}

// --- UART RX Callback (PC Commands) ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        
        // 1. Accumulate Byte
        if (gs_isr_idx < MAX_FRAME_SIZE) {
            gs_isr_buffer[gs_isr_idx++] = uart_rx_char;
        }
        
        // 2. Sync Check: Reset if first byte isn't FEND
        if (gs_isr_idx == 1 && uart_rx_char != FEND) {
            gs_isr_idx = 0; 
        }
        
        // 3. End of Frame Check
        if (gs_isr_idx > 1 && uart_rx_char == FEND) {
            if (is_gs_frame_ready == 0) {
                 // Move to App Buffer
                 memcpy(gs_app_buffer, gs_isr_buffer, gs_isr_idx);
                 gs_app_len = gs_isr_idx;
                 is_gs_frame_ready = 1;
            }
            gs_isr_idx = 0;
        }

        // Re-arm interrupt
        HAL_UART_Receive_IT(&hlpuart1, &uart_rx_char, 1);
    }
}

// --- Receive Handler (Called from usbd_cdc_if.c ISR) ---
// This fills Buffer A (Accumulator) and copies to Buffer B when complete
void OBC_On_Receive(uint8_t* Buf, uint32_t *Len) {
    for (uint32_t i = 0; i < *Len; i++) {
        uint8_t byte = Buf[i];
        
        // 1. Accumulate Byte in Buffer A
        if (isr_rx_idx < MAX_FRAME_SIZE) {
            isr_rx_buffer[isr_rx_idx++] = byte;
        }
        
        // 2. Sync Check: Reset if first byte isn't Start Marker (FEND)
        if (isr_rx_idx == 1 && byte != FEND) {
            isr_rx_idx = 0; 
        }
        
        // 3. End of Frame Check
        if (isr_rx_idx > 1 && byte == FEND) {
            // Frame is complete. Try to move Buffer A -> Buffer B
            
            if (is_frame_ready == 0) {
                 // Buffer B is free. Perform the Copy.
                 memcpy(app_process_buffer, isr_rx_buffer, isr_rx_idx);
                 app_process_len = isr_rx_idx;
                 
                 // Signal Main Loop
                 is_frame_ready = 1;
            }
            // else: Buffer B is still busy being processed by Main Loop.
            // We drop this frame to prevent data corruption.
            
            // Reset Buffer A Index for next frame
            isr_rx_idx = 0;
        }
    }
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
