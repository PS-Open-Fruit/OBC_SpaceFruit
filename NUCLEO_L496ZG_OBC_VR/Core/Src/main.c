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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD
#define MAX_FRAME_SIZE 512 // Increased for Image Chunks
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
uint16_t SLIP_Encode(uint8_t* payload, uint16_t len, uint8_t* out_buffer);
uint16_t SLIP_Decode(uint8_t* frame, uint16_t len, uint8_t* out_payload);
void OBC_Process_Loop(void);
void OBC_On_Receive(uint8_t* Buf, uint32_t *Len);
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
  printf("[OBC] System Booted. VCP Active.\r\n");
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
  return ch;
}

// --- SLIP Encoder ---
uint16_t SLIP_Encode(uint8_t* payload, uint16_t len, uint8_t* out_buffer) {
    uint16_t idx = 0;
    out_buffer[idx++] = FEND;
    out_buffer[idx++] = 0x00; // Header
    
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = payload[i];
        if (c == FEND) {
            out_buffer[idx++] = FESC;
            out_buffer[idx++] = TFEND;
        } else if (c == FESC) {
            out_buffer[idx++] = FESC;
            out_buffer[idx++] = TFESC;
        } else {
            out_buffer[idx++] = c;
        }
    }
    out_buffer[idx++] = FEND;
    return idx;
}

// --- SLIP Decoder ---
uint16_t SLIP_Decode(uint8_t* frame, uint16_t len, uint8_t* out_payload) {
    if (len < 3) return 0;
    if (frame[0] != FEND || frame[len-1] != FEND) return 0;
    if (frame[1] != 0x00) return 0; // Invalid Header
    
    uint16_t out_idx = 0;
    for (uint16_t i = 2; i < len - 1; i++) {
        uint8_t c = frame[i];
        if (c == FESC) {
            i++;
            if (i >= len - 1) return 0; // Error
            if (frame[i] == TFEND) out_payload[out_idx++] = FEND;
            else if (frame[i] == TFESC) out_payload[out_idx++] = FESC;
        } else {
            out_payload[out_idx++] = c;
        }
    }
    return out_idx;
}

// --- Main Process Loop (Runs in while(1)) ---
void OBC_Process_Loop(void) {
    static uint32_t last_tick = 0;
    static uint16_t next_chunk_to_req = 0;
    static uint8_t download_active = 0;
    
    // 1. Process Received Packets (Reading Buffer B)
    if (is_frame_ready) {
        uint8_t decoded[512]; // Increased buffer size
        uint16_t dec_len = SLIP_Decode(app_process_buffer, app_process_len, decoded);
        
        if (dec_len > 0) {
             uint8_t cmd_id = decoded[0];
             
             // Check for PONG (0x10 is Ping, typically Payload replies with raw PONG string if configured that way, 
             // but let's assume standard framing. The PONG string detection remains for legacy test)
             if (dec_len == 4 && strncmp((char*)decoded, "PONG", 4) == 0) {
                 printf("[OBC] >> PONG Received! Link Active.\r\n");
                 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
             }
             // Handle Image Capture Response (0x12)
             else if (cmd_id == 0x12) {
                 // Format: [0x12] [ImgID:2] [Size:4]
                 uint32_t img_size;
                 memcpy(&img_size, &decoded[3], 4);
                 printf("[OBC] Image Captured! Size: %lu bytes. Starting Download...\r\n", img_size);
                 
                 // Start Downloading
                 download_active = 1;
                 next_chunk_to_req = 0;

                 // Kickstart download immediately
                 uint8_t req[3] = {0x13, 0x00, 0x00};
                 uint8_t tx_frame[16];
                 uint16_t len = SLIP_Encode(req, 3, tx_frame);
                 CDC_Transmit_FS(tx_frame, len);
             }
             // Handle Status Response (0x11)
             else if (cmd_id == 0x11) {
                 // Payload: [0x11] [CPU:1] [Temp:4] [RAM:2]
                 if (dec_len >= 8) {
                     uint8_t cpu = decoded[1];
                     float temp;
                     uint16_t ram;
                     memcpy(&temp, &decoded[2], 4);
                     memcpy(&ram, &decoded[6], 2);
                     
                     // Manual Float-to-Int conversion to avoid %f linker issues
                     int t_int = (int)temp;
                     int t_dec = (int)((temp - t_int) * 10);
                     if (t_dec < 0) t_dec = -t_dec; // Handle negative decimals
                     
                     // Print Formatted String (This goes to LPUART1 -> PC)
                     printf("[OBC] STATUS >> CPU: %d%% | Temp: %d.%d C | RAM: %d MB\r\n", cpu, t_int, t_dec, ram);
                 }
             }
             // Handle Image Chunk (0x13)
             else if (cmd_id == 0x13) {
                 // Format: [0x13] [ChunkID:2] [Data...]
                 uint16_t chunk_id;
                 memcpy(&chunk_id, &decoded[1], 2);
                 uint16_t data_len = dec_len - 3;
                 uint8_t* raw_data = &decoded[3];
                 
                 if (chunk_id == next_chunk_to_req) {
                     // Toggle LED for Visual Feedback
                     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

                     // SEND COMPLIANT KISS PROTOCOL FRAME (OBC -> GS)
                     // Packet Format: [FEND] [0x00] [Escaped Payload] [FEND]
                     // Payload Format: [ChunkID:2] [Data:N] [CRC:2]
                     
                     // 1. Construct Raw Payload (ChunkID + Data)
                     uint8_t payload_buffer[600];
                     payload_buffer[0] = chunk_id & 0xFF;
                     payload_buffer[1] = (chunk_id >> 8) & 0xFF;
                     memcpy(&payload_buffer[2], raw_data, data_len);
                     
                     uint16_t current_payload_len = 2 + data_len;

                     // 2. Append CRC-32 (Hardware - STM32L4)
                     // ZLIB CRC32 Standard: XOR output with 0xFFFFFFFF
                     uint32_t crc_hw = HAL_CRC_Calculate(&hcrc, (uint32_t*)payload_buffer, current_payload_len);
                     crc_hw ^= 0xFFFFFFFF; 

                     payload_buffer[current_payload_len++] = crc_hw & 0xFF;
                     payload_buffer[current_payload_len++] = (crc_hw >> 8) & 0xFF;
                     payload_buffer[current_payload_len++] = (crc_hw >> 16) & 0xFF;
                     payload_buffer[current_payload_len++] = (crc_hw >> 24) & 0xFF;

                     // 3. Encode (Escapes + FENDs)
                     // Packet Format: [FEND] [0x00] [Escaped Payload] [FEND]
                     // Payload Format: [ChunkID:2] [Data:N] [CRC:4]
                     uint8_t kiss_tx_buffer[1200];
                     uint16_t kiss_len = SLIP_Encode(payload_buffer, current_payload_len, kiss_tx_buffer);
                     
                     HAL_UART_Transmit(&hlpuart1, kiss_tx_buffer, kiss_len, 2000);
                     
                     // Advance Counter
                     next_chunk_to_req++;

                     // Stop condition (Empty chunk or small chunk means EOF)
                     if (data_len < 200) { 
                         printf("[OBC] Download Complete!\r\n");
                         download_active = 0;
                     } else {
                         // REQ NEXT CHUNK
                         uint8_t req[3];
                         req[0] = 0x13;
                         req[1] = next_chunk_to_req & 0xFF;
                         req[2] = (next_chunk_to_req >> 8) & 0xFF;
                         
                         uint8_t tx_frame[16];
                         uint16_t len = SLIP_Encode(req, 3, tx_frame);
                         CDC_Transmit_FS(tx_frame, len);
                     }
                 }
             }
             else {
                 printf("[OBC] Rx Frame: %d bytes. Payload: ", dec_len);
                 for(int k=0; k<dec_len; k++) printf("%02X ", decoded[k]);
                 printf("\r\n");
             }
        }
        // Mark Buffer B as empty, allowing ISR to fill it again
        is_frame_ready = 0; 
    }

    // 1b. Process Ground Station Packets (LPUART1)
    if (is_gs_frame_ready) {
        uint8_t decoded_gs[512];
        uint16_t dec_len = SLIP_Decode(gs_app_buffer, gs_app_len, decoded_gs);
        
        if (dec_len > 0) {
             uint8_t cmd_id = decoded_gs[0];
             
             if (cmd_id == 0x12) { // Capture
                 uint8_t cmd[1] = {0x12};
                 uint8_t tx[32];
                 uint16_t len = SLIP_Encode(cmd, 1, tx);
                 CDC_Transmit_FS(tx, len);
                 printf("[OBC] GS CMD: Capture!\r\n");
             }
             else if (cmd_id == 0x10) { // Ping
                 uint8_t cmd[1] = {0x10};
                 uint8_t tx[32];
                 uint16_t len = SLIP_Encode(cmd, 1, tx);
                 CDC_Transmit_FS(tx, len);
                 printf("[OBC] GS CMD: Ping!\r\n");
             }
             else if (cmd_id == 0x20) { // Status Request
                 uint8_t cmd[1] = {0x11}; // Map to VR Status Cmd
                 uint8_t tx[32];
                 uint16_t len = SLIP_Encode(cmd, 1, tx);
                 CDC_Transmit_FS(tx, len);
                 printf("[OBC] GS CMD: Status Req!\r\n");
             }
        }
        is_gs_frame_ready = 0;
    }

    // 2. Periodic Tasks
    if (HAL_GetTick() - last_tick > 1000) {
        last_tick = HAL_GetTick();
        
        // Heartbeat LED (Red)
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
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
