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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "mt25ql.h"
#include "rv3028c7.h"
#include "tmp1075.h"
#include "littlefs_port.h"
#include "kiss_utils.h"
#include "obc_helper.h"
#include "eps_protocol.h"
#include "sd_spi.h"
#include "kiss_protocol.h"
#include "protocol_utils.h"
#include "commu_helper.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

usb_data_t usb_buff = {
    .is_new_message = 0,
    .len = 0,
};

obc_sensor_data_t obc_sensors_data;
eps_sensor_data_t eps_sensors_data;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EPS_RECV_LEN 64 // Increased size for safety
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c4;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USBTask */
osThreadId_t USBTaskHandle;
const osThreadAttr_t USBTask_attributes = {
  .name = "USBTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for wdtFeed */
osThreadId_t wdtFeedHandle;
const osThreadAttr_t wdtFeed_attributes = {
  .name = "wdtFeed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for sensorQuery */
osThreadId_t sensorQueryHandle;
const osThreadAttr_t sensorQuery_attributes = {
  .name = "sensorQuery",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for uartRxTask */
osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = {
  .name = "uartRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for cdcDataQueue */
osMessageQueueId_t cdcDataQueueHandle;
uint8_t cdcDataQueueBuffer[ 128 * sizeof( usb_data_t ) ];
osStaticMessageQDef_t cdcDataQueueControlBlock;
const osMessageQueueAttr_t cdcDataQueue_attributes = {
  .name = "cdcDataQueue",
  .cb_mem = &cdcDataQueueControlBlock,
  .cb_size = sizeof(cdcDataQueueControlBlock),
  .mq_mem = &cdcDataQueueBuffer,
  .mq_size = sizeof(cdcDataQueueBuffer)
};
/* Definitions for epsUartQueue */
osMessageQueueId_t epsUartQueueHandle;
const osMessageQueueAttr_t epsUartQueue_attributes = {
  .name = "epsUartQueue"
};
/* Definitions for printQueue */
osMessageQueueId_t printQueueHandle;
const osMessageQueueAttr_t printQueue_attributes = {
  .name = "printQueue"
};
/* Definitions for sensorsMutex */
osMutexId_t sensorsMutexHandle;
const osMutexAttr_t sensorsMutex_attributes = {
  .name = "sensorsMutex"
};
/* Definitions for epsFlag */
osEventFlagsId_t epsFlagHandle;
const osEventFlagsAttr_t epsFlag_attributes = {
  .name = "epsFlag"
};
/* USER CODE BEGIN PV */

#define DOWNLINK_WINDOW_SIZE 5

#define NO_COMMU_TIMEOUT 10000
#define DATA_POLLING_INTERVAL 1000
#define BEACON_INTERVAL 10000

const uint32_t COMMU_RX_TIMEOUT = 3000;
#define MAX_BEACON_PACKET_SIZE 256U


#define SYSTEM_STATE_BEACON         0x00000001U
#define SYSTEM_STATE_DOWNLINK       0x00000002U
#define SYSTEM_STATE_WAIT_ACK       0x00000004U
#define SYSTEM_STATE_ALL            0x00FFFFFFU

osEventFlagsId_t systemStateFlagHandle;
const osEventFlagsAttr_t systemStateFlag_attr = {
  .name = "systemStateFlag"
};

typedef enum{
      PAYLOAD_STATE_IDLE,
      PAYLOAD_STATE_RX,
} payload_state;

#define PAYLOAD_RX_TIMEOUT 1000


uint8_t eps_state = EPS_DATA_NO_DATA;

osEventFlagsId_t payloadFlagHandle;
const osEventFlagsAttr_t payloadFlag_attr = {
  .name = "payloadFlag"
};

uint8_t kiss_buffer[512];
kiss_frame_t payload_kiss;

osSemaphoreId_t norSemaphoreHandle;
const osSemaphoreAttr_t norSemaphoreAttr = {
  .name = "norFlashSemaphore"
};

osSemaphoreId_t epsSemaphoreHandle;
const osSemaphoreAttr_t epsSemaphoreAttr = {
  .name = "epsSemaphore"
};

osSemaphoreId_t sdTxSemaphoreHandle;
osSemaphoreId_t sdRxSemaphoreHandle;

const osSemaphoreAttr_t sdRxSemaphoreAttr = {
  .name = "sdRxSemaphore"
};
const osSemaphoreAttr_t sdTxSemaphoreAttr = {
  .name = "sdTxSemaphore"
};

#define COM_UART huart4
#define EPS_UART huart2
#define DBG_UART huart5
#define SD_SPI hspi1
#define NOR_SPI hspi2

#define EVENT_FLAG_ERROR              0x80000000U
#define EPS_FLAG_POLL_START           0x00000001U
#define EPS_FLAG_POLL_SUCCESS         0x00000002U 
#define EPS_FLAG_POLL_ERROR           0x00000004U
#define EPS_FLAG_POLL_TIMEOUT         0x00000008U

#define PAYLOAD_FLAG_IDLE             0x00000020U
#define PAYLOAD_FLAG_POLL_CAPTURE     0x00000001U
#define PAYLOAD_FLAG_POLL_STATUS      0x00000002U
#define PAYLOAD_FLAG_IMAGE_REQUEST    0x00000004U
#define PAYLOAD_FLAG_IMAGE_TRANSFER   0x00000008U
#define PAYLOAD_FLAG_IMAGE_DATA       0x00000010U

// usb_data_t usb_buff;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C4_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
void mainTask(void *argument);
void usbTask(void *argument);
void wdtFeedTask(void *argument);
void sensorQueryTask(void *argument);
void logTask(void *argument);
void uartRx(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_CAN2_Init();
  MX_I2C4_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  commu_init();
  eps_init();
  // HAL_CAN_Start(&hcan2);

  // rv3028c7_set_hour(&rtc,8);
  payload_kiss.content = kiss_buffer;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sensorsMutex */
  sensorsMutexHandle = osMutexNew(&sensorsMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  sdTxSemaphoreHandle = osSemaphoreNew(1,0,&sdTxSemaphoreAttr);
  sdRxSemaphoreHandle = osSemaphoreNew(1,0,&sdRxSemaphoreAttr);
  norSemaphoreHandle = osSemaphoreNew(1,0,&norSemaphoreAttr);
  epsSemaphoreHandle = osSemaphoreNew(1,0,&epsSemaphoreAttr);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of cdcDataQueue */
  cdcDataQueueHandle = osMessageQueueNew (128, sizeof(usb_data_t), &cdcDataQueue_attributes);

  /* creation of epsUartQueue */
  epsUartQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &epsUartQueue_attributes);

  /* creation of printQueue */
  printQueueHandle = osMessageQueueNew (256, sizeof(uint8_t), &printQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(mainTask, NULL, &MainTask_attributes);

  /* creation of USBTask */
  USBTaskHandle = osThreadNew(usbTask, NULL, &USBTask_attributes);

  /* creation of wdtFeed */
  wdtFeedHandle = osThreadNew(wdtFeedTask, NULL, &wdtFeed_attributes);

  /* creation of sensorQuery */
  sensorQueryHandle = osThreadNew(sensorQueryTask, NULL, &sensorQuery_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(logTask, NULL, &printTask_attributes);

  /* creation of uartRxTask */
  uartRxTaskHandle = osThreadNew(uartRx, NULL, &uartRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of epsFlag */
  epsFlagHandle = osEventFlagsNew(&epsFlag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  payloadFlagHandle = osEventFlagsNew(&payloadFlag_attr);
  systemStateFlagHandle = osEventFlagsNew(&systemStateFlag_attr);
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 60;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10D19CE4;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 120;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_MUX_GPIO_Port, SD_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, NOR_RST_Pin|NOR_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TEMP_ALERT_Pin */
  GPIO_InitStruct.Pin = TEMP_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEMP_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_Pin */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_MUX_Pin */
  GPIO_InitStruct.Pin = SD_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_MUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NOR_RST_Pin NOR_CS_Pin */
  GPIO_InitStruct.Pin = NOR_RST_Pin|NOR_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
extern volatile int dma_tx_done;
extern volatile int dma_rx_done;

/* USER CODE BEGIN 4 */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SD_SPI.Instance) {
    osSemaphoreRelease(sdTxSemaphoreHandle);
  }
  else if (hspi->Instance == NOR_SPI.Instance) { // Fixed: == instead of =
    osSemaphoreRelease(norSemaphoreHandle);
  }
}

// ADDED: The missing Receive callback required for HAL_SPI_Receive_DMA
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SD_SPI.Instance) {
    osSemaphoreRelease(sdRxSemaphoreHandle);
  }
  else if (hspi->Instance == NOR_SPI.Instance) {
    osSemaphoreRelease(norSemaphoreHandle);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SD_SPI.Instance) {
    osSemaphoreRelease(sdRxSemaphoreHandle);
  }
  else if (hspi->Instance == NOR_SPI.Instance) { // Fixed: == instead of =
    osSemaphoreRelease(norSemaphoreHandle);
  }
}

/**
 * @brief  Sends a command, waits for response, validates, and decodes it.
 * @param  cmd_buf: Pointer to the KISS command packet to send
 * @param  cmd_len: Length of the command
 * @param  out_buf: Buffer to store the DECODED payload (without KISS bytes)
 * @return uint16_t: Length of decoded payload (0 if failed)
 */
uint16_t EPS_Perform_Transaction(uint8_t* cmd_buf, uint16_t cmd_len, uint8_t* out_buf)
{
    // 1. Clear Queue (Flush old/stale messages so we read the fresh response)
    osMessageQueueReset(epsUartQueueHandle);

    uint8_t frameRx[128] = {0};

    HAL_StatusTypeDef hal_ret = HAL_UARTEx_ReceiveToIdle_IT(&EPS_UART,frameRx,128);

    if (hal_ret == HAL_BUSY) {
        HAL_UART_AbortReceive(&EPS_UART);
        hal_ret = HAL_UARTEx_ReceiveToIdle_IT(&EPS_UART, frameRx, 128);
    }

    if (hal_ret != HAL_OK) {
        return 0;
    }

    // 2. Transmit Command
    if (HAL_UART_Transmit_IT(&EPS_UART, cmd_buf, cmd_len) != HAL_OK) {
        return 0;
    }
    
    // 3. Wait for Response (Timeout 500ms)
    uint16_t rx_len;
    osStatus_t status = osMessageQueueGet(epsUartQueueHandle, &rx_len, NULL, EPS_WAIT_TIMTOUT);

    if (status != osOK) { 
      // We timed out. The UART is likely still waiting for data.
      // We MUST stop it, otherwise the next call will fail with HAL_BUSY.
      HAL_UART_AbortReceive(&EPS_UART); 
      return 0; 
    }

    if (status != osOK) return 0; // Timeout

   
    if (rx_len == 0) return 0;

    // 5. Validate KISS
    if (!KISS_IsFrameComplete(frameRx, rx_len)) {
        return 0; 
    }

    // 6. Decode SLIP (Remove Escapes)
    // Returns the length of the clean data
    return KISS_Decode(frameRx, rx_len, out_buf);
}


// int _write(int file, char *ptr, int len)
//{
//     // Transmit data over the USB CDC VCP
//     CDC_Transmit_FS((uint8_t*) ptr, len);
//     return len;
// }

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  // HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);
  osMessageQueuePut(printQueueHandle,&ch,1,0xFFFF);

  return ch;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == EPS_UART.Instance) {
        osMessageQueuePut(epsUartQueueHandle, (void *)&Size, 0, 0);
    }
    if (huart->Instance == COM_UART.Instance) {
        osMessageQueuePut(communicationUartQueueHandle, &Size, 0, 0);
        // HAL_UARTEx_ReceiveToIdle_IT(&COM_UART, commu_temp_buff, COMMU_RX_SIZE);
        /* Do NOT re-arm here — let the task do it after copying commu_temp_buff */
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == &EPS_UART)
  {
    // uint8_t ready = 1;
    // printf("callback index : %u\r\n",Size);
    // osSemaphoreRelease(epsDataSemHandle);
    // osMessageQueuePut(epsUartQueueHandle, (void *)&ready, 0, 0);
  }

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_mainTask */
/**
 * @brief  Function implementing the MainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_mainTask */
void mainTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  printf("Start of mainTask\r\n");
  osThreadSuspend(sensorQueryHandle);
  gpio_t cs_flash = {
      .GPIOx = NOR_CS_GPIO_Port,
      .Pin = NOR_CS_Pin};

  gpio_t rst_flash = {
      .GPIOx = NOR_RST_GPIO_Port,
      .Pin = NOR_RST_Pin,
  };

  mt25q_t flash = {
      .flash_spi.hspi = &hspi2,
      .flash_spi.cs_pin = cs_flash,
      .flash_spi.spiSem = norSemaphoreHandle,
      .rst_pin = rst_flash,
  };



  mt25q_init(&flash);
  printf("Program Start\r\n");

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    printf("Reset by IWDG\r\n");
  }
  
  fs_init(&flash);
  lfs_file_t file;
  printf("LFS Init\r\n");
  osDelay(100);
  // mount the filesystem
  int err = lfs_mount(&lfs, &cfg);
  printf("LFS Mount %d\r\n",err);
  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err)
  {
    printf("lfs mount error\r\n");
    lfs_format(&lfs, &cfg);
    lfs_mount(&lfs, &cfg);
  }


  // read current count
  uint32_t boot_count = 0;
  lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
  // update boot count
  boot_count += 1;
  lfs_file_rewind(&lfs, &file);
  lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
  osDelay(100);

  // remember the storage is not updated until the file is closed successfully
  lfs_file_close(&lfs, &file);

  // release any resources we were using
  lfs_unmount(&lfs);
  printf("After lfs_unmount\r\n");
  printf("boot_count: %ld\r\n", boot_count);
  // print the boot count


  osThreadResume(sensorQueryHandle);
  obc_sensor_data_t _obc_sensors;
  eps_sensor_data_t _eps_sensors;

  uint8_t sensors_data_ready = 0;

   rv3028c7_t rtc = {
      .rv3028c7_i2c_hal.hi2c = &hi2c4,
      .address = 0x52,
  };
  rv3028c7_init(&rtc);
  date_time_t datetime;
  uint8_t temp_commu_data_buff[COMMU_BUF_SIZE];  // accumulation buffer
  uint8_t decode_buf[COMMU_BUF_SIZE] = {0};
  uint8_t kiss_content[COMMU_BUF_SIZE] = {0};
  kiss_frame_t output_frame = {
    .content = kiss_content
  };

  uint8_t local_eps_state = EPS_DATA_NO_DATA;
  // osDelay(2000);
  osEventFlagsSet(payloadFlagHandle,PAYLOAD_FLAG_IDLE);
  /* Infinite loop */

  uint8_t eps_beacon_buf[EPS_PACKED_BUFFER_SIZE] = {0};
  uint8_t eps_beacon_len = 0;

  uint8_t beacon_content_buf[MAX_BEACON_PACKET_SIZE] = {0};
  uint16_t beacon_content_len = 0;
  uint32_t beacon_timeNow = 0;
  osEventFlagsClear(systemStateFlagHandle,SYSTEM_STATE_ALL);
  osEventFlagsSet(systemStateFlagHandle,SYSTEM_STATE_BEACON);

  uint32_t data_polling_timeNow = 0;
  uint32_t local_state;

  uint32_t last_commu_timeNow = 0;

  uint8_t downlink_seq_num = 0;

  commu_file_data downlink_file_data;

  for(;;)
  {
    local_state = osEventFlagsGet(systemStateFlagHandle);
    uint32_t ticks = osKernelGetTickCount();
    uint32_t freq  = osKernelGetTickFreq();

      // Convert ticks to milliseconds
    uint32_t millis = (ticks * 1000U) / freq;

    if (millis - data_polling_timeNow > DATA_POLLING_INTERVAL){
        printf("local state %ld\r\n",local_state);
        sensors_data_ready = 0;
        HAL_StatusTypeDef ret = rv3028c7_read_time(&rtc, &datetime);
        osStatus_t os_ret = osMutexAcquire(sensorsMutexHandle,300);
        if (os_ret != osOK){
            printf("Aquire OBC Sensor error\r\n");
        }
        else{
            _obc_sensors.temp = obc_sensors_data.temp;
            _obc_sensors.datetime = datetime;
            _eps_sensors = eps_sensors_data;
            sensors_data_ready = 1;
            local_eps_state = eps_state;
            eps_state = EPS_DATA_CONSUMED;
            osMutexRelease(sensorsMutexHandle);
        }
      // if (!sensors_data_ready){
      //   continue;
      // }
      // printf("Temperature : %ld\r\n", _obc_sensors.temp);
      // printf("20%02d/%02d/%02d %02d:%02d:%02d\r\n", _obc_sensors.datetime.year, _obc_sensors.datetime.month, _obc_sensors.datetime.day, _obc_sensors.datetime.hour, _obc_sensors.datetime.min, _obc_sensors.datetime.sec);

      if (local_eps_state == EPS_DATA_OK){
        for (int i = 0; i < EPS_NUM_VI_CHANNEL;i++){
          uint8_t channel = _eps_sensors.vi_sensor[i].channel;
          int16_t voltage = _eps_sensors.vi_sensor[i].voltage;
          int16_t current = _eps_sensors.vi_sensor[i].current;
          eps_data_state data_state = _eps_sensors.vi_sensor[i].data_state;
          if (data_state == EPS_DATA_OK){
            // printf("VI Sensor CH %d, Volage : %dmV, Current %dmA\r\n",channel,voltage,current);
          }
          _eps_sensors.vi_sensor[i].data_state = EPS_DATA_CONSUMED;
        }
    
        for (int i = 0; i < EPS_NUM_OUTPUT_CHANNEL;i++){
          uint8_t channel = _eps_sensors.output_sensor[i].channel;
          int16_t voltage = _eps_sensors.output_sensor[i].voltage;
          int16_t current = _eps_sensors.output_sensor[i].current;
          eps_data_state data_state = _eps_sensors.output_sensor[i].data_state;
          if (data_state == EPS_DATA_OK){
            // printf("Output Sensor CH %d, Volage : %dmV, Current %dmA\r\n",channel,voltage,current);
          }
          _eps_sensors.output_sensor[i].data_state = EPS_DATA_CONSUMED;
        }
    
        for (int i = 0; i < EPS_NUM_OUTPUT_CHANNEL;i++){
          uint8_t channel = _eps_sensors.output_state[i].channel;
          uint8_t status = _eps_sensors.output_state[i].status;
          eps_data_state data_state = _eps_sensors.output_state[i].data_state;
          if (data_state != EPS_DATA_OK){
            continue;
          }
          // printf("Output State CH %d, State %d\r\n",channel,status);
          _eps_sensors.output_state[i].data_state = EPS_DATA_CONSUMED;
        }
    
        for (int i = 0; i < EPS_NUM_TEMP_BATT;i++){
          uint8_t channel = _eps_sensors.battery_temperature[i].channel;
          int16_t temp = _eps_sensors.battery_temperature[i].temperature * 1000;
          eps_data_state data_state = _eps_sensors.battery_temperature[i].data_state;
          if (data_state != EPS_DATA_OK){
            continue;
          }
          // printf("Battery Temperature CH %d, Temperature %d State %d\r\n",channel,temp,data_state);
          _eps_sensors.battery_temperature[i].data_state = EPS_DATA_CONSUMED;
        }
        local_eps_state = EPS_DATA_CONSUMED;

        eps_beacon_len = eps_pack_sensor_data(eps_beacon_buf,sizeof(_eps_sensors),&_eps_sensors);

      }


      data_polling_timeNow = millis;
      // printf("\r\n");
    }

    if (local_state & SYSTEM_STATE_DOWNLINK){
      uint8_t buf_a[2048] = {0};
      uint8_t buf_b[2048] = {0};
      uint16_t buf_len = 0;
      uint8_t status = 0;
      uint16_t actual_read_len = 0;
      FATFS FatFS;
      FRESULT ret;
      FIL file;
      ret = f_mount(&FatFS,"",1);
      if (ret != FR_OK){
        status = 1;
      }
      ret = f_open(&file,downlink_file_data.file_name,FA_READ);
      if (ret != FR_OK){
        status = 1;
      }
      ret = f_lseek(&file,downlink_file_data.file_offset);
      if (ret != FR_OK){
        status = 1;
      }
      ret = f_read(&file,buf_a,downlink_file_data.chunk_len,(UINT*)&actual_read_len);
      if (ret != FR_OK){
        status = 1;
      }
      f_close(&file);
      ret = f_mount(NULL,"",0);
      buf_len = commu_file_downlink_encode(downlink_file_data,status,buf_a,actual_read_len,buf_b);
      if (buf_len == 0){
        printf("file downlink encode error\r\n");
      }
      buf_len = commu_encode(downlink_seq_num,COMMU_PAYLOAD_ID_OBC,PID_OBC_GS_RESPONSE_FILE_DATA,buf_len,buf_b,buf_a,65535);
      if (buf_len == 0){
        printf("downlink commu encode error\r\n");
      }
      buf_len = KISS_Encode_Custom_Cmd(buf_a,KISS_CMD_DATA_FRAME,buf_len,buf_b);
      if (buf_len == 0){
        printf("downlink kiss encode error\r\n");
      }
      HAL_UART_Transmit(&COM_UART,buf_b,buf_len,1000);
      
      downlink_file_data.file_offset += actual_read_len;
      // printf("downlink %d file_name : %s chunk_len : %d, file_offset %ld\r\n",local_state,downlink_file_data.file_name,downlink_file_data.chunk_len,downlink_file_data.file_offset);
      
      downlink_seq_num++;
      last_commu_timeNow = millis;
      if (actual_read_len < downlink_file_data.chunk_len){
          printf("End of file reached. Downlink complete.\r\n");
          osEventFlagsClear(systemStateFlagHandle,SYSTEM_STATE_DOWNLINK);
          osEventFlagsSet(systemStateFlagHandle,SYSTEM_STATE_BEACON);
      }
      else if (downlink_seq_num % DOWNLINK_WINDOW_SIZE == 0){
        osEventFlagsClear(systemStateFlagHandle,SYSTEM_STATE_DOWNLINK);
        osEventFlagsSet(systemStateFlagHandle,SYSTEM_STATE_WAIT_ACK);
      }
    }

    if (millis - last_commu_timeNow > NO_COMMU_TIMEOUT){
      printf("Reset system state\r\n");
      osEventFlagsClear(systemStateFlagHandle,SYSTEM_STATE_ALL);
      osEventFlagsSet(systemStateFlagHandle,SYSTEM_STATE_BEACON);
      downlink_seq_num = 0;
      last_commu_timeNow = millis;
    }

    if ((millis - beacon_timeNow > BEACON_INTERVAL) && (local_state & SYSTEM_STATE_BEACON)){
      uint8_t date_time_buf[32] = {0};
      rv3028c7_pack_datetime(&_obc_sensors.datetime,date_time_buf);
      uint8_t date_time_buf_len = 7;

      uint8_t temperature_buf[4];
      uint8_t temperature_buf_len = tmp1075_pack_temperature(_obc_sensors.temp,temperature_buf);

      uint8_t beacon_packet[MAX_BEACON_PACKET_SIZE] = {0};
      uint16_t current_idx = 0U;
      
      (void)memcpy(&beacon_packet[current_idx], (const void *)eps_beacon_buf, (size_t)eps_beacon_len);
      current_idx += (uint16_t)eps_beacon_len;

      (void)memcpy(&beacon_packet[current_idx], (const void *)date_time_buf, (size_t)date_time_buf_len);
      current_idx += (uint16_t)date_time_buf_len;

      (void)memcpy(&beacon_packet[current_idx], (const void *)temperature_buf, (size_t)temperature_buf_len);
      current_idx += (uint16_t)temperature_buf_len;

      (void)memcpy(beacon_content_buf,(const void *)beacon_packet,(size_t)current_idx);
      beacon_content_len = current_idx;
      if (beacon_content_len > 0){
        uint8_t osi_buf[200];
        uint8_t seq_num = 0;
        uint8_t payload_id = 0;
        uint8_t osi_buf_len = commu_encode(seq_num,payload_id,PID_OBC_GS_BEACON,beacon_content_len,beacon_content_buf,osi_buf,200);
        
        uint8_t beacon_kiss[200];
        uint8_t beacon_kiss_len = KISS_Encode_Custom_Cmd(osi_buf,KISS_CMD_DATA_FRAME,osi_buf_len,beacon_kiss);
        printf("Broadcast beacon len : %d final len %d\r\n",beacon_content_len,beacon_kiss_len);
        
        HAL_UART_Transmit_IT(&COM_UART,beacon_kiss,beacon_kiss_len);
        beacon_content_len = 0;
      }
      beacon_timeNow = millis;
    }

    if (osMutexAcquire(uartMutexHandle,UART_MUTEX_TIMEOUT) == osOK){
      if (commu_data_ready){
        commu_data_ready = 0;
        printf("commu ready\r\n");
        memcpy(temp_commu_data_buff,commu_data_buff,commu_size);
        uint16_t buff_size = commu_size;
        uint8_t dekissed_buff[256];
        kiss_status_t dekissed_len = KISS_Decode(temp_commu_data_buff,buff_size,dekissed_buff);
        commu_header_t commu_request_header;
        uint8_t commu_payload[128] = {0};
        
        // Check for 0xAC (GS ACK) before COMMU decode
        if (temp_commu_data_buff[1] == 0xAC) {
            printf("[OBC] Received ACK from GS\r\n");
            osEventFlagsClear(systemStateFlagHandle, SYSTEM_STATE_WAIT_ACK);
            osEventFlagsSet(systemStateFlagHandle, SYSTEM_STATE_DOWNLINK);
            continue; // Skip COMMU decode for ACK
        }
        
        commu_status_t status_commu =  commu_decode(dekissed_buff,dekissed_len,&commu_request_header,commu_payload);
        // kiss_status_t status_kiss = KISS_UnwrapFrame(temp_commu_data_buff,buff_size,decode_buf,&output_frame);
        

        // printf("ret = %d dekissed len %d\r\n",status_commu,dekissed_len);
        if (status_commu == COMMU_VALID_DATA){
          // printf("Valid commu data\r\n");
          if (commu_request_header.payload_id == COMMU_PAYLOAD_ID_VR){
            switch (commu_request_header.pid)
            {
            case PID_GS_VR_REQUEST_COPY_IMAGE_TO_SD:
              printf("Download image command\r\n");
              uint8_t ack_msg[32] = {0x00,0xAC};
              uint16_t ack_len = 0;
              uint8_t msg[32] = {0};
              ack_len = KISS_Encode(ack_msg,2,msg);
              printf("ACK to COMMU\r\n");
              HAL_UART_Transmit_IT(&COM_UART,msg,ack_len);
              osEventFlagsSet(payloadFlagHandle,PAYLOAD_FLAG_IMAGE_REQUEST);
              break;
            case PID_GS_VR_REQUEST_CAPTURE:
              printf("GS Requests to Capture\r\n");
              osEventFlagsSet(payloadFlagHandle,PAYLOAD_FLAG_POLL_CAPTURE);
              break;
            case PID_GS_VR_REQUEST_PI_STATUS:
              printf("GS Requests PI Status\r\n");
              break;
            case PID_GS_VR_REQUEST_PING:
              printf("GS Requests PI Ping\r\n");
              break;
            default:
              printf("GS PID that does not exists in the system\r\n");
              break;
            }
          }
          else if (commu_request_header.payload_id == COMMU_PAYLOAD_ID_OBC){
            switch (commu_request_header.pid){
              case PID_GS_OBC_REQUEST_PING:
                printf("GS Ping OBC\r\n");
                uint8_t response_ping_buf[32] = {0};
                uint8_t response_ping_len = commu_encode(0,COMMU_PAYLOAD_ID_OBC,PID_OBC_GS_RESPONSE_PING,0,NULL,response_ping_buf,10);

                uint8_t kiss_encoded_res[32] = {0};
                uint8_t kiss_respond_len = KISS_Encode_Custom_Cmd(response_ping_buf,KISS_CMD_DATA_FRAME,response_ping_len,kiss_encoded_res);

                HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(&COM_UART,kiss_encoded_res,kiss_respond_len);
                printf("Responsded ping from commu with return %d from UART\r\n",ret);
                break;
              case PID_GS_OBC_REQUEST_LIST_FILE:
                printf("GS Request List File\r\n");
                uint32_t currentPayloadFlag1 = osEventFlagsGet(payloadFlagHandle);
                if (currentPayloadFlag1 & (PAYLOAD_FLAG_IMAGE_REQUEST | PAYLOAD_FLAG_IMAGE_TRANSFER)){
                  printf("\033[0;31mNot now, payload file copying\033[0m\r\n");
                  break;
                }
                FATFS FatFs;
                FRESULT res;
                DIR dir;
                FILINFO fno;
                printf("\r\n");

                res = f_mount(&FatFs,"",1);
                if (res != FR_OK){
                  printf("Mount error\r\n");
                  break;
                }
                char files_name[20][256];
                uint8_t file_count = 0;
                // Open the root directory
                res = f_opendir(&dir, "/");
                if (res == FR_OK) {
                    for (;;) {
                        // Read a directory item
                        res = f_readdir(&dir, &fno);
                        if (res != FR_OK || fno.fname[0] == 0) break; // End of directory

                        // Check if it is a directory or file
                        if (fno.fattrib & AM_DIR) {
                            printf("Dir: %s\r\n", fno.fname);
                        } else {
                            printf("File: %s (Size: %u)\r\n", fno.fname, fno.fsize);
                            strcpy(files_name[file_count],fno.fname);
                            file_count++;
                        }
                    }
                    f_closedir(&dir);
                }
                res = f_mount(NULL,"",0);
                printf("\r\n");
                uint8_t list_file_buf[256] = {0};
                uint16_t list_file_encoded_len = commu_list_file_encode(files_name,file_count,list_file_buf);

                if (list_file_encoded_len == 0){
                  printf("Encode list file error\r\n");
                  break;
                }

                uint8_t commu_list_file_encoded[256] = {0};
                uint16_t commu_list_file_encoded_len = commu_encode(0,COMMU_PAYLOAD_ID_OBC,PID_OBC_GS_RESPONSE_LIST_FILE,list_file_encoded_len,list_file_buf,commu_list_file_encoded,512); 

                uint8_t kiss_list_file_encoded[256] = {0};
                uint16_t kiss_list_file_encoded_len = KISS_Encode_Custom_Cmd(commu_list_file_encoded,KISS_CMD_DATA_FRAME,commu_list_file_encoded_len,kiss_list_file_encoded);

                ret = HAL_UART_Transmit_IT(&COM_UART,kiss_list_file_encoded,kiss_list_file_encoded_len);
                printf("Responsded ping from commu with return %d from UART\r\n",ret);

                break;
              case PID_GS_OBC_REQUEST_FILE_INFO:
                printf("GS Request File info\r\n");
                uint32_t currentPayloadFlagForInfo = osEventFlagsGet(payloadFlagHandle);
                if (currentPayloadFlagForInfo & (PAYLOAD_FLAG_IMAGE_REQUEST | PAYLOAD_FLAG_IMAGE_TRANSFER)){
                  printf("\033[0;31mNot now, payload file copying\033[0m\r\n");
                  break;
                }
                commu_file_data requested_file_for_info;
                commu_status_t decode_status_for_info = decode_file_info_request(commu_payload,commu_request_header.data_len,&requested_file_for_info);
                if (decode_status_for_info != COMMU_VALID_DATA){
                  printf("Decoded return %d\r\n",decode_status_for_info);
                  break;
                }
                printf("\r\n");
                FATFS FatFs_info;
                FRESULT res_info;
                FIL file_info;
                FILINFO fno_info;
                FSIZE_t file_size_info;
                res_info = f_mount(&FatFs_info,"",1);
                if (res_info != FR_OK){
                  printf("Mount error\r\n");
                  break;
                }
                res_info = f_open(&file_info, requested_file_for_info.file_name, FA_READ);
                if (res_info == FR_OK) {
                    file_size_info = f_size(&file_info);
                    f_close(&file_info);
                }
                else{
                  printf("open to get info error %d\r\n",res_info);
                }
                res_info = f_stat(requested_file_for_info.file_name, &fno);
                uint32_t epoch_info = 0;
                if (res_info == FR_OK) {
                    // 2. Map FatFs bitfields to a standard tm struct
                    struct tm t;
                    
                    // FatFs Date: bit15:9=Year(0-127), bit8:5=Month(1-12), bit4:0=Day(1-31)
                    // FatFs Year 0 starts at 1980
                    t.tm_year = ((fno.fdate >> 9) & 0x7F) + 80; 
                    t.tm_mon  = ((fno.fdate >> 5) & 0x0F) - 1; 
                    t.tm_mday = (fno.fdate & 0x1F);

                    // FatFs Time: bit15:11=Hour(0-23), bit10:5=Minute(0-59), bit4:0=Second/2(0-29)
                    t.tm_hour = (fno.ftime >> 11) & 0x1F;
                    t.tm_min  = (fno.ftime >> 5) & 0x3F;
                    t.tm_sec  = (fno.ftime & 0x1F) * 2;
                    
                    t.tm_isdst = -1; // Not considering Daylight Savings

                    // 3. Convert to Unix Epoch
                    time_t epoch = mktime(&t);
                    epoch_info = (uint32_t)epoch;
                }
                f_mount(NULL,"",0);
                printf("request %s size, which is = %ld, epoch = %ld\r\n",requested_file_for_info.file_name,file_size_info,epoch_info);
                uint8_t buf_a_info[64];
                uint8_t buf_b_info[64];
                uint8_t buf_len_info = 0;

                buf_len_info = commu_file_info_encode(0,file_size_info,epoch_info,buf_a_info);
                buf_len_info = commu_encode(0,COMMU_PAYLOAD_ID_OBC,PID_OBC_GS_RESPONSE_FILE_INFO,buf_len_info,buf_a_info,buf_b_info,64);
                if (buf_len_info == 0){
                  printf("commu encode error file info\r\n");
                }
                buf_len_info = KISS_Encode_Custom_Cmd(buf_b_info,KISS_CMD_DATA_FRAME,buf_len_info,buf_a_info);
                if (buf_len_info == 0){
                  printf("kiss encode error file info\r\n");
                }
                ret = HAL_UART_Transmit(&COM_UART,buf_a_info,buf_len_info,1000);
                printf("Responsded ping from commu with return %d from UART\r\n",ret);
                break;
              case PID_GS_OBC_REQUEST_FILE_DATA:
                printf("GS Request File data\r\n");
                uint32_t currentPayloadFlag = osEventFlagsGet(payloadFlagHandle);
                if (currentPayloadFlag & (PAYLOAD_FLAG_IMAGE_REQUEST | PAYLOAD_FLAG_IMAGE_TRANSFER)){
                  printf("\033[0;31mNot now, payload file copying\033[0m\r\n");
                  break;
                }
                commu_file_data requested_file;
                commu_status_t decode_status = decode_file_data_request(commu_payload,commu_request_header.data_len,&requested_file);
                if (decode_status != COMMU_VALID_DATA){
                  printf("Decoded return %d\r\n",decode_status);
                  break;
                }
                downlink_file_data = requested_file;
                printf("file_name : %s chunk_len : %d, file_offset %ld\r\n",requested_file.file_name,requested_file.chunk_len,requested_file.file_offset);
                osEventFlagsClear(systemStateFlagHandle,SYSTEM_STATE_BEACON);
                osEventFlagsSet(systemStateFlagHandle,SYSTEM_STATE_DOWNLINK);
                last_commu_timeNow = millis;
                break;
              default:
                printf("Unknown OBC PID\r\n");
                break;
            }
          }
        }
      }
      osMutexRelease(uartMutexHandle);
    }

    uint32_t payloadFlag = osEventFlagsGet(payloadFlagHandle);
    if (payloadFlag & PAYLOAD_FLAG_IDLE){
      if (payloadFlag & PAYLOAD_FLAG_IMAGE_REQUEST){
        osEventFlagsClear(payloadFlagHandle, PAYLOAD_FLAG_IDLE);
        uint8_t cmd_encoded[32] = {0};
        uint8_t request_content[3] = {0x00,0xFF,0xFF};
        uint16_t req_len = KISS_WrapFrame(KISS_PAYLOAD_ID_VR,KISS_VR_PID_IMAGE_REQUEST,request_content,3,KISS_CMD_DATA, cmd_encoded);
        CDC_Transmit_FS(cmd_encoded,req_len);
        printf("KISS Frame Encoded : ");
        for (int i = 0; i < req_len;i++){
          printf("0x%02X ",cmd_encoded[i]);
        }
        printf("\r\n");
      }
      else if (payloadFlag & PAYLOAD_FLAG_POLL_CAPTURE){
        osEventFlagsClear(payloadFlagHandle, PAYLOAD_FLAG_IDLE);
        uint8_t cmd_encoded[32] = {0};
        // uint8_t request_content[3] = {0x00,0xFF,0xFF};
        uint16_t req_len = KISS_WrapFrame(KISS_PAYLOAD_ID_VR,KISS_VR_PID_IMAGE_REQUEST,NULL,0,KISS_CMD_DATA, cmd_encoded);
        CDC_Transmit_FS(cmd_encoded,req_len);
      }
    }

  }
  // printf("It exits main task\r\n");
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_usbTask */
uint8_t  temp_buf[40000];
uint8_t  payload_content[40000];
/**
 * @brief Function implementing the USBTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_usbTask */
void usbTask(void *argument)
{
  /* USER CODE BEGIN usbTask */
  /* Infinite loop */
    usb_data_t usb_data_rx = { .is_new_message = 0, .len = 0 };
    uint32_t current_offset = 0;
    // uint16_t payload_len = 0;
    uint16_t current_chunk = 0;

    kiss_frame_t decoded_payload = { .content = payload_content };

    // osEventFlagsSet(payloadFlagHandle, PAYLOAD_FLAG_IDLE);

    //some variables for FatFs
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations

    payload_state payload_commu_state = PAYLOAD_STATE_IDLE;

    for (;;)
    {

        osStatus_t status = osMessageQueueGet(cdcDataQueueHandle,
                                             (void *)&usb_data_rx,
                                             NULL, PAYLOAD_RX_TIMEOUT);

          // printf("Queue Trigger\r\n");
        if (status == osErrorTimeout){
          if (payload_commu_state == PAYLOAD_STATE_RX){
            printf("USB RX Timeout, Reset State...\r\n");
            osEventFlagsClear(payloadFlagHandle,PAYLOAD_FLAG_IMAGE_REQUEST | PAYLOAD_FLAG_IMAGE_TRANSFER);
            osEventFlagsSet(payloadFlagHandle,PAYLOAD_FLAG_IDLE);
            payload_commu_state = PAYLOAD_STATE_IDLE;
            current_offset = 0;
          }
          continue;
        }

        payload_commu_state = PAYLOAD_STATE_RX;
        /* --- Accumulate --- */
        if ((current_offset + usb_data_rx.len) > sizeof(temp_buf))
        {
            /* Overflow protection: discard and reset */
            printf("ERR: buffer overflow, resetting\r\n");
            current_offset = 0;
            continue;
        }

        memcpy(&temp_buf[current_offset], usb_data_rx.usb_buff, usb_data_rx.len);
        current_offset += usb_data_rx.len;
        uint8_t check = KISS_IsFrameComplete(temp_buf, current_offset);
        
        /* --- O(1) completion check, no CRC yet --- */
        if (!check)
        {
            /* Frame not done yet, wait for more chunks */
            continue;
        }

        /* --- Frame boundary found: NOW do the full decode + CRC --- */
        // printf("Frame complete at offset %lu, unwrapping...\r\n", current_offset);

        kiss_status_t result = KISS_UnwrapFrame(temp_buf, current_offset,
                                                payload_content, &decoded_payload);

        current_offset = 0; /* Reset accumulator regardless of outcome */

        if (result != KISS_VALID_DATA)
        {
            printf("ERR: UnwrapFrame failed: %d\r\n", result);
            continue;
        }

        /* --- Process valid frame --- */

        if (decoded_payload.payload_id == KISS_PAYLOAD_ID_VR)
        {
            if (decoded_payload.type == KISS_FRAME_TYPE_IMAGE)
            {
                // printf("chunk: %d\r\n", current_chunk++);
                uint8_t  reply_frame[32];
                uint16_t reply_len = KISS_WrapFrame(KISS_PAYLOAD_ID_VR, KISS_PID_ACK,
                                                    NULL, 0, 0x00, reply_frame);
                CDC_Transmit_FS(reply_frame, reply_len);
                // decoded_payload.file_id
                fres = f_open(&fil,"0.jpg", FA_OPEN_APPEND | FA_WRITE);
                if (fres != FR_OK) {

                    printf("open to append error (%i)\r\n", fres);
                    continue;
                }
                unsigned int written = 0;
                f_write(&fil,(void*)decoded_payload.content,decoded_payload.content_len,&written);
                if (fres != FR_OK) {
                    printf("content append error (%i)\r\n", fres);
                    continue;
                }
                // printf("written %d\r\n",written);
                f_close(&fil);
                continue;
                // if (decoded_payload.content_len )
            }
            if (decoded_payload.pid == KISS_PID_ACK)
            {
                uint32_t flag = osEventFlagsGet(payloadFlagHandle);
                printf("payload_event_flag: %08lX\r\n", flag);

                if (flag & PAYLOAD_FLAG_POLL_CAPTURE)
                {
                    printf("Payload acks Capture\r\n");
                    osEventFlagsClear(payloadFlagHandle,PAYLOAD_FLAG_POLL_CAPTURE);
                    osEventFlagsSet(payloadFlagHandle, PAYLOAD_FLAG_IDLE);
                    payload_commu_state = PAYLOAD_STATE_IDLE;
                }
                if (flag & PAYLOAD_FLAG_IMAGE_REQUEST)
                {
                    printf("Payload acks Image Request\r\n");
                    osEventFlagsClear(payloadFlagHandle, PAYLOAD_FLAG_IMAGE_REQUEST);
                    osEventFlagsSet(payloadFlagHandle, PAYLOAD_FLAG_IMAGE_TRANSFER);

                    current_chunk = 0;
                    /* Set your image transfer state flag here */
                        //Open the file system
                    fres = f_mount(&FatFs, "", 1); //1=mount now
                    if (fres != FR_OK) {
                      printf("f_mount error (%i)\r\n", fres);
                      continue;
                    }
                    fres = f_open(&fil,"0.jpg",FA_CREATE_ALWAYS);
                    if (fres != FR_OK) {
                      printf("create file error (%i)\r\n", fres);
                      continue;
                    }
                    f_close(&fil);
                    printf("\033[0;32mOBC Starts Copy image from Payload\033[0m\r\n");
                    // osEventFlagsClear(payloadFlagHandle, PAYLOAD_FLAG_IMAGE_REQUEST);
                }
            }
            else if (decoded_payload.pid == KISS_VR_PID_IMAGE_DOWNLOAD_DONE){
              printf("\033[0;32mTransfer Image Done\033[0m\r\n");
              payload_commu_state = PAYLOAD_STATE_IDLE;
              osEventFlagsClear(payloadFlagHandle, PAYLOAD_FLAG_IMAGE_REQUEST | PAYLOAD_FLAG_IMAGE_TRANSFER);
              osEventFlagsSet(payloadFlagHandle, PAYLOAD_FLAG_IDLE);
              fres = f_mount(NULL, "", 0);
              if (fres != FR_OK) {
                printf("unmount error (%i)\r\n", fres);
                continue;
              }
            }
        }
    }
  /* USER CODE END usbTask */
}

/* USER CODE BEGIN Header_wdtFeedTask */
/**
//  * @brief Function implementing the wdtFeed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_wdtFeedTask */
void wdtFeedTask(void *argument)
{
  /* USER CODE BEGIN wdtFeedTask */
  /* Infinite loop */
  for (;;)
  {
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      printf("IWDT Error\r\n");
    }
    // printf("IWDG Still triggering\r\n");
    osDelay(400);
  }
  /* USER CODE END wdtFeedTask */
}

/* USER CODE BEGIN Header_sensorQueryTask */
/**
 * @brief Function implementing the sensorQuery thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sensorQueryTask */
void sensorQueryTask(void *argument)
{
  /* USER CODE BEGIN sensorQueryTask */

  eps_sensor_data_t _eps_sensors;

  tmp1075_t temp_sen = {
    .tmp1075_i2c_hal.hi2c = &hi2c4,
    .address = 0x48,
  };

 
  tmp1075_init(&temp_sen);

  for (;;)
  {
    // osEventFlagsWait(epsFlagHandle,EPS_FLAG_POLL_START,osFlagsWaitAny,osWaitForever);
    int32_t temp = 0;
    hal_status_t ret = tmp1075_read_temp(&temp_sen, &temp);
    if (ret != hal_ok)
    {
      printf("Read Temperature Error");
    }
    
    
    /* Perform Battery Temperatures Reads for 6 Channel */
    // for (int i = 0; i < EPS_NUM_TEMP_BATT;i++){
    eps_command_t cmd;
    uint8_t eps_data[EPS_BUF_SIZE] = {0};
    eps_get_all_kiss_command(&cmd);
    uint16_t decoded_len = EPS_Perform_Transaction(cmd.cmd,cmd.len,eps_data);

    uint16_t offset = 0;
    if (decoded_len != 0){
      osStatus_t os_ret = osMutexAcquire(sensorsMutexHandle,500);
      if (os_ret == osOK){
        eps_state = EPS_DATA_OK;
        osMutexRelease(sensorsMutexHandle);
      }
      // 1. Parse VI Sensors (8 Channels * 5 bytes = 40 bytes)
      for (uint8_t i = 0; i < EPS_NUM_VI_CHANNEL; i++) {
          if (offset + 5 > decoded_len) break; // Safety bounds check
          
          memcpy(&_eps_sensors.vi_sensor[i].voltage, &eps_data[offset], 2);
          offset += 2;
          memcpy(&_eps_sensors.vi_sensor[i].current, &eps_data[offset], 2);
          offset += 2;
          
          _eps_sensors.vi_sensor[i].channel = eps_data[offset++];
          _eps_sensors.vi_sensor[i].data_state = EPS_DATA_OK;
      }
  
      // 2. Parse Output Sensors (6 Channels * 5 bytes = 30 bytes)
      for (uint8_t i = 0; i < EPS_NUM_OUTPUT_CHANNEL; i++) {
          if (offset + 5 > decoded_len) break;
          
          memcpy(&_eps_sensors.output_sensor[i].voltage, &eps_data[offset], 2);
          offset += 2;
          memcpy(&_eps_sensors.output_sensor[i].current, &eps_data[offset], 2);
          offset += 2;
          
          _eps_sensors.output_sensor[i].channel = eps_data[offset++];
          _eps_sensors.output_sensor[i].data_state = EPS_DATA_OK;
      }
  
      // 3. Parse Output States (6 Channels * 2 bytes = 12 bytes)
      for (uint8_t i = 0; i < EPS_NUM_OUTPUT_CHANNEL; i++) {
          if (offset + 2 > decoded_len) break;
          
          _eps_sensors.output_state[i].status = eps_data[offset++];
          _eps_sensors.output_state[i].channel = eps_data[offset++];
          _eps_sensors.output_state[i].data_state = EPS_DATA_OK;
      }
  
      // 4. Parse Battery Temps (2 Channels * 5 bytes = 10 bytes)
      for (uint8_t i = 0; i < EPS_NUM_TEMP_BATT; i++) {
          if (offset + 5 > decoded_len) break;
          
          memcpy(&_eps_sensors.battery_temperature[i].temperature, &eps_data[offset], 4);
          offset += 4;
          
          _eps_sensors.battery_temperature[i].channel = eps_data[offset++];
          _eps_sensors.battery_temperature[i].data_state = EPS_DATA_OK;
      }
    }
    
    osStatus_t os_ret = osMutexAcquire(sensorsMutexHandle,500);
    if (os_ret == osOK){
      obc_sensors_data.temp = temp;
      // obc_sensors_data.datetime = datetime;
      eps_sensors_data = _eps_sensors;
      osMutexRelease(sensorsMutexHandle);
    }
    else{
      printf("Aquire OBC for Sending Sensor error\r\n");
    }

    osDelay(1000);

  }
  /* USER CODE END sensorQueryTask */
}

/* USER CODE BEGIN Header_logTask */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_logTask */
void logTask(void *argument)
{
  /* USER CODE BEGIN logTask */
  uint8_t ch = 0;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(printQueueHandle,&ch,NULL,osWaitForever);
    // HAL_UART_Transmit_IT(&huart5, (uint8_t *)&ch, 1);
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);
    // osDelay(1);
  }
  /* USER CODE END logTask */
}

/* USER CODE BEGIN Header_uartRx */
/**
* @brief Function implementing the uartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartRx */
void uartRx(void *argument)
{
  /* USER CODE BEGIN uartRx */
    uint16_t chunk_len = 0;
    /* Arm the very first receive — do it here, not in mainTask */
    HAL_UARTEx_ReceiveToIdle_IT(&COM_UART, commu_temp_buff, COMMU_RX_SIZE);
  /* Infinite loop */

    commu_state communication_state = COMMU_RX_IDLE;
    
    for(;;)
    {
        /* Block until the idle-line ISR signals a chunk arrived */
        osStatus_t status = osMessageQueueGet(communicationUartQueueHandle,
                                              &chunk_len, NULL, COMMU_RX_TIMEOUT);
        // if (status != osOK) continue;
        if (status == osErrorTimeout){
          if (communication_state == COMMU_RX_ONGOING){
            printf("Hanging RX Buffer for too long, Resetting\r\n");
            communication_state = COMMU_RX_IDLE;
            commu_offset = 0;
          }
          continue;
        }

        communication_state = COMMU_RX_ONGOING;

        // else if (status == osOK)
        /* --- Re-arm RX immediately so we don't miss the next bytes --- */
        HAL_UARTEx_ReceiveToIdle_IT(&COM_UART, commu_temp_buff, COMMU_RX_SIZE);

        /* --- Overflow guard --- */
        if ((commu_offset + chunk_len) > COMMU_BUF_SIZE)
        {
            printf("COMMU: buffer overflow, resetting\r\n");
            commu_offset = 0;
            continue;
        }

        /* --- Append chunk to accumulation buffer --- */
        memcpy(&commu_data_buff[commu_offset], commu_temp_buff, chunk_len);
        commu_offset += chunk_len;
        
        /* --- Debug print of what we have so far --- */
        printf("COMMU RX [%d bytes total]: ", commu_offset);
        for (int i = 0; i < commu_offset; i++) {
          printf("%02X ", commu_data_buff[i]);
        }
        printf("\r\n");
        
        /* --- Check if we have a complete KISS frame --- */
        if (!KISS_IsFrameComplete(commu_data_buff, commu_offset))
        {
            printf("COMMU: frame incomplete, waiting for more...\r\n");
            continue;
          }
          
          /* --- Complete frame: hand it off --- */
          printf("COMMU: complete KISS frame (%d bytes)\r\n", commu_offset);
          osMutexAcquire(uartMutexHandle,UART_MUTEX_TIMEOUT);
          for (int i = 0; i < commu_offset;i++){
            commu_global_buff[i] = commu_data_buff[i];
          }
          commu_size = commu_offset;
          commu_data_ready = 1;
          osMutexRelease(uartMutexHandle);  
          communication_state = COMMU_RX_IDLE;
          
        /* 
         * TODO: process commu_data_buff / commu_offset here.
         * e.g. KISS_UnwrapFrame(commu_data_buff, commu_offset, ...)
         * or copy into a queue for another task.
         */

        /* Reset accumulator for the next frame */
        commu_offset = 0;
  }
  /* USER CODE END uartRx */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
