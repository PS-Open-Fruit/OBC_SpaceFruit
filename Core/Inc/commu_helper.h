#include "main.h"

/* In your global/header section — replace the old commu vars */
#define COMMU_RX_SIZE     64
#define COMMU_BUF_SIZE   256   // accumulation buffer, large enough for multi-chunk frames
#define UART_MUTEX_TIMEOUT 1000

typedef enum{
      COMMU_RX_IDLE,
      COMMU_RX_ONGOING,
} commu_state;
static uint8_t commu_temp_buff[COMMU_RX_SIZE];   // DMA landing buffer (single chunk)
static uint8_t commu_data_buff[COMMU_BUF_SIZE];  // accumulation buffer
static uint16_t commu_offset = 0;                // how many bytes accumulated so far
static uint8_t commu_global_buff[COMMU_RX_SIZE];   // DMA landing buffer (single chunk)
uint16_t commu_size = 0;
uint8_t commu_data_ready = 0;

osMessageQueueId_t communicationUartQueueHandle;
const osMessageQueueAttr_t communicationUartQueue_attributes = {
  .name = "communicationUartQueue"
};

osSemaphoreId_t commuSemaphoreHandle;
const osSemaphoreAttr_t commuSemaphoreAttr = {
  .name = "commuSemaphore"
};

osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

osMutexId_t systemStateMutexHandle;
const osMutexAttr_t systemStateMutex_attributes = {
  .name = "systemStateMutex"
};

void commu_init(){
  commuSemaphoreHandle = osSemaphoreNew(1,0,&commuSemaphoreAttr);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  systemStateMutexHandle = osMutexNew(&systemStateMutex_attributes);
  communicationUartQueueHandle = osMessageQueueNew(16,sizeof(uint16_t),&communicationUartQueue_attributes);
}