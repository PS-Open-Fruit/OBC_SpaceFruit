#include "main.h"
#include "protocol_utils.h"
#include "string.h"
/* In your global/header section — replace the old commu vars */
#define COMMU_RX_SIZE     64
#define COMMU_BUF_SIZE   256   // accumulation buffer, large enough for multi-chunk frames
#define UART_MUTEX_TIMEOUT 1000
#define BASIC_COMMU_FRAME_LEN 9

typedef enum {
  COMMU_PAYLOAD_ID_OBC,
  COMMU_PAYLOAD_ID_VR,
} commu_payload_payload_id;

typedef enum {
  PID_GS_VR_REQUEST_PING,
  PID_GS_VR_REQUEST_PI_STATUS,
  PID_GS_VR_REQUEST_CAPTURE,
  PID_GS_VR_REQUEST_COPY_IMAGE_TO_SD,
} commu_payload_vr_pid;

typedef enum {
  PID_OBC_GS_RESPONSE_PING,
  PID_OBC_GS_RESPONSE_LIST_FILE,
  PID_OBC_GS_RESPONSE_FILE_INFO,
  PID_OBC_GS_RESPONSE_FILE_DATA,
  PID_OBC_GS_BEACON,
} obc_downlink_pid;

typedef enum {
  PID_GS_OBC_REQUEST_PING,
  PID_GS_OBC_REQUEST_LIST_FILE,
  PID_GS_OBC_REQUEST_FILE_INFO,
  PID_GS_OBC_REQUEST_FILE_DATA,
} gs_uplink_pid;

typedef enum{
      COMMU_RX_IDLE,
      COMMU_RX_ONGOING,
} commu_state;

typedef enum {
    COMMU_VALID_DATA = 0,
    COMMU_VALID_CMD  = 1,
    COMMU_ERR_EMPTY  = -1,
    COMMU_ERR_PORT   = -2,
    COMMU_ERR_TYPE   = -3,
    COMMU_ERR_FRAMING = -4,
    COMMU_ERR_CRC    = -5  // Added for CRC validation
} commu_status_t;

typedef struct {
    uint8_t seq_num; 
    uint8_t payload_id; 
    uint8_t pid; 
} commu_header_t;

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

uint16_t commu_encode(uint8_t seq_num, uint8_t payload_id, uint8_t pid, 
                      uint16_t input_len, const uint8_t *input_buffer, 
                     uint8_t *output_buffer, uint16_t max_output_len) {
    
    if (5 + input_len + 4 > max_output_len) {
        return 0;
    }

    uint16_t index = 0;
    output_buffer[index++] = seq_num;
    output_buffer[index++] = payload_id;
    output_buffer[index++] = pid;
    
    output_buffer[index++] = (uint8_t)(input_len >> 8);
    output_buffer[index++] = (uint8_t)(input_len & 0xFF);

    if (input_buffer != NULL && input_len > 0) {
        memcpy(&output_buffer[index], input_buffer, input_len);
        index += input_len;
    }

    uint32_t crc = KISS_CalculateCRC32(output_buffer, index);
    
    output_buffer[index++] = (uint8_t)(crc >> 24);
    output_buffer[index++] = (uint8_t)(crc >> 16);
    output_buffer[index++] = (uint8_t)(crc >> 8);
    output_buffer[index++] = (uint8_t)(crc & 0xFF);

    return index;
}

commu_status_t commu_decode_get_header(uint8_t *input_buf, uint16_t input_len,commu_header_t *header){
    /* seq (1) + payload_id (1) + pid (1) + dataLen (2) + crc (4)*/
    if (input_len < BASIC_COMMU_FRAME_LEN){
      return COMMU_ERR_FRAMING;
    }
    uint32_t crc = KISS_CalculateCRC32(input_buf,input_len - 4U);
    uint32_t crc_in = (((uint32_t)input_buf[input_len - 4] & 0xFF) << 24) |
                      (((uint32_t)input_buf[input_len - 3] & 0xFF) << 16) |
                      (((uint32_t)input_buf[input_len - 2] & 0xFF) << 8)  |
                        (uint32_t)input_buf[input_len - 1] & 0xFF;

    if (crc != crc_in){
      return COMMU_ERR_CRC;
    }
    header->seq_num = input_buf[0];
    header->payload_id = input_buf[1];
    header->pid = input_buf[2];
    return COMMU_VALID_DATA;
}

void commu_init(){
  commuSemaphoreHandle = osSemaphoreNew(1,0,&commuSemaphoreAttr);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  communicationUartQueueHandle = osMessageQueueNew(16,sizeof(uint16_t),&communicationUartQueue_attributes);
}