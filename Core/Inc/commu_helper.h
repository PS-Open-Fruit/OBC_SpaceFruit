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
  // PID_GS_VR_REQUEST_SYSTEM_STATUS,
  PID_GS_VR_REQUEST_SHUTDOWN = 0x90,
  PID_GS_VR_ACK = 0xAC,
  PID_GS_VR_NAK = 0xAD,
} commu_payload_vr_pid;

typedef enum {
  PID_VR_GS_RESPONSE_PING,
  PID_VR_GS_RESPONSE_PI_STATUS,
  PID_VR_GS_RESPONSE_CAPTURE,
  PID_VR_GS_RESPONSE_COPY_IMAGE_TO_SD,
  PID_VR_GS_RESPONSE_SYSTEM_STATUS,
  PID_VR_GS_ACK = 0xAC,
  PID_VR_GS_NAK = 0xAD,
} payload_commu_vr_pid;

typedef enum {
  PID_OBC_GS_RESPONSE_PING,
  PID_OBC_GS_RESPONSE_LIST_FILE,
  PID_OBC_GS_RESPONSE_FILE_INFO,
  PID_OBC_GS_RESPONSE_FILE_DATA,
  PID_OBC_GS_BEACON,
  PID_OBC_GS_ACK = 0xAC,
  PID_OBC_GS_NAK = 0xAD,
} obc_downlink_pid;

typedef enum {
  PID_GS_OBC_REQUEST_PING,
  PID_GS_OBC_REQUEST_LIST_FILE,
  PID_GS_OBC_REQUEST_FILE_INFO,
  PID_GS_OBC_REQUEST_FILE_DATA,
  PID_GS_OBC_04_UNUSED,
  PID_GS_OBC_REQUEST_SYSTEM_STATUS,
  PID_GS_OBC_ACK = 0xAC,
  PID_GS_OBC_NAK = 0xAD,
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
    COMMU_ERR_CRC    = -5,  // Added for CRC validation
    COMMU_ERR_DATA = -6,
} commu_status_t;

typedef struct {
    uint8_t seq_num; 
    uint8_t payload_id; 
    uint8_t pid; 
    uint16_t data_len;
} commu_header_t;

typedef struct{
  char file_name[256];
  uint32_t file_offset;
  uint16_t chunk_len;
} commu_file_data;

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

commu_status_t commu_decode(const uint8_t *input_buf, uint16_t input_len,commu_header_t *header,uint8_t *output_payload){
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
    header->data_len = (input_buf[3] << 8) | (input_buf[4]);
    if (output_payload != NULL && header->data_len >= 0){
      /* -4 for crc -5 for headers*/
      memcpy(output_payload,&input_buf[5],header->data_len);
    }
    return COMMU_VALID_DATA;
}

commu_status_t decode_file_info_request(const uint8_t *input_data,uint8_t input_len,commu_file_data *file_data){
  uint8_t filename_len = input_data[0];
  if (filename_len <= 0){
    return COMMU_ERR_DATA;
  }
  /* 6 = file_offset (4) + chunk_len(2) */
  if (input_len <=  filename_len){
    return COMMU_ERR_DATA;
  }

  strncpy(file_data->file_name,&input_data[1],filename_len);
  if (filename_len < 255){
    file_data->file_name[filename_len] = '\0'; /* NULL String at the end of file name */
  }
  else{
    file_data->file_name[255] = '\0'; /* NULL String at the end of file name */
  }
  file_data->file_offset = 0;
  file_data->chunk_len = 0;
  return COMMU_VALID_DATA;
}

commu_status_t decode_file_data_request(const uint8_t *input_data,uint8_t input_len,commu_file_data *file_data){
  uint8_t filename_len = input_data[0];
  if (filename_len <= 0){
    return COMMU_ERR_DATA;
  }
  /* 6 = file_offset (4) + chunk_len(2) */
  if (input_len <= 6 + filename_len){
    return COMMU_ERR_DATA;
  }

  strncpy(file_data->file_name,&input_data[1],filename_len);
  if (filename_len < 255){
    file_data->file_name[filename_len] = '\0'; /* NULL String at the end of file name */
  }
  else{
    file_data->file_name[255] = '\0'; /* NULL String at the end of file name */
  }
  uint8_t *leftover = &input_data[filename_len + 1]; /* filename_len + the file name string offset */

  file_data->file_offset =  (leftover[0] << 24) |
                            (leftover[1] << 16) |
                            (leftover[2] << 8) |
                            (leftover[3]);
  file_data->chunk_len =  (leftover[4] << 8) |
                          (leftover[5]);
  return COMMU_VALID_DATA;
}

uint16_t commu_list_file_encode(char files[20][256],uint8_t files_count,uint8_t *output_buffer){
  uint16_t output_len = 0;
  printf("file count %d\r\n",files_count);
  if (files_count <= 0){
    return 0;
  }
  if (output_buffer == NULL){
    return 0;
  }
  output_buffer[output_len] = files_count;
  output_len++;
  for (int i = 0;i<files_count;i++){
    uint16_t file_len = strlen(files[i]);
    output_buffer[output_len] = file_len;
    output_len++;
    strncpy(&output_buffer[output_len],files[i],file_len);
    output_len += file_len;
  }
  return output_len;
}

uint16_t commu_file_info_encode(uint8_t status, uint32_t file_size, uint32_t created_time,uint8_t *output_buffer){
  uint16_t output_len = 0;
  output_buffer[output_len] = status;
  output_len++;
  output_buffer[output_len] = (file_size >> 24) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_size >> 16) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_size >> 8) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_size >> 0) & 0xFF;
  output_len++;
  output_buffer[output_len] = (created_time >> 24) & 0xFF;
  output_len++;
  output_buffer[output_len] = (created_time >> 16) & 0xFF;
  output_len++;
  output_buffer[output_len] = (created_time >> 8) & 0xFF;
  output_len++;
  output_buffer[output_len] = (created_time >> 0) & 0xFF;
  output_len++;
  return output_len;
}

uint16_t commu_file_downlink_encode(commu_file_data file_data,uint8_t status, uint8_t *content, uint16_t input_len,uint8_t *output_buffer){
  uint16_t output_len = 0;
  if (content == NULL){
    return 0;
  }
  output_buffer[output_len] = status;
  output_len++;
  output_buffer[output_len] = (file_data.file_offset >> 24) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_data.file_offset >> 16) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_data.file_offset >> 8) & 0xFF;
  output_len++;
  output_buffer[output_len] = (file_data.file_offset >> 0) & 0xFF;
  output_len++;
  output_buffer[output_len] = (input_len >> 8) & 0xFF;
  output_len++;
  output_buffer[output_len] = (input_len >> 0) & 0xFF;
  output_len++;
  memcpy(&output_buffer[output_len],content,input_len);
  output_len += input_len;
  return output_len;
}

uint16_t commu_system_status_raw_downlink_encode(uint32_t obc_boot_count,uint8_t usb_status, uint8_t eps_status, uint8_t *raw_payload_status, uint16_t input_len,uint8_t *output_buffer){
  uint16_t output_len = 0;
  if (raw_payload_status == NULL || input_len == 0){
    return 0;
  }
  output_buffer[output_len] = (obc_boot_count >> 24) & 0xFF;
  output_len++;
  output_buffer[output_len] = (obc_boot_count >> 16) & 0xFF;
  output_len++;
  output_buffer[output_len] = (obc_boot_count >> 8) & 0xFF;
  output_len++;
  output_buffer[output_len] = (obc_boot_count >> 0) & 0xFF;
  output_len++;
  output_buffer[output_len] = usb_status;
  output_len++;
  output_buffer[output_len] = eps_status;
  output_len++;
  memcpy(&output_buffer[output_len],raw_payload_status,input_len);
  return output_len + input_len;
}


void commu_init(){
  commuSemaphoreHandle = osSemaphoreNew(1,0,&commuSemaphoreAttr);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  communicationUartQueueHandle = osMessageQueueNew(16,sizeof(uint16_t),&communicationUartQueue_attributes);
}