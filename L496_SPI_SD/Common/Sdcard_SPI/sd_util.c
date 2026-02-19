#include "main.h"
#include "sd_utils.h"
#include "usbd_cdc_if.h"
#include "fatfs.h"
#include "obc_packet.h"
#include <stdio.h>
#include <string.h>

extern char USERPath[4]; /* USERPath defined in fatfs.c */
extern FATFS USERFatFS; /* USERFatFS defined in fatfs.c */
extern UART_HandleTypeDef hlpuart1;
extern CRC_HandleTypeDef hcrc;

// Externs from usbd_cdc_if.c
extern uint8_t UserRxBufferFS_A[];
extern uint8_t UserRxBufferFS_B[];
extern volatile uint8_t Buffer_A_Ready;
extern volatile uint8_t Buffer_B_Ready;
extern volatile uint32_t Buffer_A_Length;
extern volatile uint32_t Buffer_B_Length;
extern void CDC_Buffer_Processed(uint8_t buffer_id);

// Private variables for save_sd
static uint32_t totalBytesReceived = 0;
static uint32_t lastByteTime = 0;
static uint32_t lastHeartbeat = 0;
static uint8_t receiveState = 0; // 0: Waiting, 1: Receiving
static char currentFilename[64] = {0};
static uint32_t startTime = 0;
static uint32_t transferCount = 0;
static FIL fil;

void SD_SaveFiles(void) {
    FRESULT fres;

    // Initialize lastByteTime on first run or use
    if (lastByteTime == 0) lastByteTime = HAL_GetTick();

    // --- Heartbeat (Red LED) ---
    if ((HAL_GetTick() - lastHeartbeat) > 500) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        lastHeartbeat = HAL_GetTick();
    }

    // check exit condition (Timeout while receiving)
    if (receiveState == 1 && (HAL_GetTick() - lastByteTime) > 3000) {
        printf("\r\nEnd of transmission (Timeout). Closing file.\r\n");
        f_close(&fil);

        // Increment Iteration
        transferCount++;

        // Calc Speed
        uint32_t endTime = HAL_GetTick();
        uint32_t duration_ms = endTime - startTime;
        if (duration_ms == 0) duration_ms = 1;
        uint64_t speed_calc = ((uint64_t)totalBytesReceived * 1000 * 100) / ((uint64_t)1024 * duration_ms);
        printf("Iter: %lu, Duration: %lu ms\r\n", transferCount, duration_ms);
        printf("Received: %lu bytes\r\n", totalBytesReceived);
        printf("Speed: %lu.%02lu KB/s\r\n", (uint32_t)(speed_calc/100), (uint32_t)(speed_calc%100));

        // --- Save Log to SD Card ---
        FIL logFile;
        // Adjust filename for log if needed, or keeping "log.txt"
        if(f_open(&logFile, "log.txt", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
            char logBuf[128];
            int len = snprintf(logBuf, sizeof(logBuf), "Iter:%lu, Tick:%lu, File:%s, Size:%lu B, Time:%lu ms, Speed:%lu.%02lu KB/s\r\n",
                               transferCount, HAL_GetTick(), currentFilename, totalBytesReceived, duration_ms, (uint32_t)(speed_calc/100), (uint32_t)(speed_calc%100));

            UINT bw_log;
            f_write(&logFile, logBuf, len, &bw_log);
            f_close(&logFile);
            printf("Log saved to log.txt\r\n");
        } else {
            printf("Failed to open log.txt\r\n");
        }

        // Reset State for next transfer
        receiveState = 0;
        totalBytesReceived = 0;
        memset(currentFilename, 0, sizeof(currentFilename));
        printf("\r\nReady for next command...\r\n");
    }

    // Process Buffer A
    if (Buffer_A_Ready) {
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

                // Retry logic for robust long-running
                if (fres != FR_OK) {
                    printf("File Open Failed (%d). Attempting Remount...\r\n", fres);
                    f_mount(NULL, USERPath, 0); // Unmount
                    HAL_Delay(50);
                    // Using USERFatFS externed from fatfs.c
                    fres = f_mount(&USERFatFS, USERPath, 1); // Remount
                    if (fres == FR_OK) {
                         fres = f_open(&fil, currentFilename, FA_WRITE | FA_CREATE_ALWAYS);
                    } else {
                         printf("Remount failed (%d)\r\n", fres);
                    }
                }

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
    if (Buffer_B_Ready) {
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

                // Retry logic for robust long-running
                if (fres != FR_OK) {
                    printf("File Open Failed (%d). Attempting Remount...\r\n", fres);
                    f_mount(NULL, USERPath, 0); // Unmount
                    HAL_Delay(50);
                    fres = f_mount(&USERFatFS, USERPath, 1); // Remount
                    if (fres == FR_OK) {
                         fres = f_open(&fil, currentFilename, FA_WRITE | FA_CREATE_ALWAYS);
                    } else {
                         printf("Remount failed (%d)\r\n", fres);
                    }
                }

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
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

extern char USERPath[4]; /* USERPath defined in fatfs.c */
extern FATFS USERFatFS; /* USERFatFS defined in fatfs.c */
extern UART_HandleTypeDef hlpuart1;
extern CRC_HandleTypeDef hcrc;

static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void Base64_Encode(uint8_t *input, uint16_t input_len, char *output) {
    int i = 0, j = 0;
    uint8_t byte3[3];
    uint8_t byte4[4];

    while (input_len--) {
        byte3[i++] = *(input++);
        if (i == 3) {
            byte4[0] = (byte3[0] & 0xfc) >> 2;
            byte4[1] = ((byte3[0] & 0x03) << 4) + ((byte3[1] & 0xf0) >> 4);
            byte4[2] = ((byte3[1] & 0x0f) << 2) + ((byte3[2] & 0xc0) >> 6);
            byte4[3] = byte3[2] & 0x3f;

            for(i = 0; i < 4; i++) output[j++] = base64_chars[byte4[i]];
            i = 0;
        }
    }

    if (i) {
        for(int k = i; k < 3; k++) byte3[k] = '\0';
        byte4[0] = (byte3[0] & 0xfc) >> 2;
        byte4[1] = ((byte3[0] & 0x03) << 4) + ((byte3[1] & 0xf0) >> 4);
        byte4[2] = ((byte3[1] & 0x0f) << 2) + ((byte3[2] & 0xc0) >> 6);
        byte4[3] = byte3[2] & 0x3f;

        for(int k = 0; k < i + 1; k++) output[j++] = base64_chars[byte4[k]];
        while((i++ < 3)) output[j++] = '=';
    }
    output[j] = '\0';
}

void SD_Init(void) {
    FRESULT res;
    // Mount SD Card
    res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) {
        // Mount Error - Blink Red LED once to indicate failure but continue
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(100); 
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        printf("SD Init Mount Failed: %d. Will retry in main loop.\r\n", res);
    }
}

void SD_SendFile(char *filename, uint16_t sector) {
    FRESULT res;
    FIL file;
    uint8_t file_buffer[64]; // Data chunk size as requested
    uint8_t packet_buffer[100]; // Buffer for KISS payload (1(CMD) + 4(ID) + 12(Name) + 64(Data) + 4(CRC))
    uint8_t encoded_buffer[256]; // Buffer for SLIP encoded frame (max overhead ~2x)

    UINT bytesRead;
    uint32_t chunk_id = 0;
    
    // Ensure packet module is initialized with CRC handle
    OBC_Packet_Init(&hcrc);

    if (f_open(&file, filename, FA_READ) == FR_OK) {
        // LED Status: PB7 (Active) ON, PB14 (Sending) OFF initially
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        
        __HAL_CRC_DR_RESET(&hcrc);
        
        if (sector == 0xFFFF) {
            // Send Whole File
            uint32_t filesize = f_size(&file);
            // We transmit in 64 byte chunks
            
            while (1) {
                // 1. Read 64 bytes
                res = f_read(&file, file_buffer, 64, &bytesRead);
                if (res != FR_OK || bytesRead == 0) break;

                // ... (packet construction) ...
                uint8_t idx = 0;
                packet_buffer[idx++] = 0x00; // KISS Command Byte (Data Chunk)

                packet_buffer[idx++] = 0x13; // APP CMD (Picture)
                memcpy(&packet_buffer[idx], &chunk_id, 4);
                idx += 4;

                // Filename (12 bytes, padded)
                memset(&packet_buffer[idx], 0, 12);
                strncpy((char*)&packet_buffer[idx], filename, 12);
                idx += 12;

                // Data (64 bytes, padded if last chunk)
                memcpy(&packet_buffer[idx], file_buffer, bytesRead);
                if (bytesRead < 64) {
                    memset(&packet_buffer[idx + bytesRead], 0, 64 - bytesRead);
                }
                idx += 64;

                // Calculate CRC (includes CMD, ID, Name, Data)
                uint32_t crc = OBC_Calculate_CRC(packet_buffer, idx);
                memcpy(&packet_buffer[idx], &crc, 4);
                idx += 4;

                // 3. Encode SLIP
                uint16_t encoded_len = SLIP_Encode(packet_buffer, idx, encoded_buffer);

                // 4. Send
                HAL_UART_Transmit(&hlpuart1, encoded_buffer, encoded_len, 1000);
                HAL_Delay(15);
                
                // LED Status: Toggle PB14 to show activity/speed
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
                
                chunk_id++;
            }
            
        } else {
            // Send Single 64-byte Chunk
            
            f_lseek(&file, (uint32_t)sector * 64);
            res = f_read(&file, file_buffer, 64, &bytesRead);
            
            if (res == FR_OK && bytesRead > 0) {
                 // Packetize 
                uint8_t idx = 0;
                packet_buffer[idx++] = 0x00; // KISS Command Byte
                packet_buffer[idx++] = 0x13; // APP Command (Image Chunk)
                
                // ID is the chunk index
                uint32_t chunk_id = sector;
                memcpy(&packet_buffer[idx], &chunk_id, 4);
                idx += 4;

                memset(&packet_buffer[idx], 0, 12);
                strncpy((char*)&packet_buffer[idx], filename, 12);
                idx += 12;

                memcpy(&packet_buffer[idx], file_buffer, bytesRead);
                 if (bytesRead < 64) memset(&packet_buffer[idx + bytesRead], 0, 64 - bytesRead);
                idx += 64;

                uint32_t crc = OBC_Calculate_CRC(packet_buffer, idx);
                memcpy(&packet_buffer[idx], &crc, 4);
                idx += 4;

                uint16_t encoded_len = SLIP_Encode(packet_buffer, idx, encoded_buffer);
                HAL_UART_Transmit(&hlpuart1, encoded_buffer, encoded_len, 1000);
                
                // Toggle PB14
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
            }
        }
        
        f_close(&file);

        // Turn LEDs OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        // File Open Error
        while(1) {
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            HAL_Delay(1000);
        }
    }
}
