#include "sd_save.h"
#include "fatfs.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

// Externs from usbd_cdc_if.c
extern uint8_t UserRxBufferFS_A[];
extern uint8_t UserRxBufferFS_B[];
extern volatile uint8_t Buffer_A_Ready;
extern volatile uint8_t Buffer_B_Ready;
extern volatile uint32_t Buffer_A_Length;
extern volatile uint32_t Buffer_B_Length;
extern void CDC_Buffer_Processed(uint8_t buffer_id);

// Externs from fatfs.c
extern FATFS USERFatFS;

// Private variables
static uint32_t totalBytesReceived = 0;
static uint32_t lastByteTime = 0;
static uint32_t lastHeartbeat = 0;
static uint8_t receiveState = 0; // 0: Waiting, 1: Receiving
static char currentFilename[64] = {0};
static uint32_t startTime = 0;
static uint32_t transferCount = 0;
static FIL fil;

void save_sd(void) {
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
