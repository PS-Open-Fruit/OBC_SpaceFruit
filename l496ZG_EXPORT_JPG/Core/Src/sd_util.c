#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

extern char USERPath[4]; /* USERPath defined in fatfs.c */
extern FATFS USERFatFS; /* USERFatFS defined in fatfs.c */
extern UART_HandleTypeDef hlpuart1;

static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void Base64_Encode(uint8_t *input, uint8_t input_len, char *output) {
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

void SD_ListFiles(void) {
    FRESULT res;
    DIR dir;
    FILINFO fno;

    printf("\r\n--- SD Card Files ---\r\n");

    res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) {
        printf("SD Mount Error: %d\r\n", res);
        return;
    }

    res = f_opendir(&dir, USERPath);
    if (res == FR_OK) {
        while (1) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_DIR) {
                printf("[DIR]  %s\r\n", fno.fname);
            } else {
                printf("%-20s %lu bytes\r\n", fno.fname, fno.fsize);
            }
        }
        f_closedir(&dir);
    }
    printf("---------------------\r\n");
}

void SD_SendFile(char *filename) {
    FRESULT res;
    FIL file;
    uint8_t buffer[48]; // Multiple of 3 for Base64 (48 bytes -> 64 chars)
    char encoded_buffer[68]; // 48 * 4/3 = 64 + null
    char msg[128];
    UINT bytesRead;

    res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) return;

    if (f_open(&file, filename, FA_READ) == FR_OK) {
        // LED Status: PB7 (LD2) ON indicates busy/transferring
        HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET); 

        // Send Header
        sprintf(msg, "START:%s:%lu\r\n", filename, f_size(&file));
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 1000);

        while (1) {
            res = f_read(&file, buffer, sizeof(buffer), &bytesRead);
            if (res != FR_OK || bytesRead == 0) break;

            Base64_Encode(buffer, bytesRead, encoded_buffer);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)encoded_buffer, strlen(encoded_buffer), 1000);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n", 1, 1000);
            
            // LED Status: Toggle PB14 (LD3) to show activity/speed
            HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        }
        f_close(&file);
        
        // Send Footer
        sprintf(msg, "\r\nEXPORT_COMPLETE\r\n");
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 1000);

        // Turn LEDs OFF
        HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
}
