/*
 * sd_utils.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Antigravity
 */

#ifndef SD_UTILS_H
#define SD_UTILS_H

#include "../../../Core/Inc/main.h"

void Base64_Encode(uint8_t *input, uint16_t input_len, char *output);
void SD_Init(void);
void SD_SendFile(char *filename, uint16_t sector);
void SD_SaveFiles(void);
void SD_ListFiles_KISS(void);
void SD_SendFile_Windowed(char *filename);

#endif /* SD_UTILS_H */
