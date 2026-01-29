#include "obc_packet.h"
#include <string.h>

static CRC_HandleTypeDef *phcrc = NULL;

void OBC_Packet_Init(CRC_HandleTypeDef *hcrc_handle) {
    phcrc = hcrc_handle;
}

uint32_t OBC_Calculate_CRC(uint8_t* buffer, uint32_t len) {
    if (phcrc == NULL) return 0;
    
    // Checksum over buffer
    // Note: If buffer is not 32-bit aligned, HAL_CRC might require specific handling or settings
    // But for this code base, it seems to work as is.
    uint32_t crc = HAL_CRC_Calculate(phcrc, (uint32_t*)buffer, len);
    return crc ^ 0xFFFFFFFF; // Inverse for Standard CRC32
}

uint16_t SLIP_Encode(uint8_t* payload, uint16_t len, uint8_t* out_buffer) {
    uint16_t idx = 0;
    out_buffer[idx++] = FEND;
    
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

uint16_t SLIP_Decode(uint8_t* frame, uint16_t len, uint8_t* out_payload) {
    // Basic validation moved to caller or handled here 
    // Logic copied from original main.c
    
    if (len < 3) return 0;
    if (frame[0] != FEND || frame[len-1] != FEND) return 0;

    // We decode everything between the FENDs, including the KISS Command Byte (frame[1])
    // This allows the CRC check to cover [CMD] + [PAYLOAD] correctly.
    
    uint16_t out_idx = 0;
    uint16_t start_idx = 1; // Skip only the Start FEND
    
    uint16_t out_i = 0;
    for (uint16_t i = start_idx; i < len - 1; i++) {
        uint8_t c = frame[i];
        if (c == FESC) {
            i++;
            if (i >= len - 1) return 0;
            if (frame[i] == TFEND) out_payload[out_i++] = FEND;
            else if (frame[i] == TFESC) out_payload[out_i++] = FESC;
        } else {
            out_payload[out_i++] = c;
        }
    }
    return out_i;
}
