#ifndef OBC_PACKET_H
#define OBC_PACKET_H

#include <stdint.h>
#include "main.h"

// SLIP Constants
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD
#define MAX_FRAME_SIZE 512

// Function Prototypes
void OBC_Packet_Init(CRC_HandleTypeDef *hcrc_handle);
uint16_t SLIP_Encode(uint8_t* payload, uint16_t len, uint8_t* out_buffer);
uint16_t SLIP_Decode(uint8_t* frame, uint16_t len, uint8_t* out_payload);
uint32_t OBC_Calculate_CRC(uint8_t* buffer, uint32_t len);

#endif
