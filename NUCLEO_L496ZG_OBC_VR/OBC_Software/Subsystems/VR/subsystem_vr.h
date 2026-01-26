#ifndef SUBSYSTEM_VR_H
#define SUBSYSTEM_VR_H

#include <stdint.h>
#include "main.h"

// VR Subsystem Constants
#define VR_CMD_PING         0x10
#define VR_CMD_STATUS_RES   0x11
#define VR_CMD_CAPTURE_RES  0x12
#define VR_CMD_CHUNK_RES    0x13

#define VR_STATUS_REQ       0x20  // Command from GS to request VR Status
#define VR_TIMEOUT_MS       5000  // VR Timeout in ms

typedef struct {
    uint8_t download_active;
    uint16_t next_chunk_to_req;
    uint32_t last_seen_tick;
    uint8_t is_online;
} VR_State_t;

void VR_Init(void);
void VR_Handle_Packet(uint8_t* payload, uint16_t len);
void VR_SendCmd(uint8_t cmd_id);
void VR_Update(void);
uint8_t VR_IsOnline(void);

#endif
