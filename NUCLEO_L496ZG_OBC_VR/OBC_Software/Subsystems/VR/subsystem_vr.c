#include "subsystem_vr.h"
#include "obc_packet.h"
#include "usbd_cdc_if.h" // For CDC_Transmit_FS
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef hlpuart1; // Need access to send data to GS
extern uint32_t gs_led_timer; // GS LED Timer from main.c

VR_State_t vr_state;
uint32_t vr_led_timer = 0;

void VR_Init(void) {
    vr_state.download_active = 0;
    vr_state.next_chunk_to_req = 0;
    vr_state.last_seen_tick = 0;
    vr_state.is_online = 0;
    vr_state.gs_ping_pending = 0;
}

// Helper to Log via GS Link (duplicated from main.c for now, can be centralized later)
extern void OBC_Log(const char *fmt, ...); 

void VR_Handle_Packet(uint8_t* decoded, uint16_t dec_len) {
    // Mark VR as Online
    uint8_t was_online = vr_state.is_online;
    vr_state.is_online = 1;
    vr_state.last_seen_tick = HAL_GetTick();

    // Validate CRC
    if (dec_len < 5) {
        OBC_Log("[VR] Error: Payload short (%d)", dec_len);
        return;
    }
    
    // Extract CRC
    uint32_t rx_crc;
    memcpy(&rx_crc, &decoded[dec_len-4], 4);
    
    // Calc CRC
    // decoded[0] is now KISS Command (usually 0x00)
    // Application Command is at decoded[1]
    uint32_t calc_crc = OBC_Calculate_CRC(decoded, dec_len-4);
    uint8_t kiss_cmd = decoded[0];
    uint8_t cmd_id = decoded[1]; // Shifted due to SLIP_Decode change
    
    if (calc_crc != rx_crc) {
        if (cmd_id == VR_CMD_CHUNK_RES) {
             OBC_Log("[VR] CRC Fail on Img Chunk %02X! Forwarding for SSDV fix...", decoded[2]);
        } else {
             OBC_Log("[VR] CRC Fail: %08X vs %08X (Cmd: %02X). Dropped.", rx_crc, calc_crc, cmd_id);
             return;
        }
    }
    
    // Only process if it's a Data Frame
    if (kiss_cmd != 0x00) {
        return;
    }

    switch (cmd_id) {
        case VR_CMD_PING:
            if (!was_online) {
                OBC_Log("[VR] Connection Restored!");
            }
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Blink On
            vr_led_timer = HAL_GetTick();
            
            // Only log Pong if GS requested it
            if (vr_state.gs_ping_pending) {
                OBC_Log("[VR] Pong!");
                vr_state.gs_ping_pending = 0;
            }
            break;
            
        case VR_CMD_CAPTURE_RES: {
            // [0x12] [ImgID:2] [Size:4] [CRC:4]
            // decoded map: 0:KISS, 1:CMD, 2:ID, 4:Size
            uint32_t img_size;
            memcpy(&img_size, &decoded[4], 4);
            OBC_Log("[VR] Image Captured! Size: %lu bytes. Starting Download...", img_size);
            
            // Forward Start Packet to GS (CMD 0x12)
            // Header is already processed, we want to construct GS packet:
            // [0x12] [ImgID] [Size] ...
            // decoded[2] is ID start. Copy 10 bytes (ID:2 + Size:4 + CRC:4)
            uint8_t gs_payload[20];
            gs_payload[0] = 0x12;
            memcpy(&gs_payload[1], &decoded[2], 10); 
            
            // Append CRC
            uint32_t crc_gs = OBC_Calculate_CRC(gs_payload, 11);
            memcpy(&gs_payload[11], &crc_gs, 4);
            
            uint8_t tx_frame_gs[32];
            uint16_t len_gs = SLIP_Encode(gs_payload, 15, tx_frame_gs);
            HAL_UART_Transmit(&hlpuart1, tx_frame_gs, len_gs, 100);

            // Blink GS LED
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            gs_led_timer = HAL_GetTick();

            vr_state.download_active = 1;
            vr_state.next_chunk_to_req = 0;

            // Request First Chunk
            uint8_t req[8] = {0x00, VR_CMD_CHUNK_RES, 0x00, 0x00, 0, 0, 0, 0}; // CMD+SubCMD+Chunk+CRC
            uint32_t crc_req = OBC_Calculate_CRC(req, 4);
            memcpy(&req[4], &crc_req, 4);

            uint8_t tx_frame[32];
            uint16_t len = SLIP_Encode(req, 8, tx_frame);
            CDC_Transmit_FS(tx_frame, len);
            
            vr_state.last_chunk_req_tick = HAL_GetTick(); // Track Request Time
            break;
        }

        case VR_CMD_STATUS_RES: {
            // Forward status packet to GS (CMD 0x21)
            // Payload: [0x21] + [VR_Payload] + [VR_CRC]
            
            // dec_len includes: [CMD 0x11] [Payload...] [CRC 4 bytes]
            if (dec_len < 5) return;

            // We include the RPi's CRC in the forwarded packet for transparency
            uint16_t data_len = (dec_len >= 6) ? (dec_len - 6) : 0; // Exclude KISS(1)+VRCMD(1)+CRC(4)
            
            uint8_t gs_payload[64];
            gs_payload[0] = 0x21; // GS VR Status Report CMD
            memcpy(&gs_payload[1], &decoded[2], data_len);
            
            // Append GS CRC
            uint32_t crc_gs = OBC_Calculate_CRC(gs_payload, 1 + data_len);
            memcpy(&gs_payload[1 + data_len], &crc_gs, 4);
            
            uint8_t tx_frame_gs[128];
            uint16_t len_gs = SLIP_Encode(gs_payload, 1 + data_len + 4, tx_frame_gs);
            HAL_UART_Transmit(&hlpuart1, tx_frame_gs, len_gs, 100);

            // Blink GS LED
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            gs_led_timer = HAL_GetTick();
            
            // Blink debug LED
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            vr_led_timer = HAL_GetTick();
            break;
        }

        case VR_CMD_CHUNK_RES: {
            // [0x13] [ChunkID:2] [Data... (N)] [CRC:4]
            // decoded map: 0:KISS, 1:CMD, 2:ID_L, 3:ID_H, 4..:DATA
            
            uint16_t chunk_id;
            memcpy(&chunk_id, &decoded[2], 2);
            
            uint16_t data_len = (dec_len >= 8) ? (dec_len - 8) : 0;
            uint8_t* raw_data = &decoded[4];
            
            if (chunk_id == vr_state.next_chunk_to_req) {
                 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Blink On
                 vr_led_timer = HAL_GetTick();

                 // Forward to GS
                 // We need to reconstruct the KISS Frame manually here because SLIP_Encode is generic
                 
                 uint8_t payload_buffer[600];
                 payload_buffer[0] = 0x00; // KISS Data Command
                 payload_buffer[1] = chunk_id & 0xFF;
                 payload_buffer[2] = (chunk_id >> 8) & 0xFF;
                 memcpy(&payload_buffer[3], raw_data, data_len);
                 
                 uint16_t current_payload_len = 3 + data_len;
                 
                 // Append CRC
                 uint32_t crc_hw = OBC_Calculate_CRC(payload_buffer, current_payload_len);
                 payload_buffer[current_payload_len++] = crc_hw & 0xFF;
                 payload_buffer[current_payload_len++] = (crc_hw >> 8) & 0xFF;
                 payload_buffer[current_payload_len++] = (crc_hw >> 16) & 0xFF;
                 payload_buffer[current_payload_len++] = (crc_hw >> 24) & 0xFF;

                 uint8_t kiss_tx_buffer[1200];
                 uint16_t kiss_len = SLIP_Encode(payload_buffer, current_payload_len, kiss_tx_buffer);
                 
                 HAL_UART_Transmit(&hlpuart1, kiss_tx_buffer, kiss_len, 2000);

                 // Blink GS LED
                 HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
                 gs_led_timer = HAL_GetTick();
                 
                 vr_state.next_chunk_to_req++;
                 
                 if (data_len < 200) {
                     OBC_Log("[VR] Download Complete!");
                     vr_state.download_active = 0;
                 } else {
                     // Request Next
                     uint8_t req[8];
                     req[0] = 0x00; 
                     req[1] = VR_CMD_CHUNK_RES;
                     req[2] = vr_state.next_chunk_to_req & 0xFF;
                     req[3] = (vr_state.next_chunk_to_req >> 8) & 0xFF;
                     
                     uint32_t crc_next = OBC_Calculate_CRC(req, 4);
                     memcpy(&req[4], &crc_next, 4);
                     
                     uint8_t tx_frame[32];
                     uint16_t len = SLIP_Encode(req, 8, tx_frame);
                     CDC_Transmit_FS(tx_frame, len);
                     
                     vr_state.last_chunk_req_tick = HAL_GetTick(); // Track Request Time
                 }
            }
            break;
        }

        default:
            OBC_Log("[VR] Unknown Cmd: %02X", cmd_id);
            break;
    }
}

void VR_SendCmd(uint8_t cmd_id) {
    uint8_t payload[6];
    payload[0] = 0x00; // KISS Data
    payload[1] = cmd_id;
    
    uint32_t crc = OBC_Calculate_CRC(payload, 2); 
    memcpy(&payload[2], &crc, 4);
    
    uint8_t tx_frame[32];
    uint16_t len = SLIP_Encode(payload, 6, tx_frame);
    CDC_Transmit_FS(tx_frame, len);
}

void VR_RequestGSPing(void) {
    vr_state.gs_ping_pending = 1;
    VR_SendCmd(VR_CMD_PING);
}

void VR_Update(void) {
    uint32_t now = HAL_GetTick();

    // LED Off Logic
    if (vr_led_timer > 0 && (now - vr_led_timer > 50)) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        vr_led_timer = 0;
    }

    // Check Timeout
    if (vr_state.is_online && (now - vr_state.last_seen_tick > VR_TIMEOUT_MS)) {
        vr_state.is_online = 0;
        OBC_Log("[VR] Connection Lost (Timeout)");
    }

    // Auto-Ping (Keep-Alive)
    static uint32_t last_ping = 0;
    if (now - last_ping > 2000) {
        last_ping = now;
        VR_SendCmd(VR_CMD_PING);
    }

    // Chunk Retry Mechanism (Every 500ms if stuck)
    if (vr_state.download_active && vr_state.is_online) {
        if (now - vr_state.last_chunk_req_tick > 500) {
             // Retry Request
             uint8_t req[8];
             req[0] = 0x00; 
             req[1] = VR_CMD_CHUNK_RES;
             req[2] = vr_state.next_chunk_to_req & 0xFF; // Request SAME chunk
             req[3] = (vr_state.next_chunk_to_req >> 8) & 0xFF;
             
             uint32_t crc = OBC_Calculate_CRC(req, 4);
             memcpy(&req[4], &crc, 4);
             
             uint8_t tx_frame[32];
             uint16_t len = SLIP_Encode(req, 8, tx_frame);
             CDC_Transmit_FS(tx_frame, len);
             
             vr_state.last_chunk_req_tick = now;
             
             // Log sparingly?
             OBC_Log("[VR] Retry Chunk %d", vr_state.next_chunk_to_req);
        }
    }
}

uint8_t VR_IsOnline(void) {
    return vr_state.is_online;
}
