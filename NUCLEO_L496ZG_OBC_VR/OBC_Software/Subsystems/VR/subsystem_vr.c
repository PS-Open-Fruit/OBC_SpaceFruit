#include "subsystem_vr.h"
#include "obc_packet.h"
#include "usbd_cdc_if.h" // For CDC_Transmit_FS
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef hlpuart1; // Need access to send data to GS

VR_State_t vr_state;

void VR_Init(void) {
    vr_state.download_active = 0;
    vr_state.next_chunk_to_req = 0;
}

// Helper to Log via GS Link (duplicated from main.c for now, can be centralized later)
extern void OBC_Log(const char *fmt, ...); 

void VR_Handle_Packet(uint8_t* decoded, uint16_t dec_len) {
    // Validate CRC
    if (dec_len < 5) {
        OBC_Log("[VR] Error: Payload short (%d)", dec_len);
        return;
    }
    
    // Extract CRC
    uint32_t rx_crc;
    memcpy(&rx_crc, &decoded[dec_len-4], 4);
    
    // Calc CRC
    uint32_t calc_crc = OBC_Calculate_CRC(decoded, dec_len-4);
    
    if (calc_crc != rx_crc) {
        OBC_Log("[VR] CRC Fail: %08X vs %08X", rx_crc, calc_crc);
        return;
    }

    uint8_t cmd_id = decoded[0];

    switch (cmd_id) {
        case VR_CMD_PING:
            OBC_Log("[VR] >> PONG Received! Link Active.");
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            break;
            
        case VR_CMD_CAPTURE_RES: {
            // [0x12] [ImgID:2] [Size:4]
            uint32_t img_size;
            memcpy(&img_size, &decoded[3], 4);
            OBC_Log("[VR] Image Captured! Size: %lu bytes. Starting Download...", img_size);
            
            // Forward Start Packet to GS (CMD 0x12)
            // Payload: [0x12] [ImgID:2] [Size:4]
            uint8_t gs_payload[16];
            memcpy(gs_payload, decoded, 7); 
            
            // Append CRC
            uint32_t crc_gs = OBC_Calculate_CRC(gs_payload, 7);
            memcpy(&gs_payload[7], &crc_gs, 4);
            
            uint8_t tx_frame_gs[32];
            uint16_t len_gs = SLIP_Encode(gs_payload, 11, tx_frame_gs);
            HAL_UART_Transmit(&hlpuart1, tx_frame_gs, len_gs, 100);

            vr_state.download_active = 1;
            vr_state.next_chunk_to_req = 0;

            // Request First Chunk
            uint8_t req[8] = {0x00, VR_CMD_CHUNK_RES, 0x00, 0x00, 0, 0, 0, 0}; // CMD+SubCMD+Chunk+CRC
            uint32_t crc_req = OBC_Calculate_CRC(req, 4);
            memcpy(&req[4], &crc_req, 4);

            uint8_t tx_frame[32];
            uint16_t len = SLIP_Encode(req, 8, tx_frame);
            CDC_Transmit_FS(tx_frame, len);
            break;
        }

        case VR_CMD_STATUS_RES: {
            if (dec_len >= 12) {
                uint8_t cpu = decoded[1];
                float temp;
                uint16_t ram;
                uint32_t disk;
                
                memcpy(&temp, &decoded[2], 4);
                memcpy(&ram, &decoded[6], 2);
                memcpy(&disk, &decoded[8], 4);
                
                int t_int = (int)temp;
                int t_dec = (int)((temp - t_int) * 10);
                if (t_dec < 0) t_dec = -t_dec;
                
                OBC_Log("[VR] STATUS >> CPU: %d%% | Temp: %d.%d C | RAM: %d MB | Disk: %lu MB", cpu, t_int, t_dec, ram, disk);
            }
            break;
        }

        case VR_CMD_CHUNK_RES: {
            // [0x13] [ChunkID:2] [Data... (N)] [CRC:4]
            
            uint16_t chunk_id;
            memcpy(&chunk_id, &decoded[1], 2);
            
            uint16_t data_len = (dec_len >= 7) ? (dec_len - 7) : 0;
            uint8_t* raw_data = &decoded[3];
            
            if (chunk_id == vr_state.next_chunk_to_req) {
                 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

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

void VR_Update(void) {
    // nothing for now, driven by packets
}
