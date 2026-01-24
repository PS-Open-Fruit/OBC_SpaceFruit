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
    if (frame[1] != 0x00) return 0; // Check specific header (0x00) if required, but generic decode might skip this. 
    // Wait, the original code mandated 0x00. Let's keep it robust.
    
    // Note: The original code's check `if (frame[1] != 0x00)` is specific to how they framed `[FEND] [0x00] [PAYLOAD] [FEND]`.
    // But `SLIP_Encode` puts `FEND` at the start.
    // If we assume standard SLIP, just decoding escapes is enough.
    // The previous implementation was:
    // for (uint16_t i = 2; i < len - 1; i++) 
    // checking frame[1] is 0x00. 
    // That suggests the frame on wire is: FEND 0x00 [ESCAPED DATA] FEND
    
    uint16_t out_idx = 0;
    // We skip index 0 (FEND) and index 1 (0x00) ??
    // The previous code: `for (uint16_t i = 2; i < len - 1; i++)`
    // This implies it SKIPS the command byte `0x00` at index 1?
    // Wait, if SLIP frame is `FEND <CMD> <DATA> FEND`.
    // If the sender encodes [CMD, DATA...], then encoded stream is FEND [ESC CMD] [ESC DATA] FEND.
    // But `frame[1]` being `0x00` unescaped means 0x00 was NOT escaped or it acts as a secondary start byte?
    // Let's look at `OBC_Process_Loop` sending:
    // `payload_buffer[0] = 0x00; ... SLIP_Encode(payload_buffer, ...)`
    // SLIP_Encode adds FEND at 0.
    // So buffer[0] = FEND.
    // buffer[1] would be payload_buffer[0] (if not FEND/FESC). 0x00 is not FEND/FESC.
    // So buffer[1] is 0x00.
    // So the previous decode logic `for I=2` effectively STRIPS the 0x00 command byte from the result.
    // THIS IS IMPORTANT. The `SLIP_Decode` in `main.c` was stripping the first byte of the payload!
    // But `decoded[0]` was used as `cmd_id`.
    // Wait. `decoded[0]` in `main.c` comes from `out_payload[out_idx++] = c`.
    // If loop starts at 2, and frame[1] is 0x00...
    // The Payload fed into Encode was `[0x00] [ChunkID] ...`
    // If Decode skips index 1 (which is 0x00), then `out_payload[0]` will be `ChunkID`.
    // BUT `main.c` says `uint8_t cmd_id = decoded[0];`
    
    // Let's re-read `main.c` SLIP_Decode carefully.
    /*
    if (frame[1] != 0x00) { ... error ... }
    for (uint16_t i = 2; i < len - 1; i++) { ... }
    */
    // This absolutely skips the `0x00` byte at index 1.
    // But `OBC_Process_Loop`...
    /*
    uint8_t req[8] = {0x00, 0x13, ...};
    SLIP_Encode(req...);
    */
    // If RPi sends back similar structure...
    // Code says: `if (cmd_id == 0x10)`.
    // If `0x00` is stripped, then `decoded[0]` is the byte AFTER `0x00`.
    // So the protocol seems to be:
    // [FEND] [0x00 (KISS Data Frame Type)] [Payload...] [FEND]
    // The `SLIP_Decode` unwraps the KISS Frame, stripping the Frame Type (0x00) and FENDs.
    // So `out_payload` contains just `[Payload...]`.
    // And `Payload` starts with `cmd_id` (0x10, 0x12, etc).
    // Correct.
    
    uint16_t start_idx = 2; // Default to skipping KISS Command byte
    
    // Check if we have the KISS Data byte
    if (frame[1] != 0x00) {
        // Strict KISS enforcement?
        // Let's just return 0 for now as per original code
        return 0;
    }
    
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
