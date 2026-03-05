import serial
import struct
import time
import random

# Import your custom KISS protocol class
from Shared.Python.kiss_protocol import KISSProtocol

# --- CONFIGURATION ---
PORT = 'COM5'  # Change to your virtual or real COM port
BAUD = 9600

# --- SUBSYSTEMS ---
OBC_SUBSYSTEM = 0x00
VR_SUBSYSTEM = 0x01

def colorize_raw_frame(frame: bytes) -> str:
    """Colors the raw KISS frame hex string for easier reading."""
    if len(frame) < 12: return frame.hex(' ').upper()
    parts = [
        f"\033[90m{frame[0:1].hex().upper()}\033[0m", # FEND
        f"\033[95m{frame[1:2].hex().upper()}\033[0m", # CMD
        f"\033[94m{frame[2:3].hex().upper()}\033[0m", # SeqNum
        f"\033[93m{frame[3:4].hex().upper()}\033[0m", # PayloadID
        f"\033[96m{frame[4:5].hex().upper()}\033[0m", # PID
        f"\033[92m{frame[5:7].hex(' ').upper()}\033[0m", # DataLen
    ]
    data_len = len(frame) - 12
    if data_len > 0:
        parts.append(f"\033[97m{frame[7:7+data_len].hex(' ').upper()}\033[0m") # Data
    parts.append(f"\033[91m{frame[-5:-1].hex(' ').upper()}\033[0m") # CRC
    parts.append(f"\033[90m{frame[-1:].hex().upper()}\033[0m") # FEND
    return " ".join(parts)

def build_custom_payload(payload_id: int, pid: int, seq_num: int, data: bytes) -> bytes:
    """Builds the internal payload: [SeqNum][PayloadID][PID][DataLen][Data][CRC32]"""
    data_len = len(data)
    # Pack Header: > (Big-Endian), B (1-byte), H (2-byte)
    header = struct.pack('>BBBH', seq_num, payload_id, pid, data_len)
    content = header + data
    
    # Calculate and append CRC32
    crc = KISSProtocol.calculate_crc(content)
    return content + struct.pack('>I', crc)

def parse_custom_payload(payload_bytes: bytes):
    """Validates CRC and extracts the custom payload fields."""
    if len(payload_bytes) < 9:  # Min size: 5 (header) + 0 (data) + 4 (crc)
        return None
        
    content = payload_bytes[:-4]
    received_crc = struct.unpack('>I', payload_bytes[-4:])[0]
    calculated_crc = KISSProtocol.calculate_crc(content)
    
    if received_crc != calculated_crc:
        print("CRC Error!")
        return None
        
    seq_num, payload_id, pid, data_len = struct.unpack('>BBBH', content[:5])
    data = content[5:5+data_len]
    return payload_id, pid, seq_num, data

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"OBC Started on {PORT}. Waiting for Ground Station requests...")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    seq_counter = 0
    
    while True:
        byte = ser.read(1)
        if byte:
            if byte == FEND_BYTE:
                rx_buffer += byte
                # If buffer has a start FEND, command byte, and end FEND, attempt unwrap
                if len(rx_buffer) >= 3:
                    unwrapped = KISSProtocol.unwrap_frame(bytes(rx_buffer))
                    if unwrapped:
                        print(f"  <- RX Raw Frame: {colorize_raw_frame(rx_buffer)}")
                        cmd, payload_bytes = unwrapped
                        parsed = parse_custom_payload(payload_bytes)
                        
                        if parsed:
                            p_id, pid, seq, data = parsed
                            
                            # Handle GS Request (Command 0x00)
                            if cmd == 0x00:
                                print(f"\n[OBC] Received Request - PayloadID: 0x{p_id:02X}, PID: 0x{pid:02X}")
                                
                                # --- ROUTING LOGIC ---
                                if p_id == OBC_SUBSYSTEM:
                                    print("  -> Routing to: OBC Subsystem (SD Card)")
                                    if pid == 0x00: print("  -> Action: Request List files")
                                    elif pid == 0x01: print("  -> Action: Request File Info")
                                    elif pid == 0x02: print("  -> Action: Request File Data")
                                    else: print("  -> Action: Unknown OBC Command")
                                        
                                elif p_id == VR_SUBSYSTEM:
                                    print("  -> Routing to: VR Payload Subsystem")
                                    if pid == 0x00: print("  -> Action: Request Pi Status")
                                    elif pid == 0x01: print("  -> Action: Request Capture")
                                    elif pid == 0x02: print("  -> Action: Request Copy Image to SD")
                                    else: print("  -> Action: Unknown VR Command")
                                else:
                                    print("  -> Routing to: Unknown Subsystem!")
                                
                                # Emulate sending a response back to GS
                                seq_counter = (seq_counter + 1) % 256
                                
                                # Generate specific dummy data based on the request
                                dummy_data = b'OK'
                                if p_id == OBC_SUBSYSTEM:
                                    if pid == 0x00:
                                        dummy_data = b'image_01.jpg,telemetry.csv,log.txt'
                                    elif pid == 0x01:
                                        dummy_data = b'image_01.jpg|2.4MB|2026-03-05'
                                    elif pid == 0x02:
                                        dummy_data = b'\\xFF\\xD8\\xFF\\xE0 (Simulated JPEG Magic Bytes)'
                                elif p_id == VR_SUBSYSTEM:
                                    if pid == 0x00:
                                        dummy_data = b'CPU: 45C | RAM: 60% | CAM: Ready'
                                    elif pid == 0x01:
                                        dummy_data = b'Capture Success! Saved as image_02.jpg'
                                    elif pid == 0x02:
                                        dummy_data = b'Copying image_02.jpg to SD -> Done.'
                                
                                # Send response (Command 0x01) echoing the p_id and pid
                                custom_payload = build_custom_payload(p_id, pid, seq_counter, dummy_data)
                                tx_frame = KISSProtocol.wrap_frame(custom_payload, command=0x01)
                                
                                print(f"  -> TX Raw Frame: {colorize_raw_frame(tx_frame)}")
                                ser.write(tx_frame)
                                    
                            # Handle GS ACK (Command 0xAC)
                            elif cmd == 0xAC:
                                print(f"[OBC] Received ACK for SeqNum up to: {seq}")
                                
                # Reset buffer and start with FEND for the next potential frame
                rx_buffer = bytearray(FEND_BYTE)
            else:
                # Only append if we've initialized the buffer with a FEND
                if len(rx_buffer) > 0:
                    rx_buffer += byte

if __name__ == '__main__':
    main()