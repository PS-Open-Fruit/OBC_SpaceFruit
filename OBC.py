import serial
import struct
import os

SD_CARD_DIR = "sd_card"
os.makedirs(SD_CARD_DIR, exist_ok=True)
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
                                    if pid == 0x00: print("  -> Action: Request Ping")
                                    elif pid == 0x01: print("  -> Action: Request List files")
                                    elif pid == 0x02: print("  -> Action: Request File Info")
                                    elif pid == 0x03: print("  -> Action: Request File Data")
                                    else: print("  -> Action: Unknown OBC Command")
                                        
                                elif p_id == VR_SUBSYSTEM:
                                    print("  -> Routing to: VR Payload Subsystem")
                                    if pid == 0x00: print("  -> Action: Request Ping")
                                    elif pid == 0x01: print("  -> Action: Request Pi Status")
                                    elif pid == 0x02: print("  -> Action: Request Capture")
                                    elif pid == 0x03: print("  -> Action: Request Copy Image to SD")
                                    else: print("  -> Action: Unknown VR Command")
                                else:
                                    print("  -> Routing to: Unknown Subsystem!")
                                
                                # Use the data already parsed from the incoming Request
                                req_data = data

                                # Emulate sending a response back to GS
                                seq_counter = (seq_counter + 1) % 256
                                
                                # Generate specific dummy binary data based on the Request
                                dummy_data = b''
                                if p_id == OBC_SUBSYSTEM:
                                    if pid == 0x00: # Ping
                                        dummy_data = b''
                                        
                                    elif pid == 0x01: # List Files
                                        try:
                                            files = os.listdir(SD_CARD_DIR)[:255] # limit to 1 byte count
                                            dummy_data = struct.pack('>B', len(files))
                                            for f in files:
                                                fb = f.encode('utf-8')
                                                dummy_data += struct.pack('>B', len(fb)) + fb
                                        except Exception as e:
                                            dummy_data = struct.pack('>B', 0)
                                            
                                    elif pid == 0x02: # File Info
                                        if len(req_data) >= 1:
                                            name_len = req_data[0]
                                            fname = req_data[1:1+name_len].decode('utf-8')
                                            fpath = os.path.join(SD_CARD_DIR, fname)
                                            if os.path.exists(fpath):
                                                sz = os.path.getsize(fpath)
                                                ct = int(os.path.getctime(fpath))
                                                dummy_data = struct.pack('>BII', 0x00, sz, ct)
                                            else:
                                                dummy_data = struct.pack('>BII', 0x01, 0, 0)
                                        else:
                                            dummy_data = struct.pack('>BII', 0x01, 0, 0)
                                            
                                    elif pid == 0x03: # File Data (Chunk)
                                        if len(req_data) >= 1:
                                            name_len = req_data[0]
                                            fname = req_data[1:1+name_len].decode('utf-8')
                                            fpath = os.path.join(SD_CARD_DIR, fname)
                                            if len(req_data) >= 1 + name_len + 6:
                                                offset, r_len = struct.unpack('>IH', req_data[1+name_len : 1+name_len+6])
                                                if os.path.exists(fpath):
                                                    try:
                                                        with open(fpath, 'rb') as f:
                                                            f.seek(offset)
                                                            chunk = f.read(r_len)
                                                            dummy_data = struct.pack('>BIH', 0x00, offset, len(chunk)) + chunk
                                                    except:
                                                        dummy_data = struct.pack('>BIH', 0x01, offset, 0)
                                                else:
                                                    dummy_data = struct.pack('>BIH', 0x01, offset, 0)
                                            else:
                                                dummy_data = struct.pack('>BIH', 0x01, 0, 0)
                                        else:
                                            dummy_data = struct.pack('>BIH', 0x01, 0, 0)

                                elif p_id == VR_SUBSYSTEM:
                                    if pid == 0x00: # Ping
                                        dummy_data = b''
                                        
                                    elif pid == 0x01: # Pi Status
                                        dummy_data = struct.pack('>IIBbBBB3x',
                                            1772722800, # Timestamp (uint32)
                                            3600,       # Uptime seconds (uint32)
                                            15,         # CPULoadPercent (uint8)
                                            45,         # CPUTemp (int8)
                                            60,         # RAMUsagePercent (uint8)
                                            30,         # DiskUsagePercent (uint8)
                                            0x01        # CameraStatus: Ready (uint8)
                                        )
                                        
                                    elif pid == 0x02: # Capture
                                        filename = b"image_02.jpg"
                                        dummy_data = struct.pack('>BB', 0x00, len(filename)) + filename
                                        
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