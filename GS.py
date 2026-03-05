import serial
import struct
import time
import threading
import queue

# Import your custom KISS protocol class
from Shared.Python.kiss_protocol import KISSProtocol

# --- CONFIGURATION ---
PORT = 'COM4'  # Change to your virtual or real COM port
BAUD = 9600

# --- SIMULATED REQUESTS (PayloadID, PID, Description) ---
# Emulating the Excel design flow
REQUESTS = [
    (0x00, 0x00, "Request List files (SD)"),
    (0x00, 0x01, "Request File Info (SD)"),
    (0x00, 0x02, "Request File Data (SD)"),
    (0x01, 0x00, "Request Pi Status (VR)"),
    (0x01, 0x01, "Request Capture (VR)")
]

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

command_queue = queue.Queue()

def cli_thread():
    print("\n\033[1;33m--- Command Line Interface ---\033[0m")
    print("Commands:")
    for i, req in enumerate(REQUESTS):
        print(f"  {i}: {req[2]}")
    print("\033[3mType the number and press Enter to send.\033[0m")
    while True:
        try:
            choice = input()
            idx = int(choice.strip())
            if 0 <= idx < len(REQUESTS):
                p_id, pid, desc = REQUESTS[idx]
                req_data = b''
                
                # Build specific request paylods
                if p_id == 0x00 and pid == 0x01: # Info
                    fname = input("  Enter Filename: ").encode()
                    req_data = struct.pack('>B', len(fname)) + fname
                elif p_id == 0x00 and pid == 0x02: # File Data
                    fname = input("  Enter Filename: ").encode()
                    offset = int(input("  Enter Offset: ") or "0")
                    length = int(input("  Enter Read Length: ") or "1024")
                    req_data = struct.pack('>B', len(fname)) + fname + struct.pack('>IH', offset, length)
                    
                command_queue.put((p_id, pid, desc, req_data))
            else:
                print("Invalid choice.")
        except Exception as e:
            print(f"Input Error: {e}")

def build_custom_payload(payload_id: int, pid: int, seq_num: int, data: bytes) -> bytes:
    data_len = len(data)
    header = struct.pack('>BBBH', seq_num, payload_id, pid, data_len)
    content = header + data
    crc = KISSProtocol.calculate_crc(content)
    return content + struct.pack('>I', crc)

def parse_custom_payload(payload_bytes: bytes):
    if len(payload_bytes) < 9: 
        return None
    content = payload_bytes[:-4]
    received_crc = struct.unpack('>I', payload_bytes[-4:])[0]
    calculated_crc = KISSProtocol.calculate_crc(content)
    if received_crc != calculated_crc:
        return None
    seq_num, payload_id, pid, data_len = struct.unpack('>BBBH', content[:5])
    data = content[5:5+data_len]
    return payload_id, pid, seq_num, data

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Ground Station Started on {PORT}.")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    highest_seq_received = -1
    packets_received_in_window = 0
    
    threading.Thread(target=cli_thread, daemon=True).start()
    
    while True:
        # 1. Process CLI Commands
        try:
            target_p_id, target_pid, desc, req_data = command_queue.get_nowait()
            print(f"\n[GS] Sending: {desc} (PayloadID: 0x{target_p_id:02X}, PID: 0x{target_pid:02X})")
            custom_payload = build_custom_payload(target_p_id, target_pid, 0x00, req_data) 
            req_frame = KISSProtocol.wrap_frame(custom_payload, command=0x00)
            
            print("  Color Legend: \033[90mFEND\033[0m \033[95mCMD\033[0m \033[94mSEQ\033[0m \033[93mPL_ID\033[0m \033[96mPID\033[0m \033[92mLEN\033[0m \033[97mDATA\033[0m \033[91mCRC\033[0m \033[90mFEND\033[0m")
            print(f"  -> TX Raw Frame: {colorize_raw_frame(req_frame)}")
            ser.write(req_frame)
            
            highest_seq_received = -1
            packets_received_in_window = 0
        except queue.Empty:
            pass

        # 2. Process Incoming Bytes
        byte = ser.read(1)
        if byte:
            if byte == FEND_BYTE:
                rx_buffer += byte
                if len(rx_buffer) >= 3:
                    unwrapped = KISSProtocol.unwrap_frame(bytes(rx_buffer))
                    if unwrapped:
                        print(f"  <- RX Raw Frame: {colorize_raw_frame(rx_buffer)}")
                        cmd, payload_bytes = unwrapped
                        parsed = parse_custom_payload(payload_bytes)
                        
                        if parsed:
                            p_id, pid, seq, data = parsed
                            
                            # Handle Data Frame (Command 0x01)
                            if cmd == 0x01:
                                print(f"  <- Received Response [PayloadID: 0x{p_id:02X}, PID: 0x{pid:02X}, Seq: {seq}]")
                                try:
                                    if p_id == 0x00:
                                        if pid == 0x00: # List Files
                                            num_files = data[0]
                                            print(f"     Found {num_files} files:")
                                            offset = 1
                                            for _ in range(num_files):
                                                name_len = data[offset]
                                                name = data[offset+1 : offset+1+name_len].decode()
                                                print(f"       - {name}")
                                                offset += 1 + name_len
                                                
                                        elif pid == 0x01: # File Info
                                            status, size, ts = struct.unpack('>BII', data)
                                            s_str = "OK" if status == 0 else "Error"
                                            print(f"     Status: {s_str} | Size: {size} bytes | Created: {ts}")
                                            
                                        elif pid == 0x02: # File Data
                                            status, offset, dl = struct.unpack('>BIH', data[:7])
                                            chunk = data[7:7+dl]
                                            print(f"     Status: {status} | Offset: {offset} | Len: {dl}")
                                            print(f"     Data: {chunk.hex(' ')}")
                                            
                                    elif p_id == 0x01:
                                        if pid == 0x00: # Pi Status
                                            ts, up, cpu_l, cpu_t, ram, disk, cam = struct.unpack('>IIBbBBB3x', data)
                                            cam_str = {0:"Err", 1:"Ready", 2:"Busy"}.get(cam, "Unknown")
                                            print(f"     Pi Status -> Time: {ts} | Up: {up}s | CPU: {cpu_l}% ({cpu_t}C) | RAM: {ram}% | Disk: {disk}% | CAM: {cam_str}")
                                            
                                        elif pid == 0x01: # Capture
                                            status, name_len = struct.unpack('>BB', data[:2])
                                            name = data[2:2+name_len].decode()
                                            print(f"     Capture -> Status: {status} | File: {name}")

                                except Exception as e:
                                    print(f"     \033[91mParse Error:\033[0m {e} (Raw: {data.hex()})")
                                
                                highest_seq_received = max(highest_seq_received, seq)
                                packets_received_in_window += 1
                                
                                # Send ACK when window is complete
                                if packets_received_in_window == 5:
                                    print(f"[GS] Window complete. Sending ACK for SeqNum {highest_seq_received}")
                                    ack_payload = build_custom_payload(0x00, 0x00, highest_seq_received, b'')
                                    ack_frame = KISSProtocol.wrap_frame(ack_payload, command=0xAC)
                                    print(f"  -> TX Raw Frame: {colorize_raw_frame(ack_frame)}")
                                    ser.write(ack_frame)
                                    
                rx_buffer = bytearray(FEND_BYTE)
            else:
                if len(rx_buffer) > 0:
                    rx_buffer += byte

if __name__ == '__main__':
    main()