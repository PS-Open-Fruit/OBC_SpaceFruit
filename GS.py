import serial
import struct
import os

DOWNLOADS_DIR = "downloads"
os.makedirs(DOWNLOADS_DIR, exist_ok=True)
current_download_file = "unknown.bin"
import time
import threading
import queue
import sys
from beacon_helper import *

# Import your custom KISS protocol class
from Shared.Python.kiss_protocol import KISSProtocol

# --- CONFIGURATION ---
PORT = '/dev/cu.usbserial-A10OMHTZ'  # Change to your virtual or real COM port
BAUD = 9600



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
    print("\n\033[1;33m--- Ground Station CLI ---\033[0m")
    print("Type 'help' for a list of available commands.")
    while True:
        try:
            choice = input("GS> ").strip()
            if not choice:
                continue
            
            parts = choice.split()
            cmd = parts[0].lower()
            
            if cmd == 'help':
                print("Commands:")
                print("  ping <obc/vr>         - Ping subsystem")
                print("  list                  - List files on OBC SD card")
                print("  info <filename>       - Request file info")
                print("  download <filename>   - Download file from OBC")
                print("  status                - Request Pi Status")
                print("  capture               - Request Image Capture")
                print("  copy                  - Request Copy Image to SD")
                print("  exit                  - Exit Ground Station")
            elif cmd == 'ping':
                if len(parts) > 1 and parts[1].lower() == 'obc':
                    command_queue.put(('MANUAL', 0x00, 0x00, "Request Ping (OBC)", b''))
                elif len(parts) > 1 and parts[1].lower() == 'vr':
                    command_queue.put(('MANUAL', 0x01, 0x00, "Request Ping (VR)", b''))
                else:
                    print("Usage: ping <obc|vr>")
            elif cmd == 'list':
                command_queue.put(('MANUAL', 0x00, 0x01, "Request List files (SD)", b''))
            elif cmd == 'info':
                if len(parts) > 1:
                    fname = parts[1].encode()
                    req_data = struct.pack('>B', len(fname)) + fname
                    command_queue.put(('MANUAL', 0x00, 0x02, "Request File Info (SD)", req_data))
                else:
                    print("Usage: info <filename>")
            elif cmd == 'download':
                if len(parts) > 1:
                    fname = parts[1]
                    command_queue.put(('AUTO_DOWNLOAD', 0x00, 0x03, "Request File Data (SD)", fname))
                else:
                    print("Usage: download <filename>")
            elif cmd == 'status':
                command_queue.put(('MANUAL', 0x01, 0x01, "Request Pi Status (VR)", b''))
            elif cmd == 'capture':
                command_queue.put(('MANUAL', 0x01, 0x02, "Request Capture (VR)", b''))
            elif cmd == 'copy':
                command_queue.put(('MANUAL', 0x01, 0x03, "Request Copy Image to SD (VR)", b''))
            elif cmd == 'exit':
                print("Exiting...")
                os._exit(0)
            else:
                print(f"Unknown command: {choice}. Type 'help' for commands.")
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
    global beacon_count, last_beacon_rx_time, latest_beacon_data
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Ground Station Started on {PORT}.")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    highest_seq_received = -1
    packets_received_in_window = 0
    
    threading.Thread(target=cli_thread, daemon=True).start()
    
    # State flags for automated downloading
    dl_active = False
    dl_filename_bytes = b''
    dl_offset = 0
    dl_chunk_size = 1024
    dl_total_size = 0
    dl_start_time = 0.0
    
    last_request_time = 0.0
    retry_count = 0
    MAX_RETRIES = 5
    ping_send_time = 0.0
    
    last_beacon_rx_time = 0.0
    beacon_count = 0
    latest_beacon_data = None
    
    print("\n[GS] Starting RX/TX Loops...")
    
    while True:
        # 1. Process CLI Commands
        try:
            item = command_queue.get_nowait()
            
            # Handle the new tuple structures sent by the CLI
            if len(item) == 5 and item[0] == 'AUTO_DOWNLOAD':
                _, target_p_id, target_pid, desc, fname = item
                global current_download_file
                current_download_file = fname
                dl_filename_bytes = fname.encode()
                dl_offset = 0
                dl_active = True
                dl_total_size = 0
                retry_count = 0
                last_request_time = time.time()
                
                print(f"\n[GS] --- Starting Automated Download for '{fname}' ---")
                # Request File Info first to determine file size
                req_data = struct.pack('>B', len(dl_filename_bytes)) + dl_filename_bytes
                command_queue.put(('PRE_DOWNLOAD_INFO', 0x00, 0x02, "Auto-Request File Info", req_data))
                continue
                
            elif len(item) == 5 and item[0] in ['MANUAL', 'PRE_DOWNLOAD_INFO']:
                _, target_p_id, target_pid, desc, req_data = item
                if item[0] == 'MANUAL':
                    print(f"\n[GS] Sending: {desc} (PayloadID: 0x{target_p_id:02X}, PID: 0x{target_pid:02X})")
            
            else:
                # Fallback for old tuples that might still be in the queue right after a script refresh
                target_p_id, target_pid, desc, req_data = item
                print(f"\n[GS] Sending: {desc} (PayloadID: 0x{target_p_id:02X}, PID: 0x{target_pid:02X})")
                
            custom_payload = build_custom_payload(target_p_id, target_pid, 0x00, req_data) 
            req_frame = KISSProtocol.wrap_frame(custom_payload, command=0x00)
            
            if target_pid == 0x00: # Ping
                ping_send_time = time.time()
            
            print("  Color Legend: \033[90mFEND\033[0m \033[95mCMD\033[0m \033[94mSEQ\033[0m \033[93mPL_ID\033[0m \033[96mPID\033[0m \033[92mLEN\033[0m \033[97mDATA\033[0m \033[91mCRC\033[0m \033[90mFEND\033[0m")
            print(f"  -> TX Raw Frame: {colorize_raw_frame(req_frame)}")
            ser.write(req_frame)
            
            if dl_active:
                last_request_time = time.time()
            
            highest_seq_received = -1
            packets_received_in_window = 0
        except queue.Empty:
            if dl_active and last_request_time > 0 and (time.time() - last_request_time > 2.0):
                if retry_count < MAX_RETRIES:
                    print(f"     \033[93m[GS] Timeout waiting for offset {dl_offset}. Retrying {retry_count + 1}/{MAX_RETRIES}...\033[0m")
                    req_data = struct.pack('>B', len(dl_filename_bytes)) + dl_filename_bytes + struct.pack('>IH', dl_offset, dl_chunk_size)
                    command_queue.put(('MANUAL', 0x00, 0x03, "Auto-Request Chunk Retry", req_data))
                    last_request_time = time.time()
                    retry_count += 1
                else:
                    print(f"     \033[91m[GS] Max retries reached. Download aborted.\033[0m")
                    dl_active = False
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
                                        if pid == 0x00: # Ping
                                            delay = (time.time() - ping_send_time) * 1000
                                            print(f"     Ping Response from OBC Received! time={delay:.1f}ms")
                                            
                                        elif pid == 0x01: # List Files
                                            num_files = data[0]
                                            print(f"     Found {num_files} files:")
                                            offset = 1
                                            for _ in range(num_files):
                                                name_len = data[offset]
                                                name = data[offset+1 : offset+1+name_len].decode()
                                                print(f"       - {name}")
                                                offset += 1 + name_len
                                                
                                        elif pid == 0x02: # File Info
                                            status, size, ts = struct.unpack('>BII', data)
                                            s_str = "OK" if status == 0 else "Error"
                                            print(f"     Status: {s_str} | Size: {size} bytes | Created: {ts}")
                                            
                                            if dl_active and dl_total_size == 0:
                                                if status == 0x00:
                                                    dl_total_size = size
                                                    dl_start_time = time.time()
                                                    print(f"     [GS] File Size Acquired. Starting transfer of {size} bytes...")
                                                    req_data = struct.pack('>B', len(dl_filename_bytes)) + dl_filename_bytes + struct.pack('>IH', dl_offset, dl_chunk_size)
                                                    command_queue.put(('MANUAL', 0x00, 0x03, "Auto-Request Chunk", req_data))
                                                else:
                                                    print(f"     \033[91m[GS] Target file not found. Auto-download aborted.\033[0m")
                                                    dl_active = False
                                            
                                        elif pid == 0x03: # File Data
                                            status, offset, dl = struct.unpack('>BIH', data[:7])
                                            chunk = data[7:7+dl]
                                            print(f"     Status: {status} | Offset: {offset} | Len: {dl}")
                                            
                                            if status == 0x00 and dl > 0:
                                                filepath = os.path.join(DOWNLOADS_DIR, current_download_file)
                                                mode = 'r+b' if os.path.exists(filepath) and offset > 0 else 'wb'
                                                try:
                                                    with open(filepath, mode) as f:
                                                        f.seek(offset)
                                                        f.write(chunk)
                                                        
                                                    # Provide a visual progress indicator instead of basic text
                                                    if dl_total_size > 0:
                                                        progress = min(1.0, (offset + dl) / dl_total_size)
                                                        bar_len = 30
                                                        filled = int(bar_len * progress)
                                                        bar = '=' * filled + '-' * (bar_len - filled)
                                                        
                                                        elapsed = time.time() - dl_start_time
                                                        speed = (offset + dl) / elapsed if elapsed > 0 else 0
                                                        speed_str = f"{speed / 1024:.1f} KB/s" if speed >= 1024 else f"{(speed):.1f} B/s"
                                                        
                                                        rem_bytes = dl_total_size - (offset + dl)
                                                        eta = rem_bytes / speed if speed > 0 else 0
                                                        
                                                        print(f"     \033[96m[{bar}] {progress*100:.1f}% ({offset+dl}/{dl_total_size} B) | {speed_str} | ETA: {eta:.1f}s\033[0m")
                                                    else:
                                                        print(f"     \033[92m[Saved chunk to {filepath} at offset {offset}]\033[0m")
                                                    
                                                    # Automatic sliding window for the next chunk
                                                    if dl_active and dl == dl_chunk_size:
                                                        dl_offset += dl_chunk_size
                                                        retry_count = 0
                                                        # print(f"     [GS] Auto-Requesting next chunk at offset {dl_offset}...")
                                                        time.sleep(0.05) # critical delay to let AS-32 hardware buffer clear
                                                        req_data = struct.pack('>B', len(dl_filename_bytes)) + dl_filename_bytes + struct.pack('>IH', dl_offset, dl_chunk_size)
                                                        command_queue.put(('MANUAL', 0x00, 0x03, "Auto-Request Chunk", req_data))
                                                    elif dl_active and dl < dl_chunk_size:
                                                        elapsed_time = time.time() - dl_start_time
                                                        avg_speed = dl_total_size / elapsed_time if elapsed_time > 0 and dl_total_size > 0 else 0
                                                        speed_str = f"{avg_speed / 1024:.1f} KB/s" if avg_speed >= 1024 else f"{avg_speed:.1f} B/s"
                                                        print(f"     \033[92m[GS] Download Complete! '{current_download_file}' is fully retrieved.\033[0m")
                                                        print(f"     \033[92m[GS] Total Time: {elapsed_time:.2f}s | Avg Speed: {speed_str}\033[0m")
                                                        dl_active = False

                                                except Exception as e:
                                                    print(f"     \033[91m[Save Error: {e}]\033[0m")
                                                    dl_active = False
                                            else:
                                                if dl_active:
                                                    print(f"     \033[93m[GS] End of file reached or error occurred.\033[0m")
                                                    dl_active = False
                                        
                                        elif pid == 0x04: # EPS Beacon
                                            ret_beacon_dict = decode_beacon_packet(data)
                                            if ret_beacon_dict:
                                                beacon_count += 1
                                                last_beacon_rx_time = time.time()
                                                latest_beacon_data = ret_beacon_dict
                                                
                                                print("") # Force a newline over the GS> prompt
                                                print_decoded_beacon_data(ret_beacon_dict)
                                                print("GS> ", end="", flush=True) # Reprint prompt after jumping
                                            
                                            
                                    elif p_id == 0x01:
                                        if pid == 0x00: # Ping
                                            delay = (time.time() - ping_send_time) * 1000
                                            print(f"     Ping Response from VR Received! time={delay:.1f}ms")
                                            
                                        elif pid == 0x01: # Pi Status
                                            ts, up, cpu_l, cpu_t, ram, disk, cam = struct.unpack('>IIBbBBB3x', data)
                                            cam_str = {0:"Err", 1:"Ready", 2:"Busy"}.get(cam, "Unknown")
                                            print(f"     Pi Status -> Time: {ts} | Up: {up}s | CPU: {cpu_l}% ({cpu_t}C) | RAM: {ram}% | Disk: {disk}% | CAM: {cam_str}")
                                            
                                        elif pid == 0x02: # Capture
                                            status, name_len = struct.unpack('>BB', data[:2])
                                            name = data[2:2+name_len].decode()
                                            print(f"     Capture -> Status: {status} | File: {name}")

                                        elif pid == 0x03: # Copy Image to SD
                                            status = struct.unpack('>B', data[:1])[0]
                                            s_str = "OK" if status == 0 else "Error"
                                            print(f"     Copy to SD -> Status: {s_str} ({status})")

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