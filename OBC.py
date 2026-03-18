import serial
import struct
import os

SD_CARD_DIR = "sd_card"
os.makedirs(SD_CARD_DIR, exist_ok=True)
PI_FILES_DIR = "pi_files"
os.makedirs(PI_FILES_DIR, exist_ok=True)
import time
import random
import sys
from Shared.Python.kiss_protocol import KISSProtocol

import argparse

# --- CONFIGURATION ---
_PORTS = {
    'darwin': '/dev/cu.usbserial-XXXXXXXX',
    'win32':  'COM5',
    'linux':  '/dev/ttyUSB1',
}
DEFAULT_PORT = _PORTS.get(sys.platform, _PORTS['win32'])
DEFAULT_BAUD = 9600

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

def dec_to_bcd(val):
    return ((val // 10) << 4) | (val % 10)

def generate_dummy_beacon_data() -> bytes:
    b = bytearray()
    
    # 8 VI Sensors ('>hhBB') = 6 bytes each
    for i in range(8):
        v = 4000 + random.randint(-200, 200)
        c = 500 + random.randint(-100, 100)
        b += struct.pack('>hhBB', v, c, i, 2)
        
    # 6 Output Sensors ('>hhBB') = 6 bytes each
    for i in range(6):
        if i == 2:
            b += struct.pack('>hhBB', 256, 256, i, 2)
        elif i == 3:
            b += struct.pack('>hhBB', 0, 512, i, 2)
        else:
            v = random.randint(0, 50)
            c = random.randint(0, 10)
            b += struct.pack('>hhBB', v, c, i, 2)
            
    # 6 Output States ('>BBB') = 3 bytes each
    for i in range(6):
        b += struct.pack('>BBB', 0 if i not in (1,2) else 4, i, 2)
        
    # 2 Battery Temps ('>hBB') = 4 bytes each
    for i in range(2):
        t = int(25.5 * 100) + random.randint(-200, 200)
        b += struct.pack('>hBB', t, i, 2)
        
    # RTC (7 Bytes BCD)
    now = time.localtime()
    sec_bcd = dec_to_bcd(now.tm_sec)
    min_bcd = dec_to_bcd(now.tm_min)
    hour_bcd = dec_to_bcd(now.tm_hour)
    wday = now.tm_wday + 1 if now.tm_wday < 6 else 1 # Python wday is 0-6 (Mon-Sun), Target is usually 1-7.
    day_bcd = dec_to_bcd(now.tm_mday)
    mon_bcd = dec_to_bcd(now.tm_mon)
    year_bcd = dec_to_bcd(now.tm_year % 100)
    
    b += struct.pack('>BBBBBBB', sec_bcd, min_bcd, hour_bcd, wday, day_bcd, mon_bcd, year_bcd)
    
    # TMP1075 (4 Bytes int32)
    b += struct.pack('>i', 280000)
    
    return bytes(b)

def get_dummy_pi_status_snapshot():
    """Emulates OBC polling VR payload status (internal PID 0x01 request)."""
    now_ts = int(time.time())
    try:
        file_count = sum(1 for f in os.listdir(PI_FILES_DIR) if os.path.isfile(os.path.join(PI_FILES_DIR, f)))
    except Exception:
        file_count = 0

    return {
        "boot_count": 3,
        "timestamp": now_ts,
        "uptime": 3600,
        "cpu_load": 15,
        "cpu_temp_milli": 45250,  # fixed-point x1000
        "ram_usage_mb": 512,
        "disk_usage_mb": 2048,
        "camera_status": 0x01,
        "throttled": 0,
        "file_count": min(file_count, 255),
        "load_avg": 8,
    }

def main():
    parser = argparse.ArgumentParser(description="OBC Emulator")
    parser.add_argument("--port", type=str, default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--beacon", action="store_true", help="Enable autonomous EPS beaconing")
    parser.add_argument("--interval", type=float, default=5.0, help="Beacon interval in seconds (default: 5.0)")
    args = parser.parse_args()

    port = args.port
    baud = args.baud

    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"OBC Started on {port} at {baud} baud. Waiting for Ground Station requests...")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    seq_counter = 0
    
    # --- BEACON CONFIG ---
    BEACON_ENABLED = args.beacon
    last_beacon_time = time.time()
    beacon_interval = args.interval  # seconds
    
    while True:
        # --- BEACON BROADCAST ---
        if BEACON_ENABLED and (time.time() - last_beacon_time >= beacon_interval):
            last_beacon_time = time.time()
            beacon_data = generate_dummy_beacon_data()
            seq_counter = (seq_counter + 1) % 256
            
            # PayloadID: 0x00 (OBC), PID: 0x04 (Beacon)
            beacon_payload = build_custom_payload(OBC_SUBSYSTEM, 0x04, seq_counter, beacon_data)
            
            # Link Level Command 0x01 (Data/Response)
            tx_frame = KISSProtocol.wrap_frame(beacon_payload, command=0x01)
            
            print(f"\n[OBC] Broadcasting EPS Beacon (121 bytes)...")
            print(f"  -> TX Raw Frame: {colorize_raw_frame(tx_frame)}")
            ser.write(tx_frame)

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
                                    elif pid == 0x05: print("  -> Action: Request System Status (OBC -> VR Poll)")
                                    else: print("  -> Action: Unknown OBC Command")
                                        
                                elif p_id == VR_SUBSYSTEM:
                                    print("  -> Routing to: VR Payload Subsystem")
                                    if pid == 0x00: print("  -> Action: Request Ping")
                                    elif pid == 0x01: print("  -> Action: Request Pi Status")
                                    elif pid == 0x02: print("  -> Action: Request Capture")
                                    elif pid == 0x03: print("  -> Action: Request Copy Image to SD")
                                    elif pid == 0x90: print("  -> Action: Request VR Shutdown (DANGEROUS)")
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
                                            
                                    elif pid == 0x03: # File Data (Chunk) — GS drives each window via a new request
                                        if len(req_data) >= 1:
                                            name_len = req_data[0]
                                            fname = req_data[1:1+name_len].decode('utf-8')
                                            fpath = os.path.join(SD_CARD_DIR, fname)
                                            if len(req_data) >= 1 + name_len + 6:
                                                offset, r_len = struct.unpack('>IH', req_data[1+name_len : 1+name_len+6])
                                                filesize = os.path.getsize(fpath) if os.path.exists(fpath) else 0
                                                print(f"  -> Streaming window: file='{fname}' offset={offset} chunk={r_len} filesize={filesize}")
                                                
                                                cur_offset = offset
                                                for _ in range(5):  # DOWNLINK_WINDOW_SIZE = 5
                                                    if cur_offset >= filesize:
                                                        break
                                                    chunk = b''
                                                    try:
                                                        with open(fpath, 'rb') as f:
                                                            f.seek(cur_offset)
                                                            chunk = f.read(r_len)
                                                        chunk_data = struct.pack('>BIH', 0x00, cur_offset, len(chunk)) + chunk
                                                    except Exception as e:
                                                        print(f"  -> Read error: {e}")
                                                        chunk_data = struct.pack('>BIH', 0x01, cur_offset, 0)
                                                    seq_counter = (seq_counter + 1) % 256
                                                    custom_payload = build_custom_payload(p_id, pid, seq_counter, chunk_data)
                                                    tx_frame = KISSProtocol.wrap_frame(custom_payload, command=0x01)
                                                    print(f"  -> TX Raw Frame (offset {cur_offset}): {colorize_raw_frame(tx_frame)}")
                                                    ser.write(tx_frame)
                                                    time.sleep(0.05)
                                                    cur_offset += len(chunk)
                                                    if len(chunk) < r_len:
                                                        break  # EOF reached
                                                dummy_data = None  # already sent above
                                            else:
                                                dummy_data = struct.pack('>BIH', 0x01, 0, 0)
                                        else:
                                            dummy_data = struct.pack('>BIH', 0x01, 0, 0)

                                    elif pid == 0x05: # System Status (GS->OBC, OBC internally polls VR status PID 0x01)
                                        print("  -> Internal Poll: OBC -> VR Request Pi Status (PID 0x01)")
                                        pi_status = get_dummy_pi_status_snapshot()

                                        obc_boot_count = 12
                                        usb_bus_status = 0x00  # 0x00 = OK, 0x01 = Busy
                                        eps_status = 0x00      # 0x00 = OK, 0x01 = No Response

                                        dummy_data = struct.pack(
                                            '>IBBIIIBiIIBBBB',
                                            obc_boot_count,
                                            usb_bus_status,
                                            eps_status,
                                            pi_status["boot_count"],
                                            pi_status["timestamp"],
                                            pi_status["uptime"],
                                            pi_status["cpu_load"],
                                            pi_status["cpu_temp_milli"],
                                            pi_status["ram_usage_mb"],
                                            pi_status["disk_usage_mb"],
                                            pi_status["camera_status"],
                                            pi_status["throttled"],
                                            pi_status["file_count"],
                                            pi_status["load_avg"],
                                        )

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
                                        
                                    elif pid == 0x03: # Copy Image to SD
                                        filename = b"all_files"
                                        try:
                                            import shutil
                                            # Copy all files from PI_FILES_DIR to SD_CARD_DIR
                                            files_copied = 0
                                            for f in os.listdir(PI_FILES_DIR):
                                                src = os.path.join(PI_FILES_DIR, f)
                                                if os.path.isfile(src):
                                                    dst = os.path.join(SD_CARD_DIR, f)
                                                    shutil.copy2(src, dst)
                                                    files_copied += 1
                                            
                                            print(f"  -> Emulated Copy: Copied {files_copied} files from '{PI_FILES_DIR}' to '{SD_CARD_DIR}'")
                                            # Status 0x00 = OK
                                            dummy_data = struct.pack('>B', 0x00)
                                        except Exception as e:
                                            print(f"  -> Emulated Copy Error: {e}")
                                            # Status 0x01 = Error
                                            dummy_data = struct.pack('>B', 0x01)

                                    elif pid == 0x90: # Shutdown VR Pi (dangerous command, 0x9X range)
                                        print("  -> Emulated VR Shutdown: Sending ACK (real Pi would halt after this)")
                                        dummy_data = b''  # Empty payload — mirrors the ACK the real Pi sends
                                            
                                        
                                # Send response (Command 0x01) for non-streaming commands
                                if dummy_data is not None:
                                    custom_payload = build_custom_payload(p_id, pid, seq_counter, dummy_data)
                                    tx_frame = KISSProtocol.wrap_frame(custom_payload, command=0x01)
                                    print(f"  -> TX Raw Frame: {colorize_raw_frame(tx_frame)}")
                                    ser.write(tx_frame)
                                    
                            # ACK (cmd 0xAC) is no longer used — GS drives windows via new requests
                                            
                # Reset buffer and start with FEND for the next potential frame
                rx_buffer = bytearray(FEND_BYTE)
            else:
                # Only append if we've initialized the buffer with a FEND
                if len(rx_buffer) > 0:
                    rx_buffer += byte

if __name__ == '__main__':
    main()
