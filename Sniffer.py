import serial
import struct
import sys
import argparse
from datetime import datetime

# Import Shared project components
from Shared.Python.beacon_helper import *
from Shared.Python.kiss_protocol import KISSProtocol

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

def parse_custom_payload(payload_bytes: bytes):
    if len(payload_bytes) < 9: 
        return None
    content = payload_bytes[:-4]
    received_crc = struct.unpack('>I', payload_bytes[-4:])[0]
    calculated_crc = KISSProtocol.calculate_crc(content)
    if received_crc != calculated_crc:
        return ("CRC_ERROR", received_crc, calculated_crc)
    
    seq_num, payload_id, pid, data_len = struct.unpack('>BBBH', content[:5])
    data = content[5:5+data_len]
    return payload_id, pid, seq_num, data

def get_packet_description(cmd, p_id, pid):
    source = "GS" if cmd == 0x00 else "OBC"
    dest = "OBC" if cmd == 0x00 else "GS"
    
    subsystem = "OBC" if p_id == 0x00 else ("VR" if p_id == 0x01 else f"0x{p_id:02X}")
    
    desc = "Unknown"
    if p_id == 0x00: # OBC
        if pid == 0x00: desc = "Ping"
        elif pid == 0x01: desc = "File List"
        elif pid == 0x02: desc = "File Info"
        elif pid == 0x03: desc = "File Data"
        elif pid == 0x04: desc = "Beacon"
    elif p_id == 0x01: # VR
        if pid == 0x00: desc = "Ping"
        elif pid == 0x01: desc = "Pi Status"
        elif pid == 0x02: desc = "Capture"
        elif pid == 0x03: desc = "Copy to SD"
        elif pid == 0x90: desc = "Shutdown"
        
    return f"{source} -> {dest} | {subsystem} {desc}"

def main():
    _PORTS = {
        'darwin': '/dev/cu.usbserial-A10OMHTZ',
        'win32':  'COM4',
        'linux':  '/dev/ttyUSB0',
    }
    DEFAULT_PORT = _PORTS.get(sys.platform, _PORTS['win32'])
    
    parser = argparse.ArgumentParser(description="SpaceFruit Traffic Sniffer (Receive-Only)")
    parser.add_argument("--port", type=str, default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"\033[91mError opening serial port {args.port}: {e}\033[0m")
        sys.exit(1)

    print(f"\033[1;36m--- SpaceFruit Traffic Sniffer Started ---\033[0m")
    print(f"Monitoring {args.port} at {args.baud} baud...")
    print(f"Press Ctrl+C to stop.\n")
    print("\033[90mLegend: FEND CMD SEQ PL_ID PID LEN DATA CRC FEND\033[0m\n")

    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()

    try:
        while True:
            byte = ser.read(1)
            if byte:
                if byte == FEND_BYTE:
                    rx_buffer += byte
                    if len(rx_buffer) >= 3:
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        unwrapped = KISSProtocol.unwrap_frame(bytes(rx_buffer))
                        
                        if unwrapped:
                            cmd, payload_bytes = unwrapped
                            parsed = parse_custom_payload(payload_bytes)
                            
                            print(f"[{timestamp}] \033[1;37mRAW:\033[0m {colorize_raw_frame(rx_buffer)}")
                            
                            if isinstance(parsed, tuple) and parsed[0] == "CRC_ERROR":
                                print(f"    \033[91m[!] CRC Error: Received 0x{parsed[1]:08X}, Calculated 0x{parsed[2]:08X}\033[0m")
                            elif parsed:
                                p_id, pid, seq, data = parsed
                                desc = get_packet_description(cmd, p_id, pid)
                                print(f"    \033[1;32m{desc}\033[0m (Seq: {seq}, DataLen: {len(data)})")
                                
                                # Deep decode for specific packets
                                try:
                                    if p_id == 0x00: # OBC
                                        if pid == 0x01 and cmd == 0x01: # List Files Response
                                            num_files = data[0]
                                            print(f"      Count: {num_files}")
                                        elif pid == 0x02 and cmd == 0x01: # File Info Response
                                            status, size, ts = struct.unpack('>BII', data)
                                            print(f"      Status: {status} | Size: {size} B | Time: {ts}")
                                        elif pid == 0x03: # File Data
                                            status, offset, dl = struct.unpack('>BIH', data[:7])
                                            print(f"      Status: {status} | Offset: {offset} | Len: {dl}")
                                        elif pid == 0x04: # Beacon
                                            beacon = decode_beacon_packet(data)
                                            if beacon:
                                                print(f"      Beacon: V_Batt={beacon.get('v_batt',0)}V, I_Batt={beacon.get('i_batt',0)}mA, Temp={beacon.get('temp_eps',0)}C")
                                    elif p_id == 0x01: # VR
                                        if pid == 0x01 and cmd == 0x01: # Pi Status
                                            ts, up, cpu_l, cpu_t, ram, disk, cam = struct.unpack('>IIBbBBB3x', data)
                                            cam_str = {0:"Err", 1:"Ready", 2:"Busy"}.get(cam, "Unknown")
                                            print(f"      Status -> CPU: {cpu_l}% ({cpu_t}C) | RAM: {ram}% | CAM: {cam_str}")
                                except Exception as e:
                                    print(f"      \033[90m[Decoding failed: {e}]\033[0m")
                            else:
                                print(f"    \033[93m[?] Known KISS frame, unknown custom payload\033[0m")
                            
                            print("-" * 80)
                        else:
                            # Malformed KISS
                            print(f"[{timestamp}] \033[91m[!] Malformed KISS Frame:\033[0m {rx_buffer.hex(' ').upper()}")
                            print("-" * 80)
                            
                        sys.stdout.flush()
                    
                    rx_buffer = bytearray(FEND_BYTE)
                else:
                    if len(rx_buffer) > 0:
                        rx_buffer += byte
    except KeyboardInterrupt:
        print(f"\n\033[1;33mSniffer stopped.\033[0m")
    except Exception as e:
        print(f"\033[91m\nSniffer Error: {e}\033[0m")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
