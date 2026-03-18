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
        return ("CRC_ERROR", received_crc, calculated_crc, b"")
    
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
        elif pid == 0x05: desc = "System Status"
        elif pid == 0xAC: desc = "ACK"
    elif p_id == 0x01: # VR
        if pid == 0x00: desc = "Ping"
        elif pid == 0x01: desc = "Pi Status"
        elif pid == 0x02: desc = "Capture"
        elif pid == 0x03: desc = "Copy to SD"
        elif pid == 0x04: desc = "Check Copy"
        elif pid == 0x90: desc = "Shutdown"
        elif pid == 0xAC: desc = "ACK"
        
    return f"{source} -> {dest} | {subsystem} {desc}"

def _parse_len_prefixed_filename(data: bytes):
    if len(data) < 1:
        return None, "Missing filename length byte"
    name_len = data[0]
    if len(data) < 1 + name_len:
        return None, f"Filename truncated: expected {name_len}B, got {len(data)-1}B"
    name_raw = data[1:1+name_len]
    try:
        name = name_raw.decode('utf-8')
    except Exception:
        name = name_raw.decode('utf-8', errors='replace')
    remainder = data[1+name_len:]
    return (name, remainder), None

def _format_beacon_lines(decoded_data):
    if not decoded_data:
        return ["[Beacon decode failed]"]

    lines = ["EPS SENSOR DATA:"]

    for idx, vi in enumerate(decoded_data["eps"]["vi_sensors"]):
        lines.append(
            f"VI Sensor {idx:<1}   | V: {vi['voltage']} mV, I: {vi['current']} mA, Ch: {vi['channel']}, State: {vi['data_state']}"
        )

    for idx, out in enumerate(decoded_data["eps"]["output_sensors"]):
        lines.append(
            f"Out Sensor {idx:<1}  | V: {out['voltage']} mV, I: {out['current']} mA, Ch: {out['channel']}, State: {out['data_state']}"
        )

    for idx, out_state in enumerate(decoded_data["eps"]["output_states"]):
        lines.append(
            f"Out State {idx:<1}   | Status: {out_state['status']}, Ch: {out_state['channel']}, State: {out_state['data_state']}"
        )

    for idx, temp in enumerate(decoded_data["eps"]["battery_temps"]):
        lines.append(
            f"Batt Temp {idx:<1}   | Temp: {temp['temperature']:.2f} °C, Ch: {temp['channel']}, State: {temp['data_state']}"
        )

    lines.append("RTC DATETIME:")
    rtc = decoded_data["rtc"]
    lines.append(
        f"20{rtc['year']:02d}-{rtc['month']:02d}-{rtc['day']:02d} {rtc['hour']:02d}:{rtc['min']:02d}:{rtc['sec']:02d} (WDay: {rtc['wday']})"
    )

    lines.append("TMP1075 SENSOR:")
    lines.append(f"Raw Temp Value: {decoded_data['tmp1075']['raw_temp']}")
    return lines

def decode_layer3_data(cmd, p_id, pid, data: bytes):
    lines = []

    if cmd == 0x00:
        lines.append("Layer3: Request")
        if p_id == 0x00:  # OBC requests
            if pid in (0x00, 0x01, 0x04, 0x05):
                lines.append(f"Data: empty ({len(data)}B)")
            elif pid == 0x02:  # File Info request
                parsed, err = _parse_len_prefixed_filename(data)
                if err:
                    lines.append(f"Decode warning: {err}")
                else:
                    if parsed is None:
                        lines.append("Decode warning: filename parse failed")
                        return lines
                    name, rem = parsed
                    lines.append(f"Filename: {name}")
                    if rem:
                        lines.append(f"Trailing bytes: {rem.hex(' ').upper()}")
            elif pid == 0x03:  # File Data request
                parsed, err = _parse_len_prefixed_filename(data)
                if err:
                    lines.append(f"Decode warning: {err}")
                else:
                    if parsed is None:
                        lines.append("Decode warning: filename parse failed")
                        return lines
                    name, rem = parsed
                    if len(rem) < 6:
                        lines.append(f"Filename: {name}")
                        lines.append(f"Decode warning: missing offset/chunk fields (need 6B, got {len(rem)}B)")
                    else:
                        offset, chunk_len = struct.unpack('>IH', rem[:6])
                        lines.append(f"Filename: {name}")
                        lines.append(f"Offset: {offset} | ChunkLen: {chunk_len}")
                        if len(rem) > 6:
                            lines.append(f"Trailing bytes: {rem[6:].hex(' ').upper()}")
            else:
                if data:
                    lines.append(f"Raw Data: {data.hex(' ').upper()}")

        elif p_id == 0x01:  # VR requests
            if pid in (0x00, 0x01, 0x02, 0x03, 0x04, 0x90):
                lines.append(f"Data: empty ({len(data)}B)")
            else:
                if data:
                    lines.append(f"Raw Data: {data.hex(' ').upper()}")

        return lines

    if cmd == 0x01:
        lines.append("Layer3: Data/Response")
        if p_id == 0x00:  # OBC responses
            if pid == 0x00:  # Ping
                lines.append(f"Ping ACK payload length: {len(data)}B")

            elif pid == 0x01:  # List files
                if len(data) < 1:
                    lines.append("Decode warning: missing file count byte")
                else:
                    num_files = data[0]
                    lines.append(f"File count: {num_files}")
                    offset = 1
                    for i in range(num_files):
                        if offset >= len(data):
                            lines.append(f"Decode warning: truncated at file index {i}")
                            break
                        name_len = data[offset]
                        start = offset + 1
                        end = start + name_len
                        if end > len(data):
                            lines.append(f"Decode warning: file name #{i+1} truncated")
                            break
                        name_raw = data[start:end]
                        try:
                            name = name_raw.decode('utf-8')
                        except Exception:
                            name = name_raw.decode('utf-8', errors='replace')
                        lines.append(f"  [{i+1}] {name}")
                        offset = end
                    if offset < len(data):
                        lines.append(f"Trailing bytes: {data[offset:].hex(' ').upper()}")

            elif pid == 0x02:  # File info response
                if len(data) < struct.calcsize('>BII'):
                    lines.append(f"Decode warning: FileInfo needs 9B, got {len(data)}B")
                else:
                    status, size, ts = struct.unpack('>BII', data[:9])
                    status_str = {0: "OK", 1: "Error"}.get(status, f"0x{status:02X}")
                    lines.append(f"Status: {status_str} ({status}) | Size: {size} B | Created: {ts}")
                    if len(data) > 9:
                        lines.append(f"Trailing bytes: {data[9:].hex(' ').upper()}")

            elif pid == 0x03:  # File data chunk
                if len(data) < struct.calcsize('>BIH'):
                    lines.append(f"Decode warning: FileData header needs 7B, got {len(data)}B")
                else:
                    status, offset, dl = struct.unpack('>BIH', data[:7])
                    chunk = data[7:7+dl]
                    lines.append(f"Status: {status} | Offset: {offset} | ChunkLen: {dl}")
                    lines.append(f"Chunk bytes available: {len(chunk)}")
                    if len(chunk) != dl:
                        lines.append(f"Decode warning: chunk length mismatch (header {dl}, actual {len(chunk)})")
                    if len(data) > 7 + len(chunk):
                        lines.append(f"Trailing bytes: {data[7+len(chunk):].hex(' ').upper()}")

            elif pid == 0x04:  # Beacon
                lines.append("Beacon decoded below:")

            elif pid == 0x05:  # System status
                fmt = '>IBBIIIBiIIBBBB'
                expected = struct.calcsize(fmt)
                if len(data) != expected:
                    lines.append(f"Decode warning: SystemStatus needs {expected}B, got {len(data)}B")
                else:
                    (
                        obc_boot_count,
                        usb_bus_status,
                        eps_status,
                        payload_boot_count,
                        payload_timestamp,
                        payload_uptime,
                        payload_cpu_load,
                        payload_cpu_temp_milli,
                        payload_ram_mb,
                        payload_disk_mb,
                        payload_camera_status,
                        payload_throttled,
                        payload_file_count,
                        payload_load_avg,
                    ) = struct.unpack(fmt, data)

                    usb_str = "OK" if usb_bus_status == 0x00 else "Busy"
                    eps_str = "OK" if eps_status == 0x00 else "No Response"
                    cam_str = {0: "Err", 1: "Ready", 2: "Busy"}.get(payload_camera_status, "Unknown")
                    lines.append(f"OBC Boot: {obc_boot_count} | USB: {usb_str} ({usb_bus_status}) | EPS: {eps_str} ({eps_status})")
                    lines.append(f"Payload Boot: {payload_boot_count} | Time: {payload_timestamp} | Uptime: {payload_uptime}s")
                    lines.append(
                        f"CPU: {payload_cpu_load}% ({payload_cpu_temp_milli/1000.0:.3f}C) | RAM: {payload_ram_mb}MB | Disk: {payload_disk_mb}MB"
                    )
                    lines.append(
                        f"CAM: {cam_str} ({payload_camera_status}) | Throttled: {payload_throttled} | Files: {payload_file_count} | LoadAvg: {payload_load_avg}"
                    )

            elif pid == 0xAC:
                lines.append(f"ACK payload length: {len(data)}B")

            else:
                if data:
                    lines.append(f"Raw Data: {data.hex(' ').upper()}")

        elif p_id == 0x01:  # VR responses
            if pid == 0x00:  # Ping
                lines.append(f"Ping ACK payload length: {len(data)}B")

            elif pid == 0x01:  # Pi status
                expected = struct.calcsize('>IIBbBBB3x')
                if len(data) != expected:
                    lines.append(f"Decode warning: PiStatus needs {expected}B, got {len(data)}B")
                else:
                    ts, up, cpu_l, cpu_t, ram, disk, cam = struct.unpack('>IIBbBBB3x', data)
                    cam_str = {0: "Err", 1: "Ready", 2: "Busy"}.get(cam, "Unknown")
                    lines.append(f"Time: {ts} | Uptime: {up}s")
                    lines.append(f"CPU: {cpu_l}% ({cpu_t}C) | RAM: {ram}% | Disk: {disk}% | CAM: {cam_str}")

            elif pid == 0x02:  # Capture response
                if len(data) < 2:
                    lines.append(f"Decode warning: Capture response needs >=2B, got {len(data)}B")
                else:
                    status, name_len = struct.unpack('>BB', data[:2])
                    name_raw = data[2:2+name_len]
                    try:
                        name = name_raw.decode('utf-8')
                    except Exception:
                        name = name_raw.decode('utf-8', errors='replace')
                    lines.append(f"Status: {status} | File: {name}")
                    if len(name_raw) != name_len:
                        lines.append(f"Decode warning: filename length mismatch (header {name_len}, actual {len(name_raw)})")

            elif pid == 0x03:  # Copy to SD response
                if len(data) < 1:
                    lines.append("Decode warning: CopyToSD response missing status byte")
                else:
                    status = data[0]
                    status_str = "OK" if status == 0 else "Error"
                    lines.append(f"CopyToSD Status: {status_str} ({status})")

            elif pid == 0x04:  # Check copy response
                if len(data) < 1:
                    lines.append("Decode warning: CheckCopy response missing status byte")
                else:
                    status = data[0]
                    status_str = {0: "Idle", 1: "Copying", 2: "Error"}.get(status, f"Unknown({status})")
                    lines.append(f"CheckCopy Status: {status_str}")

            elif pid == 0x90:  # Shutdown ack
                lines.append(f"Shutdown ACK payload length: {len(data)}B")

            elif pid == 0xAC:
                lines.append(f"ACK payload length: {len(data)}B")

            else:
                if data:
                    lines.append(f"Raw Data: {data.hex(' ').upper()}")

    return lines

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
                                    decoded_lines = decode_layer3_data(cmd, p_id, pid, data)
                                    for line in decoded_lines:
                                        print(f"      {line}")

                                    if cmd == 0x01 and p_id == 0x00 and pid == 0x04:
                                        beacon = decode_beacon_packet(data)
                                        if beacon:
                                            for beacon_line in _format_beacon_lines(beacon):
                                                print(f"      {beacon_line}")
                                        else:
                                            print(f"      \033[90m[Beacon decode failed: len={len(data)}]\033[0m")
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
