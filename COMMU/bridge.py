import serial
import threading
import time
import struct
import sys
import os

# --- PATH SETUP ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../Shared/Python')))
try:
    from kiss_protocol import KISSProtocol
except ImportError:
    print("⚠️ Error: Could not import KISSProtocol. Check path.")
    sys.exit(1)

# --- CONFIGURATION ---
PORT_RF  = '/dev/ttyACM0'  # RF433 Module
BAUD_RF  = 9600
PORT_OBC = '/dev/ttyACM1'  # STM32 Nucleo
BAUD_OBC = 9600

class KissDecoder:
    def __init__(self, name):
        self.name = name
        self.buffer = bytearray()

    def process_chunk(self, data):
        """Accumulates data and returns list of decoded frame strings."""
        self.buffer.extend(data)
        decoded_logs = []

        # Process buffer as long as we have a full frame (ends with FEND)
        while KISSProtocol.FEND in self.buffer:
            # Find the first FEND (End of current frame)
            try:
                fend_index = self.buffer.index(KISSProtocol.FEND)
            except ValueError:
                break
            
            # Extract the raw content (still escaped, excluding the end FEND)
            # We assume the start FEND was handled by the previous split or implied stream
            frame_content = self.buffer[:fend_index]
            self.buffer = self.buffer[fend_index+1:] # Remove processed frame

            # If empty frame (back-to-back FENDs), ignore
            if len(frame_content) == 0:
                continue

            # Reconstruct a full KISS Frame for the Library [FEND, ...content..., FEND]
            # The library unwrap_frame expects the bounding FENDs
            full_frame = bytearray([KISSProtocol.FEND]) + frame_content + bytearray([KISSProtocol.FEND])
            
            result = KISSProtocol.unwrap_frame(full_frame)
            if not result:
                # If unwrap failed, it might be runt or garbage
                decoded_logs.append(f"{self.name} [ERR] Decode Fail")
                continue

            cmd_byte, payload = result

            # Parse Payload: [Data...] + [CRC:4]
            # Convention: CRC is calculated on [CMD] + [Payload] (excluding CRC itself)
            
            if len(payload) < 4:
                decoded_logs.append(f"{self.name} [ERR] Payload too short")
                continue

            data_content = payload[:-4]
            crc_received_bytes = payload[-4:]
            
            crc_rx = struct.unpack("<I", crc_received_bytes)[0]
            
            # Reconstruct the data that was CRC'd: [CMD BYTE] + [DATA CONTENT]
            # Checked against OBC: It includes the Command Byte in CRC!
            data_to_check = bytearray([cmd_byte]) + data_content
            
            crc_calc = KISSProtocol.calculate_crc(data_to_check)
            
            crc_status = "✅ OK" if crc_rx == crc_calc else f"❌ BAD (Calc {crc_calc:08X})"

            # Identify Content
            info = ""
            if cmd_byte == 0x10:
                info = "COMMAND: PING"
            elif cmd_byte == 0x01: # Log Message
                try:
                    info = f"LOG: {data_content.decode('utf-8', errors='ignore')}"
                except:
                    info = "LOG: (Binary)"
            elif cmd_byte == 0x00: # Data
                 # Generic ASCII peek
                clean_ascii = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in data_content)
                info = f"DATA: {clean_ascii}"
            else:
                 info = f"CMD {cmd_byte:02X}"

            decoded_logs.append(f"   >>> {self.name} DECODE: {info} | CRC: {crc_status}")

        return decoded_logs

# --- BRIDGE LOGIC ---
def bridge_worker(ser_source, ser_dest, color_code, label, decoder):
    while True:
        try:
            if ser_source.in_waiting > 0:
                data = ser_source.read(ser_source.in_waiting)
                
                # 1. Forward Immediately (Low Latency)
                ser_dest.write(data)
                ser_dest.flush()

                # 2. Print Raw Hex
                hex_str = ' '.join(f'{b:02X}' for b in data)
                print(f"{color_code}[{label}] {hex_str}\033[0m")

                # 3. Decode & Print Analysis
                logs = decoder.process_chunk(data)
                for log in logs:
                    print(f"\033[93m{log}\033[0m") # Yellow for decoder info

            time.sleep(0.005)
        except OSError:
            break

# --- MAIN ---
try:
    print("--- SMART SERIAL BRIDGE STARTED ---")
    print(f"Connecting {PORT_RF} <--> {PORT_OBC}")
    
    rf = serial.Serial(PORT_RF, BAUD_RF, timeout=0.1)
    obc = serial.Serial(PORT_OBC, BAUD_OBC, timeout=0.1)

    dec_rf = KissDecoder("RF->OBC")
    dec_obc = KissDecoder("OBC->RF")

    # Colors: Green for Uplink, Cyan for Downlink
    t1 = threading.Thread(target=bridge_worker, args=(rf, obc, "\033[92m", "RF -> OBC", dec_rf))
    t2 = threading.Thread(target=bridge_worker, args=(obc, rf, "\033[96m", "OBC -> RF", dec_obc))
    
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

    while True: time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping...")
