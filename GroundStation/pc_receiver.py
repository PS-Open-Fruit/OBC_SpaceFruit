import serial
import threading
import time
import sys
import os
import struct

# Import KISS Protocol
current_dir = os.path.dirname(os.path.abspath(__file__))
shared_path = os.path.join(os.path.dirname(current_dir), 'Shared', 'Python')
if shared_path not in sys.path:
    sys.path.append(shared_path)
try:
    from kiss_protocol import KISSProtocol
except ImportError:
    print("❌ Error: Could not import KISSProtocol.")
    sys.exit(1)

# ==========================================
# 1. Config
# ==========================================
PC_PORT = 'COM9'  # Update if needed
BAUDRATE = 9600   # PRODUCTION BAUDRATE
DOWNLOAD_DIR = "received_images"

if not os.path.exists(DOWNLOAD_DIR):
    os.makedirs(DOWNLOAD_DIR)

# ==========================================
# 2. Ground Station Class
# ==========================================
class GroundStation:
    def __init__(self, port='COM9', baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        
        # Image Transfer State
        self.current_img_data = bytearray()
        self.received_chunk_ids = set() # Track unique chunks
        self.expected_size = 0
        self.downloading = False
        self.start_time = 0
        
        # RX State Machine buffers
        self.log_buffer = bytearray()
        self.kiss_buffer = bytearray()
        self.in_frame = False

    def start(self):
        print(f"Connecting to {self.port} at {self.baud} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2) # Wait for reset
            print("✅ Connected!")
        except Exception as e:
            print(f"❌ Connection Failed: {e}")
            return

        # Start background listener thread
        rx_thread = threading.Thread(target=self.listen_loop, daemon=True)
        rx_thread.start()

        # Start main CLI loop
        self.cli_loop()

    def send_kiss_command(self, cmd_id):
        """Sends a KISS-framed command to the OBC with CRC32."""
        if not self.ser or not self.ser.is_open: 
            print("❌ Serial Closed")
            return
        
        # Construct Payload: [APP_CMD]
        app_payload = bytes([cmd_id])
        
        # Calculate CRC over Payload
        crc = KISSProtocol.calculate_crc(app_payload)
        crc_bytes = struct.pack('<I', crc)
        
        # Full Payload: [APP_CMD] [CRC]
        full_payload = app_payload + crc_bytes
        
        # Wrap in KISS Data Frame (CMD 0x00)
        # Wire: [FEND] [0x00] [APP_CMD] [CRC] [FEND]
        frame = KISSProtocol.wrap_frame(full_payload, KISSProtocol.CMD_DATA)
        
        self.ser.write(frame)
        print(f"   [TX] Sent CMD: 0x{cmd_id:02X} (CRC: {crc:08X})")

    def cli_loop(self):
        """Interactive Command Line Interface."""
        print("\n--- 🛰️ OBC GROUND STATION CLI ---")
        print("Commands: [c]apture, [s]tatus, [p]ing, [q]uit")
        print("---------------------------------------")

        while self.running:
            try:
                cmd = input("GS> ").strip().lower()
                
                if cmd in ['q', 'quit', 'exit']:
                    print("Shutting down...")
                    self.running = False
                    break
                
                elif cmd in ['p', 'ping']:
                    self.send_kiss_command(0x10) # 0x10 = Ping
                    
                elif cmd in ['s', 'status']:
                    self.send_kiss_command(0x20) # 0x20 = Status Request
                    
                elif cmd in ['c', 'capture']:
                    self.send_kiss_command(0x12) # 0x12 = Capture
                    print("   REQUESTED CAPTURE...")

            except KeyboardInterrupt:
                self.running = False
                break

    def listen_loop(self):
        """
        Main RX Loop.
        Parses stream for:
        1. Plain ASCII logs (outside KISS frames)
        2. KISS Frames (Image Data)
        """
        print("🎧 Listening for Telemetry...")
        
        while self.running and self.ser.is_open:
            try:
                # Read 1 byte at a time for precise state machine
                byte = self.ser.read(1)
                
                if not byte:
                    continue
                
                b = byte[0] # Get int value
                
                if b == KISSProtocol.FEND:
                    if self.in_frame:
                        # End of Frame
                        if len(self.kiss_buffer) > 0:
                            self.process_kiss_frame(self.kiss_buffer)
                        self.kiss_buffer = bytearray() # Reset
                        self.in_frame = False
                    else:
                        # Start of Frame
                        self.in_frame = True
                        self.kiss_buffer = bytearray()
                        
                elif self.in_frame:
                    self.kiss_buffer.append(b)
                
                # Ignore raw bytes outside frames (No Raw Text allowed)
            
            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(1)

    def process_kiss_frame(self, escaped_data):
        """Unescapes and handles KISS payload."""
        payload = KISSProtocol.unescape(escaped_data)
        
        # Min size: [CMD:1] [CRC:4] = 5 bytes
        if len(payload) < 5: return 
        
        # 1. Extract Components
        cmd_byte = payload[0]
        data_content = payload[0:-4] # CMD + PAYLOAD (Excluding CRC)
        received_crc_bytes = payload[-4:]
        
        # 2. Extract CRC
        received_crc = struct.unpack('<I', received_crc_bytes)[0]
        
        # 3. Calculate CRC (Over CMD + PAYLOAD)
        calc_crc = KISSProtocol.calculate_crc(data_content)
        
        if calc_crc != received_crc:
            print(f"\n⚠️ CRC ERROR: Calc {calc_crc:08X} != Rx {received_crc:08X} (Cmd: {cmd_byte:02X})")
            return

        # 4. Dispatch
        payload_body = data_content[1:] # Exclude CMD

        # CMD 0x01: Log Message
        if cmd_byte == 0x01:
             text = payload_body.decode('utf-8', errors='replace')
             self.process_log_line(text)
             return

        # CMD 0x12: Capture Response (Start Download)
        if cmd_byte == 0x12:
            # Payload: [ImgID:2] [Size:4]
            if len(payload_body) < 6: return
            img_id = struct.unpack('<H', payload_body[0:2])[0]
            self.expected_size = struct.unpack('<I', payload_body[2:6])[0]
            
            sys.stdout.write("\r")
            
            print(f"   [RX] START IMG #{img_id}, Size: {self.expected_size/1024:.2f} KB")
            
            self.downloading = True
            self.current_img_data = bytearray()
            self.received_chunk_ids = set()
            self.start_time = time.time()
            return

        # CMD 0x00: Image Data (Space Ready Format)
        if cmd_byte == 0x00:
            if len(payload_body) < 2: return # Need ChunkID
            
            # ChunkID (2 bytes)
            chunk_id = payload_body[0] | (payload_body[1] << 8)
            img_chunk = payload_body[2:]
            
            # Deduplication Check
            if chunk_id in self.received_chunk_ids:
                 return # Duplicate

            self.received_chunk_ids.add(chunk_id)
            self.current_img_data.extend(img_chunk)
            self.print_progress()
            return

        print(f"   Rx Unknown CMD: 0x{cmd_byte:02X}")

    def process_log_line(self, text):
        """Parses ASCII log lines."""
        if not text: return
        
        # 1. Detect End of Download
        if "Download Complete!" in text:
            filename = os.path.join(DOWNLOAD_DIR, f"img_{int(time.time())}.jpg")
            with open(filename, "wb") as f:
                f.write(self.current_img_data)
            
            print(f"\n   IMAGE SAVED: {filename}")
            if self.expected_size > 0:
                loss = 100 * (1 - len(self.current_img_data) / self.expected_size)
                if loss > 0: print(f"   ⚠️ Data Loss: {loss:.1f}%")
            
            self.downloading = False
            sys.stdout.write("GS> ")
            sys.stdout.flush()

        # 3. Standard Logs
        else:
            if not self.downloading:
                # Move to start of line to overwrite 'GS> ' if it's there
                sys.stdout.write("\r") 
                
                prefix = "   " 
                
                # Avoid double prefix if OBC already sent it
                if text.startswith("[OBC]") or text.startswith("[VR]"):
                    print(f"{prefix}{text}")
                else:
                    print(f"{prefix}[OBC] {text}")
                
                # Restore Prompt visually so user knows they can type
                # Heuristic: Only show prompt if we are likely done with the sequence
                if "GS CMD:" not in text:
                    sys.stdout.write("GS> ")
                    sys.stdout.flush()

    def print_progress(self):
        """Prints a simple progress bar."""
        if not self.downloading: return
        
        current_len = len(self.current_img_data)
        elapsed = time.time() - self.start_time
        speed = (current_len / 1024) / elapsed if elapsed > 0.5 else 0.0
        
        if self.expected_size > 0:
            pct = (current_len / self.expected_size) * 100.0
            # Simple condensed output to not break CLI input too much
            sys.stdout.write(f"\r   Downloading: {pct:.1f}% | {current_len/1024:.1f} KB | {speed:.1f} KB/s")
            sys.stdout.flush()

if __name__ == "__main__":
    gs = GroundStation(PC_PORT, BAUDRATE)
    gs.start()
