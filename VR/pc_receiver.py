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

    def send_telecommand(self, cmd_str):
        """Sends a command string/char to the OBC."""
        if not self.ser or not self.ser.is_open: 
            print("❌ Serial Closed")
            return
        
        # Send as bytes (UTF-8)
        self.ser.write(cmd_str.encode('utf-8'))
        print(f"   [TX] Sent: '{cmd_str}'")

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
                    self.send_telecommand('p')
                    
                elif cmd in ['s', 'status']:
                    self.send_telecommand('s')
                    
                elif cmd in ['c', 'capture']:
                    self.send_telecommand('c')
                    print("REQUESTED CAPTURE...")

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
                    
                else:
                    # Not in frame -> ASCII Log processing
                    if b == 0x0A: # Newline
                         text = self.log_buffer.decode('utf-8', errors='ignore').strip()
                         self.process_log_line(text)
                         self.log_buffer = bytearray()
                    elif b >= 0x20 or b == 0x0D: # Printable or CR
                         # Ignore CR, append printable
                         if b != 0x0D:
                            self.log_buffer.append(b)
            
            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(1)

    def process_kiss_frame(self, escaped_data):
        """Unescapes and handles KISS payload."""
        payload = KISSProtocol.unescape(escaped_data)
        
        # Space Ready Format (CRC-32): [CMD:1] [ChunkID:2] [Data:N] [CRC:4]
        # Min size = 1 + 2 + 0 + 4 = 7 bytes
        if len(payload) < 7: return 
        
        cmd_byte = payload[0]
        if cmd_byte != 0x00: return # Only handle commands

        # Payload (excl cmd) is: [ChunkID + Data + CRC]
        payload_no_cmd = payload[1:]
        data_to_check = payload_no_cmd[:-4] # Exclude CRC-32 (last 4 bytes)
        received_crc_bytes = payload_no_cmd[-4:]
        
        # Decode CRC-32 (Little Endian)
        received_crc = struct.unpack('<I', received_crc_bytes)[0]
        
        # Calculate Local CRC
        calc_crc = KISSProtocol.calculate_crc(data_to_check)
        
        if calc_crc != received_crc:
            print(f"⚠️ CRC ERROR: Calc {calc_crc:08X} != Rx {received_crc:08X}")
            return
            
        # Extract Data
        # ChunkID (2 bytes)
        chunk_id = data_to_check[0] | (data_to_check[1] << 8)
        
        # Deduplication Check
        if chunk_id in self.received_chunk_ids:
             # We already have this chunk. Ignore to prevent >100% size or corruption.
             return

        self.received_chunk_ids.add(chunk_id)
        
        img_chunk = data_to_check[2:]
        
        # Append
        self.current_img_data.extend(img_chunk)
        self.print_progress()

    def process_log_line(self, text):
        """Parses ASCII log lines."""
        if not text: return
        
        # 1. Detect Start of Download
        if "[OBC] Image Captured! Size:" in text:
            try:
                parts = text.split("Size: ")[1].split(" ")
                self.expected_size = int(parts[0])
                print(f"\n📥 INCOMING IMAGE: {self.expected_size/1024:.2f} KB")
                
                self.downloading = True
                self.current_img_data = bytearray()
                self.received_chunk_ids = set() # Reset tracker
                self.start_time = time.time()
            except:
                print(f"⚠️ Header Parse Error: {text}")

        # 2. Detect End of Download
        elif "[OBC] Download Complete!" in text:
            filename = os.path.join(DOWNLOAD_DIR, f"img_{int(time.time())}.jpg")
            with open(filename, "wb") as f:
                f.write(self.current_img_data)
            
            print(f"\n✅ IMAGE SAVED: {filename}")
            if self.expected_size > 0:
                loss = 100 * (1 - len(self.current_img_data) / self.expected_size)
                if loss > 0: print(f"   ⚠️ Data Loss: {loss:.1f}%")
            
            self.downloading = False

        # 3. Standard Logs
        else:
            if not self.downloading:
                print(f"   [OBC] {text}")

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
