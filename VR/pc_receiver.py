import serial
import threading
import time
import sys
import os

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
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
        # Download State
        self.downloading = False
        self.current_img_data = bytearray()
        self.expected_size = 0
        self.start_time = 0

    def start(self):
        """Starts the Ground Station (Connection + Threads)."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            print(f"✅ GS Connected to {self.port} @ {self.baudrate}")
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

                elif cmd == '':
                    pass
                else:
                    self.send_telecommand(cmd)

            except KeyboardInterrupt:
                self.running = False
                break
        
        if self.ser: self.ser.close()

    def listen_loop(self):
        """Background thread to handle incoming Telemetry/Data."""
        print("   [RX] Listener started...")
        
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline()
                        if not line: continue
                        
                        # Use ignore to handle binary data mixed with text
                        text = line.decode('utf-8', errors='ignore').strip()
                        self.process_telemetry(text)
                        
                    except UnicodeDecodeError:
                        pass # Ignore noisy binary frame errors
            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(1)

    def process_telemetry(self, text):
        """Parses incoming text lines from OBC."""
        # 1. Detect Start of Download
        if "[OBC] Image Captured! Size:" in text:
            try:
                parts = text.split("Size: ")[1].split(" ")
                self.expected_size = int(parts[0])
                print(f"\n📥 INCOMING IMAGE: {self.expected_size/1024:.2f} KB")
                
                self.downloading = True
                self.current_img_data = bytearray()
                self.start_time = time.time()
            except:
                print(f"⚠️ Header Parse Error: {text}")

        # 2. Detect Data Chunk
        elif text.startswith("[DATA]"):
            try:
                hex_str = text.split("]", 1)[1].strip() # Safer split
                chunk_bytes = bytes.fromhex(hex_str)
                self.current_img_data.extend(chunk_bytes)
                
                self.print_progress()
            except ValueError:
                pass # Corrupt hex

        # 3. Detect End of Download
        elif "[OBC] Download Complete!" in text:
            filename = os.path.join(DOWNLOAD_DIR, f"img_{int(time.time())}.jpg")
            with open(filename, "wb") as f:
                f.write(self.current_img_data)
            
            print(f"\n✅ IMAGE SAVED: {filename}")
            if self.expected_size > 0:
                loss = 100 * (1 - len(self.current_img_data) / self.expected_size)
                if loss > 0: print(f"   ⚠️ Data Loss: {loss:.1f}%")
            
            self.downloading = False

        # 4. Standard Logs
        elif text:
            # Don't clutter UI while downloading unless it's an error
            if not self.downloading or "Error" in text:
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
