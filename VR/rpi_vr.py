import serial
import struct
import time
import random
import os
import subprocess
import sys

# Add Shared library path (two levels up)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from Shared.Python.kiss_protocol import KISSProtocol
except ImportError:
    # Fallback to handle running locally/different env
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
    from Shared.Python.kiss_protocol import KISSProtocol

# ==========================================
# 1. Protocol Definitions (Moved to Shared Lib)
# ==========================================

# ==========================================
# 2. Hardware Access (Real RPi Data)
# ==========================================

def get_rpi_temp():
    """Reads the actual CPU temperature of the Pi."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read()) / 1000.0
    except:
        return 25.0 # Fallback for non-Pi env

def get_cpu_load():
    """Reads 1-minute load average."""
    try:
        return os.getloadavg()[0] * 10.0 # Scale roughly to %
    except:
        return 10.0

def get_free_ram():
    """Returns free RAM in MB."""
    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if 'MemAvailable' in line:
                    return int(line.split()[1]) // 1024
    except:
        return 256

# ==========================================
# 3. VR Simulator Logic
# ==========================================

class RPiVRSimulator:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # State
        self.img_counter = 0
        self.last_captured_file = None
        self.last_file_size = 0

    def capture_real_image(self, img_id):
        """Captures a real JPEG using rpicam-still."""
        filename = f"mission_img_{img_id:04d}.jpg"
        # 1296x972 is a good binning mode for v2/v3 cameras
        cmd = [
            "rpicam-still", 
            "-o", filename, 
            "-t", "500",         # 500ms delay for Auto-Exposure/White Balance
            "--width", "1296",   
            "--height", "972",
            "--nopreview"
        ]
        
        try:
            print(f"   📸 Capturing: {filename}...")
            # Run command, suppress stdout/stderr to keep terminal clean, timeout after 5s
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            
            if os.path.exists(filename):
                self.last_captured_file = filename
                self.last_file_size = os.path.getsize(filename)
                return self.last_file_size
        except subprocess.TimeoutExpired:
            print("   ❌ Camera Timed Out!")
        except Exception as e:
            print(f"   ❌ Camera Error: {e}")
            
        return 0 # failure

    def start(self):
        print(f"--- 🍓 Raspberry Pi VR Simulator ---")
        print(f"Target Port: {self.port} (STM32 USB Device)")
        
        while True:
            try:
                # Retry connection logic for USB hotplugging
                if not os.path.exists(self.port):
                    print(f"Waiting for device {self.port}...", end='\r')
                    time.sleep(1)
                    continue
                
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.running = True
                print(f"\n✅ Connected to OBC! Listening...")
                self.listen_loop()
                
            except serial.SerialException as e:
                print(f"❌ Serial Error: {e}")
                time.sleep(1)
            except KeyboardInterrupt:
                print("\nExiting...")
                exit()

    def listen_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(chunk)
                    
                    while True:
                        try:
                            start = buffer.index(KISSProtocol.FEND)
                            end = buffer.index(KISSProtocol.FEND, start + 1)
                            frame = buffer[start : end + 1]
                            del buffer[:end + 1]
                            
                            self.handle_frame(frame)
                        except ValueError:
                            break
            except OSError:
                print("\n⚠️ Device Disconnected!")
                self.running = False
                self.serial_conn.close()
                return

    def handle_frame(self, frame_bytes):
        result = KISSProtocol.unwrap_frame(frame_bytes)
        if not result: return 
        
        kiss_cmd, payload = result
        if kiss_cmd != KISSProtocol.CMD_DATA:
            return

        if not payload: return

        cmd_id = payload[0]
        resp = b''

        if cmd_id != 0x13:
            print(f"Cmd: 0x{cmd_id:02X}")

        # --- PAYLOAD SIMULATION (Actual Pi Data) ---
        if cmd_id == 0x10: # Ping
            resp = b'PONG'
            
        elif cmd_id == 0x11: # Status
            cpu = int(get_cpu_load())
            temp = get_rpi_temp()
            ram = get_free_ram()
            resp = struct.pack("<BfH", cpu, temp, ram)
            
        elif cmd_id == 0x12: # Capture
            self.img_counter += 1
            
            # Attempt Real Capture
            size = self.capture_real_image(self.img_counter)
            
            if size == 0:
                print("   ⚠️ Capture failed (or no camera). Sending size 0.")
            
            # Response: [0x12 (Cmd ID)] [ImgID (2b)] [Size (4b)]
            resp = bytearray([0x12])
            resp.extend(struct.pack("<HI", self.img_counter, size))

        elif cmd_id == 0x13: # Get Image Chunk
            # Payload: [ChunkID (2 bytes)]
            # Response: [ChunkID (2 bytes)] [Data (up to 200 bytes)]
            if len(payload) >= 3 and self.last_captured_file:
                chunk_id = struct.unpack("<H", payload[1:3])[0]
                CHUNK_SIZE = 200
                
                # Show progress
                if self.last_file_size > 0:
                    total_chunks = (self.last_file_size + CHUNK_SIZE - 1) // CHUNK_SIZE
                    
                    # Calculate percentage based on (chunk_id + 1) since chunk_id is 0-indexed
                    percent = ((chunk_id + 1) / total_chunks) * 100
                    
                    # Print progress bar every 5 chunks or if it's the last one
                    is_last_chunk = (chunk_id >= total_chunks - 1)
                    if chunk_id % 5 == 0 or is_last_chunk:
                        print(f"   📡 Transferring: {percent:.1f}% (Chunk {chunk_id + 1}/{total_chunks})", end='\r')
                        
                        if is_last_chunk:
                            print() # Clear line/New line
                            print("   ✅ Transfer Complete!")

                offset = chunk_id * CHUNK_SIZE
                
                try:
                    with open(self.last_captured_file, "rb") as f:
                        f.seek(offset)
                        data = f.read(CHUNK_SIZE)
                        
                        # Header: 0x13 + ChunkID
                        resp = bytearray([0x13])
                        resp.extend(struct.pack("<H", chunk_id))
                        resp.extend(data)
                        
                        # print(f"   Sent Chunk {chunk_id}, Len: {len(data)}") 
                except Exception as e:
                    print(f"   Read Error: {e}")
            else:
                resp = b'' # Error or no image

        if resp:
            tx = KISSProtocol.wrap_frame(resp)
            self.serial_conn.write(tx)

if __name__ == "__main__":
    # RPi is USB Host, STM32 is Device /dev/ttyACM0
    sim = RPiVRSimulator('/dev/ttyACM0') 
    sim.start()
