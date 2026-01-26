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

def get_disk_free():
    """Returns filesystem free space in MB."""
    try:
        s = os.statvfs('/')
        return (s.f_bavail * s.f_frsize) // (1024 * 1024)
    except:
        return 0

def get_uptime():
    """Returns uptime in seconds."""
    try:
        with open('/proc/uptime', 'r') as f:
            return int(float(f.read().split()[0]))
    except:
        return 0

def get_throttled():
    """Returns throttled state from vcgencmd."""
    try:
        output = subprocess.check_output(["vcgencmd", "get_throttled"]).decode()
        # Output format: throttled=0x0
        return int(output.split('=')[1], 0)
    except:
        return 0

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
        # Note: kiss_cmd is the KISS Command Byte (usually 0x00 for Data)
        
        if kiss_cmd != KISSProtocol.CMD_DATA:
            return

        # Min size: [CMD] [CRC:4] = 5 bytes
        if not payload or len(payload) < 5: return

        # Validate CRC (Must include KISS Command Byte 0x00)
        # payload is [APP_CMD] [DATA] [CRC]
        rx_crc_bytes = payload[-4:]
        rx_crc = struct.unpack('<I', rx_crc_bytes)[0]
        
        # Reconstruct data covered by CRC: [KISS_CMD] + [APP_CMD] + [DATA]
        data_to_check = bytearray([kiss_cmd]) + payload[:-4]
        
        calc_crc = KISSProtocol.calculate_crc(data_to_check)
        if calc_crc != rx_crc:
            print(f"   ⚠️ CRC Fail: Rx {rx_crc:08X} != Calc {calc_crc:08X}")
            return
            
        # Dispatch
        # payload[:-4] is [APP_CMD] [DATA]
        cmd_id = payload[0]
        app_payload = payload[1:-4]

        resp_payload = b''

        if cmd_id != 0x13:
            print(f"Cmd: 0x{cmd_id:02X}")

        # --- PAYLOAD SIMULATION (Actual Pi Data) ---
        if cmd_id == 0x10: # Ping
            # Response: [0x10] [0x01] (Framed Pong)
            resp_payload = bytearray([0x10, 0x01])
            
        elif cmd_id == 0x11: # Status
            cpu = int(get_cpu_load())
            temp = get_rpi_temp()
            ram = get_free_ram()
            disk = get_disk_free()
            uptime = get_uptime()
            throttled = get_throttled()
            
            # Prepend 0x11 Command ID
            resp_payload = bytearray([0x11])
            resp_payload.extend(struct.pack("<BfHIIH", cpu, temp, ram, disk, uptime, throttled))
            
        elif cmd_id == 0x12: # Capture
            self.img_counter += 1
            
            # Attempt Real Capture
            size = self.capture_real_image(self.img_counter)
            
            if size == 0:
                print("   ⚠️ Capture failed (or no camera). Sending size 0.")
            
            # Response: [0x12] [ImgID] [Size]
            resp_payload = bytearray([0x12])
            resp_payload.extend(struct.pack("<HI", self.img_counter, size))

        elif cmd_id == 0x13: # Get Image Chunk
            # Request Payload: [ChunkID (2 bytes)]
            if len(app_payload) >= 2 and self.last_captured_file:
                chunk_id = struct.unpack("<H", app_payload[0:2])[0]
                CHUNK_SIZE = 200
                
                # Show progress
                if self.last_file_size > 0:
                    total_chunks = (self.last_file_size + CHUNK_SIZE - 1) // CHUNK_SIZE
                    is_last_chunk = (chunk_id >= total_chunks - 1)
                    if chunk_id % 5 == 0 or is_last_chunk:
                        # calc percent
                        percent = ((chunk_id + 1) / total_chunks) * 100
                        print(f"   📡 Transferring: {percent:.1f}% (Chunk {chunk_id + 1}/{total_chunks})", end='\r')
                        if is_last_chunk: print("\n   ✅ Transfer Complete!")

                offset = chunk_id * CHUNK_SIZE
                
                try:
                    with open(self.last_captured_file, "rb") as f:
                        f.seek(offset)
                        data = f.read(CHUNK_SIZE)
                        
                        # Header: 0x13 + ChunkID
                        resp_payload = bytearray([0x13])
                        resp_payload.extend(struct.pack("<H", chunk_id))
                        resp_payload.extend(data)
                        
                except Exception as e:
                    print(f"   Read Error: {e}")

        # Send Response (with CRC)
        if resp_payload:
            # 1. Calc CRC (Exclude KISS CMD 0x00 because main.c SLIP_Decode strips it from check)
            crc = KISSProtocol.calculate_crc(resp_payload)
            
            # 2. Append CRC
            full_payload = bytearray(resp_payload)
            full_payload.extend(struct.pack('<I', crc))
            
            # 3. Wrap (KISS Command 0x00)
            tx = KISSProtocol.wrap_frame(full_payload)
            self.serial_conn.write(tx)

if __name__ == "__main__":
    # RPi is USB Host, STM32 is Device /dev/ttyACM0
    sim = RPiVRSimulator('/dev/ttyACM0') 
    sim.start()
