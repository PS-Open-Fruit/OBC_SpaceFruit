import serial
import struct
import time
import random
import os
import subprocess

# ==========================================
# 1. Protocol Definitions
# ==========================================

class EPSProtocol:
    FEND = 0xC0
    FESC = 0xDB
    TFEND = 0xDC
    TFESC = 0xDD
    HEADER = 0x00

    @staticmethod
    def escape(data: bytes) -> bytes:
        output = bytearray()
        for byte in data:
            if byte == EPSProtocol.FEND:
                output.extend([EPSProtocol.FESC, EPSProtocol.TFEND])
            elif byte == EPSProtocol.FESC:
                output.extend([EPSProtocol.FESC, EPSProtocol.TFESC])
            else:
                output.append(byte)
        return bytes(output)

    @staticmethod
    def unescape(data: bytes) -> bytes:
        output = bytearray()
        i = 0
        while i < len(data):
            byte = data[i]
            if byte == EPSProtocol.FESC:
                i += 1
                if i >= len(data): break
                if data[i] == EPSProtocol.TFEND:
                    output.append(EPSProtocol.FEND)
                elif data[i] == EPSProtocol.TFESC:
                    output.append(EPSProtocol.FESC)
            else:
                output.append(byte)
            i += 1
        return bytes(output)

    @staticmethod
    def wrap_frame(payload: bytes) -> bytes:
        frame = bytearray([EPSProtocol.FEND, EPSProtocol.HEADER])
        frame.extend(EPSProtocol.escape(payload))
        frame.append(EPSProtocol.FEND)
        return bytes(frame)

    @staticmethod
    def unwrap_frame(frame: bytes) -> bytes:
        if len(frame) < 3: return None
        inner = frame[1:-1]
        if inner[0] != EPSProtocol.HEADER: return None
        return EPSProtocol.unescape(inner[1:])

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
        self.switches = [0] * 6
        self.payload_powered = True # Assume ON if script is running on the Pi
        self.img_counter = 0
        self.last_captured_file = None

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
            # Run command, suppress stdout/stderr to keep terminal clean
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            if os.path.exists(filename):
                self.last_captured_file = filename
                return os.path.getsize(filename)
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
                            start = buffer.index(EPSProtocol.FEND)
                            end = buffer.index(EPSProtocol.FEND, start + 1)
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
        payload = EPSProtocol.unwrap_frame(frame_bytes)
        if not payload: return

        cmd_id = payload[0]
        resp = b''

        print(f"Cmd: 0x{cmd_id:02X}")

        # --- EPS SIMULATION ---
        if cmd_id == 0x01: # Solar
            ch = payload[1] if len(payload)>1 else 0
            resp = struct.pack("<hhB", random.randint(3800, 4100), random.randint(50, 600), ch)
        
        elif cmd_id == 0x02: # Output
            ch = payload[1] if len(payload)>1 else 0
            on = self.switches[ch] if ch < 6 else 0
            resp = struct.pack("<HHB", 5000 if on else 0, 150 if on else 0, ch)
            
        elif cmd_id == 0x03: # Switch Status
            ch = payload[1]
            on = self.switches[ch] if ch < 6 else 0
            resp = struct.pack("<BB", on, ch)
            
        elif cmd_id == 0x04: # Batt Temp
            ch = payload[1]
            resp = struct.pack("<fB", 25.0 + random.random(), ch)
            
        elif cmd_id == 0x05: # Set Switch
            ch = payload[1]
            state = payload[2]
            if ch < 6: self.switches[ch] = state
            resp = struct.pack("<BBB", 0x88, state, ch)

        # --- PAYLOAD SIMULATION (Actual Pi Data) ---
        elif cmd_id == 0x10: # Ping
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
            
            # Fallback to simulation if capture failed (or if you want to test without camera)
            if size == 0:
                print("   ⚠️ Capture failed (or no camera). Sending simulated size.")
                size = random.randint(50000, 200000)
                # Create dummy file for download test
                self.last_captured_file = "dummy_img.jpg"
                with open(self.last_captured_file, "wb") as f:
                    f.write(os.urandom(size))
            
            # Response: [0x12 (Cmd ID)] [ImgID (2b)] [Size (4b)]
            resp = bytearray([0x12])
            resp.extend(struct.pack("<HI", self.img_counter, size))

        elif cmd_id == 0x13: # Get Image Chunk
            # Payload: [ChunkID (2 bytes)]
            # Response: [ChunkID (2 bytes)] [Data (up to 200 bytes)]
            if len(payload) >= 3 and self.last_captured_file:
                chunk_id = struct.unpack("<H", payload[1:3])[0]
                CHUNK_SIZE = 200
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
            tx = EPSProtocol.wrap_frame(resp)
            self.serial_conn.write(tx)

if __name__ == "__main__":
    # RPi is USB Host, STM32 is Device /dev/ttyACM0
    sim = RPiVRSimulator('/dev/ttyACM0') 
    sim.start()
