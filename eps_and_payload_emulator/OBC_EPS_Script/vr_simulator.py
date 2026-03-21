import serial
import struct
import time
import random
import threading

# ==========================================
# 1. Data Classes (The internal registers)
# ==========================================

class viSen:
    def __init__(self, voltage: int = 0, current: int = 0, channel: int = 0):
        self.voltage = voltage
        self.current = current
        self.channel = channel

    def kissData(self) -> bytes:
        # 0x01 Solar: Signed 16-bit
        return struct.pack("<hhB", self.voltage, self.current, self.channel)

class outSen:
    def __init__(self, voltage=0, current=0, channel=0):
        self.voltage = voltage
        self.current = current
        self.channel = channel

    def kissData(self) -> bytes:
        # 0x02 Output: Unsigned 16-bit
        return struct.pack("<HHB", self.voltage, self.current, self.channel)

class outSwitchState:
    def __init__(self, channel=0, state=0):
        self.channel = channel
        self.state = state

    def get_confirm_packet(self) -> bytes:
        # 0x05 Write Confirmation
        WRITE_CONFIRM = 0x88
        return struct.pack("<BBB", WRITE_CONFIRM, self.state, self.channel)

    def get_status_packet(self) -> bytes:
        # 0x03 Read Status
        return struct.pack("<BB", self.state, self.channel)

class battTemp:
    def __init__(self, channel=0):
        self.channel = channel
        self.temp = 25.0

    def kissData(self) -> bytes:
        # 0x04 Temp: Float + Byte
        return struct.pack("<fB", self.temp, self.channel)

class payloadStatus:
    def __init__(self, cpu_load=0, temp=0, ram_free=0):
        self.cpu_load = cpu_load # Percent
        self.temp = temp         # Celsius
        self.ram_free = ram_free # MB

    def get_packet(self) -> bytes:
        # 0x11 Status: B (CPU), f (Temp), H (RAM)
        return struct.pack("<BfH", self.cpu_load, self.temp, self.ram_free)

class payloadImageInfo:
    def __init__(self, img_id=0, size=0):
        self.img_id = img_id
        self.size = size

    def get_packet(self) -> bytes:
        # 0x12 Status: H (ID), I (Size)
        return struct.pack("<HI", self.img_id, self.size)


# ==========================================
# 2. Protocol Handler (Encoder/Decoder)
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
        # Construct: C0 00 [Escaped Payload] C0
        frame = bytearray([EPSProtocol.FEND, EPSProtocol.HEADER])
        frame.extend(EPSProtocol.escape(payload))
        frame.append(EPSProtocol.FEND)
        return bytes(frame)

    @staticmethod
    def unwrap_frame(frame: bytes) -> bytes:
        # Remove C0 start/end and 00 header
        if len(frame) < 3: return None
        inner = frame[1:-1] # Strip C0
        if inner[0] != EPSProtocol.HEADER: return None # Check Header
        return EPSProtocol.unescape(inner[1:])

# ==========================================
# 3. The Combined Satellite Simulator
# ==========================================

class SatelliteSimulator:
    def __init__(self, port, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_conn = None
        
        # --- Virtual Hardware State (EPS) ---
        self.switches = [0] * 6  # 6 Switches (0=OFF, 1=ON)
        
        # --- Virtual Hardware State (Payload: RPi Zero 2W) ---
        self.payload_powered = False # Controlled by Switch 2 (Index 2 for simplicity)
        self.payload_boot_time = 0
        self.last_img_id = 100
    
    def start(self):
        """Opens serial port and starts listening loop."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            print(f"✅ VR Simulator (EPS + Payload) Online on {self.port} @ {self.baudrate}")
            print("   (Waiting for commands...)")
            self.listen_loop()
        except serial.SerialException as e:
            print(f"❌ Error opening port: {e}")

    def listen_loop(self):
        """Reads byte stream, parses frames, sends responses."""
        buffer = bytearray()
        
        while self.running:
            try:
                # Read incoming bytes
                if self.serial_conn.in_waiting > 0:
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(chunk)
                    
                    # Process buffer for complete frames
                    while True:
                        try:
                            start_idx = buffer.index(EPSProtocol.FEND)
                            end_idx = buffer.index(EPSProtocol.FEND, start_idx + 1)
                            frame = buffer[start_idx : end_idx + 1]
                            del buffer[:end_idx + 1]
                            self.handle_frame(frame)
                        except ValueError:
                            break
                            
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                self.running = False
                print("\nStopping Simulator...")
            except Exception as e:
                print(f"Error in loop: {e}")

    def handle_frame(self, frame_bytes):
        payload = EPSProtocol.unwrap_frame(frame_bytes)
        if not payload: return

        cmd_id = payload[0]
        response_payload = b''
        
        # Check EPS Logic first
        if cmd_id < 0x10:
            response_payload = self.handle_eps_command(cmd_id, payload)
        # Check Payload Logic
        else:
            response_payload = self.handle_payload_command(cmd_id, payload)

        # Send response if generated
        if response_payload:
            tx_frame = EPSProtocol.wrap_frame(response_payload)
            self.serial_conn.write(tx_frame)
            print(f"📤 Tx Frame: {tx_frame.hex().upper()}") 
            print("-" * 40)

    def handle_eps_command(self, cmd_id, payload):
        print(f"📥 EPS Cmd: 0x{cmd_id:02X}")
        
        # 1. Get Solar Data
        if cmd_id == 0x01:
            channel = payload[1] if len(payload) > 1 else 0
            read_vals = (random.randint(3500, 4200), random.randint(100, 800))
            print(f"   EPS: Read Solar Ch {channel} -> {read_vals}")
            return viSen(read_vals[0], read_vals[1], channel).kissData()

        # 2. Get Output Power
        elif cmd_id == 0x02:
            channel = payload[1] if len(payload) > 1 else 0
            is_on = self.switches[channel] if channel < 6 else 0
            read_vals = (5000 if is_on else 0, random.randint(200, 400) if is_on else 0)
            print(f"   EPS: Read Output Ch {channel} -> {read_vals}")
            return outSen(read_vals[0], read_vals[1], channel).kissData()

        # 3. Get Switch Status
        elif cmd_id == 0x03:
            channel = payload[1] if len(payload) > 1 else 0
            state = self.switches[channel] if channel < 6 else 0
            print(f"   EPS: Get Switch Ch {channel} Status -> {state}")
            return outSwitchState(channel, state).get_status_packet()

        # 4. Get Battery Temp
        elif cmd_id == 0x04:
            channel = payload[1] if len(payload) > 1 else 0
            temp = 25.0 + random.uniform(-2, 5)
            print(f"   EPS: Get Temp Ch {channel} -> {temp:.2f}")
            return battTemp(channel).kissData()

        # 5. Set Switch
        elif cmd_id == 0x05:
            if len(payload) >= 3:
                channel = payload[1]
                req_state = payload[2]
                if channel < 6:
                    self.switches[channel] = 1 if req_state > 0 else 0
                    
                    # Logic for Payload Power (Let's say Ch 2 is Payload)
                    if channel == 2:
                        if self.switches[channel]:
                            print("   >>> Powering ON Payload RPi...")
                            self.payload_boot_time = time.time()
                            self.payload_powered = True
                        else:
                            print("   >>> Powering OFF Payload RPi...")
                            self.payload_powered = False

                print(f"   EPS: Set Switch Ch {channel} to {req_state}")
                return outSwitchState(channel, self.switches[channel]).get_confirm_packet()
        
        return None

    def handle_payload_command(self, cmd_id, payload):
        print(f"📥 Payload Cmd: 0x{cmd_id:02X}")
        
        # Simulate Power State
        if not self.payload_powered:
            print("   ⚠️ Payload is OFF! No response.")
            return None # No power, no response
        
        # Simulate Boot Delay (5 seconds)
        if (time.time() - self.payload_boot_time) < 5.0:
            print("   ⚠️ Payload is BOOTING... No response.")
            return None

        # 0x10: PING
        if cmd_id == 0x10:
            print("   Payload: PONG")
            return b'PONG'

        # 0x11: Get Status
        elif cmd_id == 0x11:
            cpu = random.randint(5, 40)
            temp = 42.0 + random.uniform(0, 5)
            ram = 256
            print(f"   Payload: Status CPU={cpu}% Temp={temp:.1f}C")
            return payloadStatus(cpu, temp, ram).get_packet()

        # 0x12: Capture Image
        elif cmd_id == 0x12:
            self.last_img_id += 1
            size = random.randint(50000, 150000) # 50KB - 150KB
            print(f"   Payload: Captured Image ID={self.last_img_id}, Size={size}")
            return payloadImageInfo(self.last_img_id, size).get_packet()

        return None

if __name__ == "__main__":
    COM_PORT = 'COM18'
    sim = SatelliteSimulator(COM_PORT)
    sim.start()
