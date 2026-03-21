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
        # 0x04 Temp: Float + Byte (Note: Spec implies 4 bytes, but class used 5. 
        # Adjusting to strict 5-byte class output to match your previous code preference)
        return struct.pack("<fB", self.temp, self.channel)

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
# 3. The EPS Firmware Simulator
# ==========================================

class EPSSimulator:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_conn = None
        
        # --- Virtual Hardware State ---
        self.switches = [0] * 6  # 6 Switches (0=OFF, 1=ON)
        
    def start(self):
        """Opens serial port and starts listening loop."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            print(f"✅ EPS Simulator Online on {self.port} @ {self.baudrate}")
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
                        # Find start and end of frame (0xC0)
                        try:
                            start_idx = buffer.index(EPSProtocol.FEND)
                            # Search for end C0 after the start C0
                            end_idx = buffer.index(EPSProtocol.FEND, start_idx + 1)
                            
                            # Extract complete frame
                            frame = buffer[start_idx : end_idx + 1]
                            
                            # Remove processed data from buffer
                            del buffer[:end_idx + 1]
                            
                            # Handle the frame
                            self.handle_frame(frame)
                            
                        except ValueError:
                            # No complete frame found yet
                            break
                            
                time.sleep(0.01) # Prevent CPU hogging
                
            except KeyboardInterrupt:
                self.running = False
                print("\nStopping EPS Simulator...")
            except Exception as e:
                print(f"Error in loop: {e}")

    def handle_frame(self, frame_bytes):
        """Unwraps frame, identifies command, generates response."""
        payload = EPSProtocol.unwrap_frame(frame_bytes)
        if not payload:
            return # Invalid frame

        cmd_id = payload[0]
        response_payload = b''
        
        print(f"📥 Rx Cmd: 0x{cmd_id:02X} | Raw: {frame_bytes.hex().upper()}")

        # === COMMAND PROCESSING LOGIC ===
        
        # 1. Get Solar Data (0x01)
        if cmd_id == 0x01:
            channel = payload[1] if len(payload) > 1 else 0
            # Generate dummy values
            sim_volts = random.randint(3500, 4200) # 3.5V - 4.2V
            sim_curr = random.randint(100, 800)    # 100mA - 800mA
            
            sensor = viSen(sim_volts, sim_curr, channel)
            response_payload = sensor.kissData()
            print(f"   Action: Read Solar Ch {channel} -> {sim_volts}mV, {sim_curr}mA")

        # 2. Get Output Power (0x02)
        elif cmd_id == 0x02:
            channel = payload[1] if len(payload) > 1 else 0
            # Check if switch is ON in our virtual state
            is_on = self.switches[channel] if channel < 6 else 0
            
            sim_volts = 5000 if is_on else 0
            sim_curr = random.randint(200, 400) if is_on else 0
            
            sensor = outSen(sim_volts, sim_curr, channel)
            response_payload = sensor.kissData()
            print(f"   Action: Read Output Ch {channel} -> {sim_volts}mV, {sim_curr}mA")

        # 3. Get Switch Status (0x03)
        elif cmd_id == 0x03:
            channel = payload[1] if len(payload) > 1 else 0
            state = self.switches[channel] if channel < 6 else 0
            
            sw_obj = outSwitchState(channel, state)
            response_payload = sw_obj.get_status_packet()
            print(f"   Action: Get Switch Ch {channel} Status -> {'ON' if state else 'OFF'}")

        # 4. Get Battery Temp (0x04)
        elif cmd_id == 0x04:
            channel = payload[1] if len(payload) > 1 else 0
            sim_temp = 25.0 + random.uniform(-2, 5)
            
            temp_obj = battTemp(channel)
            temp_obj.temp = sim_temp
            response_payload = temp_obj.kissData()
            print(f"   Action: Get Temp Ch {channel} -> {sim_temp:.2f}C")

        # 5. Set Switch (0x05)
        elif cmd_id == 0x05:
            # Payload: [Cmd] [Switch] [State]
            if len(payload) >= 3:
                channel = payload[1]
                req_state = payload[2]
                
                # Update virtual state
                if channel < 6:
                    self.switches[channel] = 1 if req_state > 0 else 0
                
                sw_obj = outSwitchState(channel, self.switches[channel])
                response_payload = sw_obj.get_confirm_packet()
                print(f"   Action: Set Switch Ch {channel} to {'ON' if req_state else 'OFF'}")
        # 6. Get ALL Telemetry Data (0x06)
        elif cmd_id == 0x06:
            bulk_data = bytearray()
            
            # Loop through all 6 channels for Solar, Output, and Switch
            for ch in range(8):
                # 1. Solar Data
                sim_volts = random.randint(3500, 4200)
                sim_curr = random.randint(100, 800)
                bulk_data.extend(viSen(sim_volts, sim_curr, ch).kissData())
            for ch in range(6):
                # 2. Output Data
                is_on = self.switches[ch]
                out_volts = 5000 if is_on else 0
                out_curr = random.randint(200, 400) if is_on else 0
                bulk_data.extend(outSen(out_volts, out_curr, ch).kissData())
                
                # 3. Switch State
                bulk_data.extend(outSwitchState(ch, is_on).get_status_packet())
                
            # 4. Battery Temp (Only 2 channels: 0 and 1)
            for ch in range(2):
                temp_obj = battTemp(ch)
                temp_obj.temp = 25.0 + random.uniform(-2, 5)
                bulk_data.extend(temp_obj.kissData())
                
            response_payload = bytes(bulk_data)
            print(f"   Action: Bulk Read ALL Telemetry Data -> {len(response_payload)} bytes")
        else:
            print(f"   ⚠️ Unknown Command: 0x{cmd_id:02X}")
            return

        # === SEND RESPONSE ===
        if response_payload:
            tx_frame = EPSProtocol.wrap_frame(response_payload)
            self.serial_conn.write(tx_frame)
            print(f"📤 Tx Frame: {tx_frame.hex().upper()}")
            print("-" * 40)

# ==========================================
# 4. Main Entry Point
# ==========================================

if __name__ == "__main__":
    # Change 'COM_PORT' to match your setup
    # Windows: 'COM1', 'COM2' (Use a Virtual COM pair like com0com for testing)
    # Linux/Mac: '/dev/ttyUSB0', '/dev/pts/4' 
    
    COM_PORT = '/dev/serial0' # <--- CHANGE THIS
    
    eps = EPSSimulator(COM_PORT)
    eps.start()
