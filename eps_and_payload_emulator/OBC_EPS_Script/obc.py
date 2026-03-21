import serial
import struct
import time

# ==========================================
# 1. Protocol Definitions (Shared Logic)
# ==========================================
# (These classes must match the EPS Firmware exactly)

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
# 2. OBC Controller Class
# ==========================================

class OBCController:
    def __init__(self, port, baudrate=9600):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            print(f"✅ OBC Connected to {port}")
        except serial.SerialException as e:
            print(f"❌ Connection Failed: {e}")
            exit()

    def send_command(self, cmd_id: int, params: list = None):
        """Constructs and sends a frame."""
        payload = bytearray([cmd_id])
        if params:
            payload.extend(params)
        
        frame = EPSProtocol.wrap_frame(payload)
        self.ser.write(frame)

    def read_response(self, timeout=1.0):
        """Reads serial buffer, finds frame, decodes it."""
        buffer = bytearray()
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))
                
                if EPSProtocol.FEND in buffer:
                    try:
                        start = buffer.index(EPSProtocol.FEND)
                        end = buffer.index(EPSProtocol.FEND, start + 1)
                        frame = buffer[start : end + 1]
                        return EPSProtocol.unwrap_frame(frame)
                    except ValueError:
                        continue 
        return None

    # --- EPS Functions ---

    def get_solar_data(self, channel):
        self.send_command(0x01, [channel])
        resp = self.read_response()
        if resp and len(resp) == 5:
            return struct.unpack("<hhB", resp)
        return None, None, None

    def get_output_data(self, channel):
        self.send_command(0x02, [channel])
        resp = self.read_response()
        if resp and len(resp) == 5:
            return struct.unpack("<HHB", resp)
        return None, None, None

    def get_switch_status(self, channel):
        self.send_command(0x03, [channel])
        resp = self.read_response()
        if resp and len(resp) == 2:
            state, ch = struct.unpack("<BB", resp)
            return state == 1
        return None

    def set_switch(self, channel, state):
        state_byte = 1 if state else 0
        self.send_command(0x05, [channel, state_byte])
        resp = self.read_response()
        if resp and len(resp) == 3:
            confirm, new_state, ch = struct.unpack("<BBB", resp)
            return confirm == 0x88 and new_state == state_byte
        return False

    def get_batt_temp(self, channel):
        self.send_command(0x04, [channel])
        resp = self.read_response()
        if resp and len(resp) == 5:
            temp, ch = struct.unpack("<fB", resp)
            return temp
        return None

    # --- Payload (RPi) Functions ---

    def payload_ping(self):
        """CMD 0x10: PING -> PONG"""
        self.send_command(0x10)
        resp = self.read_response(timeout=2.0)
        if resp == b'PONG':
            return True
        return False

    def payload_get_status(self):
        """CMD 0x11: Returns (CPU%, Temp, FreeRAM)"""
        self.send_command(0x11)
        resp = self.read_response(timeout=2.0)
        if resp and len(resp) == 7: # B f H
            return struct.unpack("<BfH", resp)
        return None

    def payload_capture_image(self):
        """CMD 0x12: Returns (ImageID, SizeBytes)"""
        self.send_command(0x12)
        resp = self.read_response(timeout=5.0) # Longer timeout for capture
        if resp and len(resp) == 6: # H I
            return struct.unpack("<HI", resp)
        return None

# ==========================================
# 3. Main Mission Loop
# ==========================================

def test_eps_subsystem(obc: OBCController):
    print("\n=== [TEST] EPS SUBSYSTEM ===")
    
    # 1. Solar
    print(">> Scanning Solar Arrays...")
    for i in range(2): # Scan first 2 for brevity
        v, c, ch = obc.get_solar_data(i)
        if v is not None:
             print(f"   Panel {i}: {v}mV, {c}mA")
        else:
             print(f"   Panel {i}: No Response")
    
    # 2. Battery
    print(">> Checking Battery Temp...")
    t = obc.get_batt_temp(0)
    print(f"   Temp: {t:.2f}C")

def test_payload_subsystem(obc: OBCController):
    print("\n=== [TEST] PAYLOAD SUBSYSTEM (RPi Zero 2W) ===")
    
    PAYLOAD_PWR_CH = 2
    
    # 1. Power ON
    print(f">> Powering ON Payload (Switch {PAYLOAD_PWR_CH})...")
    if obc.set_switch(PAYLOAD_PWR_CH, True):
        print("   ✅ Power ON Command Accepted")
    else:
        print("   ❌ Power ON Failed")
        return

    # 2. Wait for Boot (Simulator simulates 5s boot time)
    print(">> Waiting for Payload Boot (6s)...")
    time.sleep(6)

    # 3. Ping
    print(">> Pinging Payload...")
    if obc.payload_ping():
        print("   ✅ PONG received! Payload is execution-ready.")
    else:
        print("   ❌ Payload Unresponsive")
        return

    # 4. Status
    print(">> Getting Health Status...")
    stats = obc.payload_get_status()
    if stats:
        cpu, temp, ram = stats
        print(f"   CPU: {cpu}% | Temp: {temp:.1f}C | RAM: {ram}MB Free")
    
    # 5. Mission: Take Photo
    print(">> Mission: Capture Image...")
    img_data = obc.payload_capture_image()
    if img_data:
        img_id, size = img_data
        print(f"   📸 Image Captured! ID: {img_id}, Size: {size/1024:.1f} KB")
    else:
        print("   ❌ Capture Failed")

    # 6. Power OFF
    print(">> Powering OFF Payload...")
    obc.set_switch(PAYLOAD_PWR_CH, False)
    print("   ✅ Power OFF.")


if __name__ == "__main__":
    OBC_PORT = 'COM19'
    obc = OBCController(OBC_PORT)
    
    print("\n--- 🚀 OBC MISSION START ---\n")

    # ==========================================
    # ENABLE / DISABLE TESTS HERE
    # ==========================================

    # --- 1. EPS Test ---
    # Comment out to disable
    test_eps_subsystem(obc) 

    # --- 2. Payload Test ---
    # Comment out to disable
    test_payload_subsystem(obc)

    print("\n--- 🏁 Mission Sequence Complete ---")
    obc.ser.close()