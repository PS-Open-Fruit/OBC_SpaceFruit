import serial
import threading
import time
import sys
import os
import struct
import zlib
import subprocess
import shutil
import json
from http.server import SimpleHTTPRequestHandler
from socketserver import TCPServer

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

class WebDashboard:
    def __init__(self, gs, port=8000, web_dir="web"):
        self.gs = gs # Reference to GroundStation for callbacks
        self.port = port
        self.web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), web_dir)
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        print(f"   🌐 Dashboard active at http://localhost:{port}")
        
    def _run_server(self):
        # Capture context for inner class
        dashboard = self
        
        class Handler(SimpleHTTPRequestHandler):
            def __init__(req_self, *args, **kwargs):
                super().__init__(*args, directory=dashboard.web_dir, **kwargs)
                
            def log_message(self, format, *args):
                pass # Suppress logging

            def do_POST(self):
                if self.path == '/api/command':
                    content_len = int(self.headers.get('Content-Length', 0))
                    post_body = self.rfile.read(content_len)
                    try:
                        data = json.loads(post_body)
                        cmd = data.get("command")
                        
                        if cmd == "ping":
                            dashboard.gs.send_kiss_command(0x10)
                        elif cmd == "status":
                            dashboard.gs.send_kiss_command(0x20)
                        elif cmd == "capture":
                            dashboard.gs.send_kiss_command(0x12)
                            
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        self.wfile.write(b'{"status":"ok"}')
                    except Exception as e:
                        print(f"   ❌ Web API Error: {e}")
                        self.send_response(500)
                        self.end_headers()
                else:
                    self.send_error(404)

        with TCPServer(("", self.port), Handler) as httpd:
            httpd.serve_forever()

class GroundStation:
    def __init__(self, port='COM9', baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        
        # Web Dashboard
        self.dashboard = WebDashboard(self)
        self.last_dashboard_update = 0
        
        # Dashboard State
        self.dash_state = {
            "connection": "OFFLINE",
            "last_seen": 0,
            "packet_count": 0,
            "img_id": 0,
            "img_progress": 0,
            "img_size": 0,
            "vr_status": {
                "cpu": 0, "temp": 0, "ram": 0, "disk": 0, "uptime": 0, "health": "N/A"
            },
            "logs": []
        }
        
        # Image Transfer State
        self.current_img_data = bytearray()
        self.received_chunk_ids = set() # Track unique chunks
        self.expected_size = 0
        self.expected_crc = 0
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
            self.dash_state["connection"] = "ONLINE"
            self.update_json_state()
        except Exception as e:
            print(f"❌ Connection Failed: {e}")
            self.dash_state["connection"] = "ERROR"
            self.update_json_state()
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
        
        self.dash_state["packet_count"] += 1
        
        # 2. Extract CRC
        received_crc = struct.unpack('<I', received_crc_bytes)[0]
        
        # 3. Calculate CRC (Over CMD + PAYLOAD)
        calc_crc = KISSProtocol.calculate_crc(data_content)
        
        if calc_crc != received_crc:
            if cmd_byte == 0x13: # Img Chunk
                print(f"\n⚠️ CRC ERROR on Chunk (Passing to SSDV...) Calc {calc_crc:08X} != Rx {received_crc:08X}")
                # Pass through for SSDV FEC to handle
            else:
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
            # Payload: [ImgID:2] [Size:4] [CRC:4]
            if len(payload_body) < 10: return
            img_id = struct.unpack('<H', payload_body[0:2])[0]
            self.expected_size = struct.unpack('<I', payload_body[2:6])[0]
            self.expected_crc = struct.unpack('<I', payload_body[6:10])[0]
            
            sys.stdout.write("\r")
            
            print(f"   [RX] START IMG #{img_id}, Size: {self.expected_size/1024:.2f} KB")
            
            self.downloading = True
            self.current_img_data = bytearray()
            self.received_chunk_ids = set()
            self.start_time = time.time()
            
            # Reset Dashboard
            self.dash_state["img_id"] = img_id
            self.dash_state["img_size"] = self.expected_size
            self.dash_state["img_progress"] = 0
            
            try:
                placeholder = os.path.join(self.dashboard.web_dir, "live.jpg")
                if os.path.exists(placeholder): os.remove(placeholder)
            except: pass
            
            self.update_json_state()
            return
            
        # CMD 0x21: VR Status Report (Raw Data)
        if cmd_byte == 0x21:
            # Payload Structure: [VR Data...] [VR CRC (4 bytes)]
            
            pl = payload_body
            pl_len = len(pl)

            # Robust unpacking (ignore tail CRC)
            try:
                if pl_len >= 17: # Handle Throttled (17 bytes)
                    cpu = pl[0]
                    temp, ram, disk, uptime, throttled = struct.unpack('<fHIIH', pl[1:17])
                    
                    # Decode Throttled (Bits 0-19)
                    status_flags = []
                    if throttled & 0x1: status_flags.append("⚡ UV LO")
                    if throttled & 0x2: status_flags.append("📉 FRQ CAP")
                    if throttled & 0x4: status_flags.append("🔥 THRTL")
                    if throttled & 0x8: status_flags.append("🌡️ SOFT T")
                    
                    hist_flags = []
                    if throttled & 0x10000: hist_flags.append("Was UV")
                    if throttled & 0x40000: hist_flags.append("Was Thrtl")
                    
                    throttled_str = "OK"
                    if status_flags: throttled_str = " | ".join(status_flags)
                    if hist_flags: throttled_str += f" (History: {', '.join(hist_flags)})"
                    
                elif pl_len >= 15: # Handle Uptime (15 bytes)
                    cpu = pl[0]
                    temp, ram, disk, uptime = struct.unpack('<fHII', pl[1:15])
                    throttled_str = "N/A"
                    
                else: 
                     print(f"   [RX] VR Status Too Short: {pl_len}")
                     return
                
                # Update Dashboard State
                self.dash_state["vr_status"] = {
                    "cpu": round(cpu, 1),
                    "temp": round(temp, 1),
                    "ram": ram,
                    "disk": disk,
                    "uptime": uptime,
                    "health": throttled_str
                }
                self.update_json_state()

                print(f"\n   📊 [VR STATUS DASHBOARD]")
                print(f"   ----------------------------------")
                print(f"   ⏱️  Uptime : {uptime} s ({uptime/3600:.1f} h)")
                print(f"   🧠 CPU    : {cpu}%")
                print(f"   🌡️  Temp   : {temp:.1f} °C")
                print(f"   💾 RAM    : {ram} MB Free")
                print(f"   💿 Disk   : {disk} MB Free")
                print(f"   ⚠️  Health : {throttled_str}")
                print(f"   ----------------------------------\n")
                sys.stdout.write("GS> ")
                sys.stdout.flush()
                
            except Exception as e:
                print(f"   [RX] Error parsing status: {e}")

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
            
            # Update Dashboard (Throttle: Max 1Hz)
            if time.time() - self.last_dashboard_update > 1.0:
                # Update Progress
                if self.expected_size > 0:
                     self.dash_state["img_progress"] = int((len(self.current_img_data) / self.expected_size) * 100)
                
                self.update_dashboard()
                self.last_dashboard_update = time.time()
                
            return

        print(f"   Rx Unknown CMD: 0x{cmd_byte:02X}")

    def update_json_state(self):
        """Writes current state to JSON for web UI."""
        self.dash_state["last_seen"] = time.time()
        try:
            path = os.path.join(self.dashboard.web_dir, "status.json")
            with open(path, "w") as f:
                json.dump(self.dash_state, f)
        except: pass

    def update_dashboard(self):
        """Attempts to decode partial SSDV data for the dashboard."""
        self.update_json_state() # Sync JSON
        
        if not self.current_img_data: return
        
        bin_path = os.path.join(self.dashboard.web_dir, "temp.bin")
        jpg_path = os.path.join(self.dashboard.web_dir, "live.jpg")
        
        try:
            with open(bin_path, "wb") as f:
                f.write(self.current_img_data)
                
            # Run SSDV Decoder
            # ssdv -d input.bin output.jpg
            subprocess.run(["ssdv", "-d", bin_path, jpg_path], 
                           check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass # Fail silently (e.g. missing ssdv tool)

    def process_log_line(self, text):
        """Parses ASCII log lines."""
        if not text: return
        
        # Add to dashboard logs (keep last 20)
        timestamp = time.strftime("%H:%M:%S")
        self.dash_state["logs"].append(f"[{timestamp}] {text}")
        if len(self.dash_state["logs"]) > 20:
             self.dash_state["logs"].pop(0)
        self.update_json_state()
        
        # 1. Detect End of Download
        if "Download Complete!" in text:
            # Save raw SSDV bin
            timestamp = int(time.time())
            filename_bin = os.path.join(DOWNLOAD_DIR, f"img_{timestamp}.bin")
            filename_jpg = os.path.join(DOWNLOAD_DIR, f"img_{timestamp}.jpg")
            
            with open(filename_bin, "wb") as f:
                f.write(self.current_img_data)
            
            print(f"\n   FILE SAVED: {filename_bin}")
            
            # Try to decode final JPG
            try:
                subprocess.run(["ssdv", "-d", filename_bin, filename_jpg], 
                              check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"   decoded -> {filename_jpg}")
            except:
                print("   (SSDV Decode skipped - tool missing)")

            if self.expected_size > 0:
                loss = 100 * (1 - len(self.current_img_data) / self.expected_size)
                if loss > 0: print(f"   ⚠️ Data Loss: {loss:.1f}%")
            
            # Integrity Check
            calc_crc = zlib.crc32(self.current_img_data) & 0xFFFFFFFF
            if calc_crc == self.expected_crc:
                 print(f"   ✅ Integrity Verified (CRC: {calc_crc:08X})")
            else:
                 print(f"   ❌ Integrity Mismatch! (Exp {self.expected_crc:08X} != Act {calc_crc:08X})")

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
                if "GS CMD:" not in text or "Ping" in text:
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
