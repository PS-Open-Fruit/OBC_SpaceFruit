#!/usr/bin/env python3
"""
Ground Station - Ground-Driven Sliding Window Protocol
Based on RF-Benchmark ground-driven architecture
"""
import os
import sys
import time
import struct
import serial
import threading
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

# Protocol Commands
CMD_PING = 0x10
CMD_STATUS = 0x11
CMD_CAPTURE = 0x12
CMD_FILE_INFO = 0x14
CMD_REQUEST = 0x15
CMD_REQUEST_ACK = 0x16
CMD_SYNC = 0x17
CMD_BURST = 0x18
CMD_REPORT = 0x19
CMD_FINAL = 0x1A
CMD_DATA = 0x00

# Config
PC_PORT = 'COM8'
BAUDRATE = 9600
DOWNLOAD_DIR = "received_images"

if not os.path.exists(DOWNLOAD_DIR):
    os.makedirs(DOWNLOAD_DIR)

# ==========================================
# Web Dashboard
# ==========================================

class WebDashboard:
    def __init__(self, gs, port=8000, web_dir="web"):
        self.gs = gs
        self.port = port
        self.web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), web_dir)
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        print(f"   🌐 Dashboard active at http://localhost:{port}")
        
    def _run_server(self):
        dashboard = self
        
        class Handler(SimpleHTTPRequestHandler):
            def __init__(req_self, *args, **kwargs):
                super().__init__(*args, directory=dashboard.web_dir, **kwargs)
                
            def log_message(self, format, *args):
                pass

            def do_POST(self):
                if self.path == '/capture':
                    dashboard.gs.send_capture()
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b'OK')

        with TCPServer(("", self.port), Handler) as httpd:
            httpd.serve_forever()

# ==========================================
# Ground Station
# ==========================================

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
        
        # Transfer state
        self.current_file_info = None
        self.chunks = []
        self.received = set()
        self.run_id = 0
        self.mtu = 256
        self.total_chunks = 0
        self.crc_fail = 0
        self.start_time = 0

    def send_kiss_command(self, cmd_id, data=b''):
        """Send a KISS-framed command with CRC32."""
        if not self.ser or not self.ser.is_open: 
            print("❌ Serial Closed")
            return
        
        app_payload = bytes([cmd_id]) + data
        data_to_crc = bytes([KISSProtocol.CMD_DATA]) + app_payload
        crc = KISSProtocol.calculate_crc(data_to_crc)
        crc_bytes = struct.pack('<I', crc)
        
        full_payload = app_payload + crc_bytes
        frame = KISSProtocol.wrap_frame(full_payload, KISSProtocol.CMD_DATA)
        
        self.ser.write(frame)

    def send_capture(self):
        """Send CAPTURE command."""
        print("📸 Sending CAPTURE...")
        self.send_kiss_command(CMD_CAPTURE)

    def send_request(self, filename):
        """Send REQUEST command."""
        print(f"📥 Requesting: {filename}")
        filename_bytes = filename.encode('utf-8')[:64]
        data = bytearray([len(filename_bytes)])
        data.extend(filename_bytes)
        self.send_kiss_command(CMD_REQUEST, bytes(data))

    def send_report(self, run_id, missing):
        """Send REPORT with missing chunk IDs."""
        data = bytearray()
        data.extend(struct.pack('<H', run_id))
        data.append(min(len(missing), 255))
        for miss_id in sorted(missing)[:255]:
            data.extend(struct.pack('<H', miss_id))
        
        self.send_kiss_command(CMD_REPORT, bytes(data))

    def send_current_report(self):
        """Calculate missing chunks and send REPORT."""
        if not hasattr(self, 'run_id') or self.total_chunks == 0:
            return
            
        missing = set(range(self.total_chunks)) - self.received
        self.send_report(self.run_id, missing)
        self.burst_active = False

    def send_final(self, run_id):
        """Send FINAL acknowledgment."""
        data = struct.pack('<H', run_id)
        self.send_kiss_command(CMD_FINAL, data)

    def start(self):
        print(f"Connecting to {self.port} at {self.baud} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
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

    def cli_loop(self):
        """Interactive Command Line Interface."""
        print("\n--- 🛰️ OBC GROUND STATION CLI (Ground-Driven) ---")
        print("Commands: [c]apture, [s]tatus, [p]ing, [q]uit")
        print("---------------------------------------")

        while self.running:
            try:
                cmd = input("GS> ").strip().lower()
                
                if not cmd:
                    continue
                
                if cmd in ['c', 'capture']:
                    self.send_capture()
                elif cmd in ['s', 'status']:
                    print("   📊 Requesting Status...")
                    self.send_kiss_command(CMD_STATUS)
                elif cmd in ['p', 'ping']:
                    print("   🏓 Ping...")
                    self.send_kiss_command(CMD_PING)
                elif cmd in ['q', 'quit', 'exit']:
                    print("   👋 Exiting...")
                    self.running = False
                    break
                else:
                    print("❓ Unknown command")

            except KeyboardInterrupt:
                print("\n👋 Exiting...")
                self.running = False
                break

    def listen_loop(self):
        """Main RX Loop."""
        print("🎧 Listening for Telemetry...")
        
        buffer = bytearray()
        
        while self.running and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    buffer.extend(chunk)
                    
                    # Extract frames
                    while buffer:
                        fend_idx = buffer.find(KISSProtocol.FEND, 1)
                        if fend_idx == -1:
                            break
                        
                        frame = buffer[:fend_idx+1]
                        buffer = buffer[fend_idx+1:]
                        
                        if len(frame) > 2 and frame[0] == KISSProtocol.FEND and frame[-1] == KISSProtocol.FEND:
                            self.process_kiss_frame(frame)
                    
                    # Prevent buffer overflow
                    if len(buffer) > 8192:
                        buffer = buffer[-4096:]
                
                # Check for burst timeout
                if hasattr(self, 'burst_active') and self.burst_active:
                    if time.time() - self.burst_start_time > 2.0:
                        self.send_current_report()
                
                if self.ser.in_waiting == 0:
                    time.sleep(0.01)
            
            except Exception as e:
                print(f"❌ Error: {e}")
                time.sleep(0.1)

    def process_kiss_frame(self, frame_bytes):
        """Unescapes and handles KISS payload."""
        result = KISSProtocol.unwrap_frame(frame_bytes)
        if not result:
            return
        
        kiss_cmd, payload = result
        
        if len(payload) < 5:
            return
        
        payload_wo_crc = payload[0:-4]
        received_crc_bytes = payload[-4:]
        
        self.dash_state["packet_count"] += 1
        
        # Extract CRC
        received_crc = struct.unpack('<I', received_crc_bytes)[0]
        data_to_check = bytearray([kiss_cmd]) + payload_wo_crc
        calc_crc = KISSProtocol.calculate_crc(data_to_check)
        
        if kiss_cmd == KISSProtocol.CMD_DATA:
            if len(payload_wo_crc) < 1:
                return
            cmd_byte = payload_wo_crc[0]
            payload_body = payload_wo_crc[1:]
        else:
            cmd_byte = kiss_cmd
            payload_body = payload_wo_crc

        if calc_crc != received_crc:
            if cmd_byte != CMD_DATA:
                sys.stdout.write("\r")
                print(f"   ⚠️ CRC Fail on CMD 0x{cmd_byte:02X}")
                sys.stdout.write("GS> ")
                sys.stdout.flush()
            else:
                self.crc_fail += 1
            return
        
        # Handle different message types
        if cmd_byte == 0x01:  # Log
            text = payload_body.decode('utf-8', errors='replace')
            self.process_log_line(text)
            return

        # Clear line for async output
        sys.stdout.write("\r")

        if cmd_byte == CMD_PING:
            print("   🟢 Pong")
            sys.stdout.write("GS> ")
            sys.stdout.flush()
            return

        elif cmd_byte == CMD_STATUS:
            self.handle_vr_status(payload_body)
            # prompt restoration is handled inside handle_vr_status
            return

        elif cmd_byte == CMD_FILE_INFO:
            self.handle_file_info(payload_body)
            sys.stdout.write("GS> ")
            sys.stdout.flush()

        elif cmd_byte == CMD_REQUEST_ACK:
            self.handle_request_ack(payload_body)
            sys.stdout.write("GS> ")
            sys.stdout.flush()

        elif cmd_byte == CMD_SYNC:
            self.handle_sync(payload_body)
            sys.stdout.write("GS> ")
            sys.stdout.flush()

        elif cmd_byte == CMD_BURST:
            self.handle_burst(payload_body)
            sys.stdout.write("GS> ")
            sys.stdout.flush()

        elif cmd_byte == CMD_DATA:
            self.handle_data_frame(payload_body)
            # handle_data_frame manages the prompt restoration when transfer is complete

        elif cmd_byte == 0x21:  # VR Status
            self.handle_vr_status(payload_body)
            # handle_vr_status manages the prompt restoration

        else:
            print(f"   ⚠️ Unknown CMD 0x{cmd_byte:02X}")
            sys.stdout.write("GS> ")
            sys.stdout.flush()

    def handle_file_info(self, payload):
        """Handle FILE_INFO response."""
        if len(payload) < 1:
            return
        
        filename_len = payload[0]
        if len(payload) < 1 + filename_len + 4 + 40 + 2 + 2:
            return
        
        offset = 1
        filename = payload[offset:offset+filename_len].decode('utf-8', errors='ignore')
        offset += filename_len
        
        size = struct.unpack('<I', payload[offset:offset+4])[0]
        offset += 4
        
        sha1 = payload[offset:offset+40].decode('ascii', errors='ignore')
        offset += 40
        
        mtu, chunks = struct.unpack('<HH', payload[offset:offset+4])
        
        self.current_file_info = {
            "filename": filename,
            "size": size,
            "sha1": sha1,
            "mtu": mtu,
            "chunks": chunks
        }
        
        print(f"\n📋 FILE_INFO Received:")
        print(f"   File: {filename}")
        print(f"   Size: {size} bytes")
        print(f"   Chunks: {chunks}")
        print(f"   MTU: {mtu}")
        
        # Auto-request the file
        self.send_request(filename)

    def handle_request_ack(self, payload):
        """Handle REQUEST_ACK response."""
        if len(payload) < 1:
            return
        
        ok = payload[0] == 1
        
        if ok:
            print("   ✅ Request ACK - Transfer starting...")
        else:
            reason = payload[2:2+payload[1]].decode('utf-8') if len(payload) > 2 else "unknown"
            print(f"   ❌ Request NACK: {reason}")

    def handle_sync(self, payload):
        """Handle SYNC message."""
        if len(payload) < 10:
            return
        
        run_id, mtu, total_chunks, total_size = struct.unpack('<HHHI', payload[:10])
        
        self.run_id = run_id
        self.mtu = mtu
        self.total_chunks = total_chunks
        self.chunks = [None] * total_chunks
        self.received = set()
        self.crc_fail = 0
        self.start_time = time.time()
        self.burst_active = False
        self.expected_burst_seqs = []
        self.burst_received_seqs = set()
        
        print(f"\n🔄 SYNC Received:")
        print(f"   Run ID: {run_id}")
        print(f"   Total Chunks: {total_chunks}")
        print(f"   Total Size: {total_size} bytes")

    def handle_burst(self, payload):
        """Handle BURST announcement."""
        if len(payload) < 3:
            return
        
        run_id = struct.unpack('<H', payload[:2])[0]
        if run_id != self.run_id:
            return
        
        seq_count = payload[2]
        self.expected_burst_seqs = []
        offset = 3
        for i in range(seq_count):
            if offset + 2 <= len(payload):
                seq = struct.unpack('<H', payload[offset:offset+2])[0]
                self.expected_burst_seqs.append(seq)
                offset += 2
        
        self.burst_received_seqs = set()
        self.burst_start_time = time.time()
        self.burst_active = True

    def handle_data_frame(self, frame):
        """Handle incoming data frame."""
        # Frame format: [Magic:2] [RunID:2] [Seq:2] [Total:2] [Len:2] [Payload]
        if len(frame) < 10:
            return
        
        magic, run_id, seq, total, payload_len = struct.unpack('>HHHHH', frame[:10])
        
        MAGIC = 0xA55A
        if magic != MAGIC:
            return
        
        if run_id != self.run_id:
            return
        
        payload_capacity = self.mtu - 14
        if len(frame) < 10 + payload_capacity:
            return
        
        payload = frame[10:10+payload_capacity]
        
        # Store chunk
        if seq < len(self.chunks):
            self.chunks[seq] = payload[:payload_len]
            self.received.add(seq)
            self.print_progress()
            
            if hasattr(self, 'burst_active') and self.burst_active:
                if seq in self.expected_burst_seqs:
                    self.burst_received_seqs.add(seq)
                
                if len(self.burst_received_seqs) >= len(self.expected_burst_seqs):
                    self.send_current_report()

    def print_progress(self):
        """Print download progress."""
        received = len(self.received)
        elapsed = time.time() - self.start_time if self.start_time > 0 else 1
        
        total_received_bytes = sum(len(c) for c in self.chunks if c is not None)
        speed = total_received_bytes / elapsed if elapsed > 0.5 else 0
        
        progress = (received / self.total_chunks * 100) if self.total_chunks > 0 else 0
        
        missing = self.total_chunks - received
        
        bar_width = 30
        filled = int(bar_width * received / max(self.total_chunks, 1))
        bar = '█' * filled + '░' * (bar_width - filled)
        
        speed_str = f"{speed/1024:.1f} KB/s" if speed > 1024 else f"{speed:.1f} B/s"
        
        # Overwrite current line (which might be GS> or previous progress)
        print(f"\r   [{bar}] {progress:>5.1f}% | {speed_str:>10} | Missing: {missing:>4} | CRC Err: {self.crc_fail}   ", end='', flush=True)
        
        # Update dashboard
        self.dash_state["img_progress"] = int(progress)
        self.dash_state["img_size"] = self.total_chunks
        self.update_json_state()
        
        # Check if complete
        if received == self.total_chunks:
            print()  # New line
            self.transfer_complete()
            sys.stdout.write("GS> ")
            sys.stdout.flush()

    def transfer_complete(self):
        """Handle transfer completion."""
        data = bytearray()
        for chunk in self.chunks:
            if chunk:
                data.extend(chunk)
        
        if self.current_file_info:
            filename = self.current_file_info['filename']
        else:
             filename = f"received_{int(time.time())}.bin"
             
        filepath = os.path.join(DOWNLOAD_DIR, filename) 
        
        with open(filepath, 'wb') as f:
            f.write(data)
        
        # Verify SHA1
        import hashlib
        actual_sha1 = hashlib.sha1(data).hexdigest()
        expected_sha1 = self.current_file_info.get('sha1', '') if self.current_file_info else ''
        
        print(f"   ✅ Transfer Complete!")
        print(f"   Saved: {filepath}")
        print(f"   Size: {len(data)} bytes")
        
        if expected_sha1:
            if actual_sha1 == expected_sha1:
                print(f"   ✅ SHA1 Verified: {actual_sha1}")
            else:
                print(f"   ⚠️ SHA1 Mismatch!")
                print(f"      Expected: {expected_sha1}")
                print(f"      Actual:   {actual_sha1}")
        
        # Send FINAL
        self.send_final(self.run_id)
        
        # Decode SSDV if available
        self.decode_ssdv(filepath)

    def decode_ssdv(self, bin_file):
        """Decode SSDV to JPEG."""
        jpg_file = bin_file.replace('.bin', '_decoded.jpg')
        
        try:
            # Check if ssdv exists in path first or use absolute path if known
            # Using shell=True for windows compatibility sometimes helps but subprocess.run is better
            # Just try running it.
            import subprocess
            cmd = ['ssdv', '-d', bin_file, jpg_file]
            
            # Use shell=True if on windows and command not found otherwise, but let's stick to default
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"   🎞️ SSDV Decoded: {jpg_file}")
        except FileNotFoundError:
             print("   ⚠️ SSDV decode failed: 'ssdv' executable not found.")
        except Exception as e:
            print(f"   ⚠️ SSDV decode failed: {e}")

    def handle_vr_status(self, payload):
        """Handle VR status report."""
        try:
            if len(payload) >= 17:
                cpu, temp, ram, disk, uptime, throttled = struct.unpack("<BfHIIH", payload[:17])
                
                self.dash_state["vr_status"] = {
                    "cpu": cpu,
                    "temp": temp,
                    "ram": ram,
                    "disk": disk,
                    "uptime": uptime,
                    "health": "OK" if throttled == 0 else "THROTTLED"
                }
                
                print(f"\n   📊 VR Status:")
                print(f"   CPU: {cpu}%")
                print(f"   Temp: {temp:.1f}°C")
                print(f"   RAM: {ram} MB")
                print(f"   Disk: {disk} MB")
                print(f"   Uptime: {uptime}s")
                
                self.update_json_state()
                sys.stdout.write("GS> ")
                sys.stdout.flush()
        except Exception as e:
            print(f"   ⚠️ Status parse error: {e}")
            sys.stdout.write("GS> ")
            sys.stdout.flush()

    def update_json_state(self):
        """Writes current state to JSON for web UI."""
        import json
        self.dash_state["last_seen"] = time.time()
        try:
            with open(os.path.join(self.dashboard.web_dir, 'state.json'), 'w') as f:
                json.dump(self.dash_state, f)
        except:
            pass

    def process_log_line(self, text):
        """Parses ASCII log lines."""
        if not text:
            return
        
        timestamp = time.strftime("%H:%M:%S")
        self.dash_state["logs"].append(f"[{timestamp}] {text}")
        if len(self.dash_state["logs"]) > 20:
            self.dash_state["logs"].pop(0)
        self.update_json_state()
        
        # Clear line to overwrite GS> prompt, print log, then restore prompt
        sys.stdout.write("\r")
        print(f"   [LOG] {text}")
        sys.stdout.write("GS> ")
        sys.stdout.flush()

if __name__ == "__main__":
    gs = GroundStation(PC_PORT, BAUDRATE)
    gs.start()
