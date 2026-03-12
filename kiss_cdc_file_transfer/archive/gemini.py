import serial
import time
import os
import csv
from datetime import datetime
from kiss_protocol import KISSProtocol as KISS

# ─────────────────────────────────────────────────────────────
#  Configuration & Test Parameters
# ─────────────────────────────────────────────────────────────
DATA_PORT   = '/dev/serial/by-id/usb-Openfruit_Aerospace_OBC_VCP_206830844543-if00'  # Main DUT link (115200)
TRIG_PORT   = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10OMHTZ-if00-port0'  # Trigger link (9600)
BAUD_DATA   = 115200
BAUD_TRIG   = 9600

# Hex trigger: c0 00 01 02 00 03 00 ff ff 67 3C 6D B4 c0
TRIGGER_CMD = bytes.fromhex("c0000102000300ffff673C6DB4c0")

# Test Matrices
FILES_TO_TEST = ["testimg-1.jpg", "testimg-2.jpg","testimg-3.jpg"]
CHUNK_SIZES   = [128, 256, 512, 1024, 2048]

METRICS_CSV   = "automated_test_results.csv"

# Protocol Constants from main.c and main.py
PAYLOAD_ID_VR          = 0x01
VR_PID_IMAGE_REQUEST   = 0x02
VR_PID_IMAGE_DOWNLOAD  = 0x03
VR_PID_DOWNLOAD_DONE   = 0x05
PID_ACK                = 0xAC

class C:
    RESET = "\033[0m"; BOLD = "\033[1m"; GREEN = "\033[32m"
    YELLOW = "\033[33m"; RED = "\033[31m"; CYAN = "\033[36m"; MAGENTA = "\033[35m"

def log(tag, msg, color=C.CYAN):
    print(f"  {datetime.now().strftime('%H:%M:%S')}  {color}{C.BOLD}[{tag:<5}]{C.RESET} {msg}")

# ─────────────────────────────────────────────────────────────
#  Logging & Metrics
# ─────────────────────────────────────────────────────────────
def append_metrics(file_name, chunk_size, total_bytes, elapsed):
    """Logs results to CSV for later analysis."""
    rate_kbps = (total_bytes * 8) / elapsed / 1000 if elapsed > 0 else 0
    fieldnames = ["timestamp", "file", "chunk_size", "total_bytes", "elapsed_s", "rate_kbps"]
    file_exists = os.path.isfile(METRICS_CSV)
    
    with open(METRICS_CSV, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists: writer.writeheader()
        writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "file": file_name,
            "chunk_size": chunk_size,
            "total_bytes": total_bytes,
            "elapsed_s": round(elapsed, 4),
            "rate_kbps": round(rate_kbps, 2)
        })

# ─────────────────────────────────────────────────────────────
#  Core Test Logic
# ─────────────────────────────────────────────────────────────
def run_test_cycle(target_file, chunk_size):
    if not os.path.exists(target_file):
        log("ERROR", f"File {target_file} not found", C.RED)
        return False

    try:
        # Initialize ports
        ser_data = serial.Serial(DATA_PORT, BAUD_DATA, timeout=2)
        ser_trig = serial.Serial(TRIG_PORT, BAUD_TRIG, timeout=0.1)
        
        log("TEST", f"Starting: {target_file} @ {chunk_size} byte chunks", C.YELLOW)

        # Step 1: Send Trigger to DUT
        log("TRIG", "Sending hex trigger command...")
        ser_trig.write(TRIGGER_CMD)
        ack_trig = recv_kiss_frame(ser_trig)
        print(f"ack_trig {ack_trig}")
        ser_trig.flush()
        ser_trig.close() # Close trigger to avoid interference

        # Step 2: Prepare image data
        with open(target_file, "rb") as f:
            image_data = f.read()
        total_size = len(image_data)
        
        # Step 3: Wait for DUT to send IMAGE_REQUEST (PID 0x02)
        bytes_sent = 0
        current_chunk_id = 0
        start_time = None

        while bytes_sent < total_size:
            line = recv_kiss_frame(ser_data)
            if not line: continue
            
            frame = KISS.unwrap_frame(line)
            pid = frame.get('pid')

            # Respond to Request or ACK
            if pid == VR_PID_IMAGE_REQUEST or pid == PID_ACK:
                if start_time is None: start_time = time.monotonic()
                
                # Advance to next chunk
                start_idx = current_chunk_id * chunk_size
                end_idx = start_idx + chunk_size
                payload = image_data[start_idx:end_idx]
                
                if not payload: break

                # Wrap and send chunk
                tx_frame = KISS.wrap_image_chunk(0, current_chunk_id, payload, VR_PID_IMAGE_DOWNLOAD)
                ser_data.write(tx_frame)
                
                bytes_sent += len(payload)
                current_chunk_id += 1
                print(f"\r  {C.MAGENTA}XFER{C.RESET} Progress: {bytes_sent/total_size*100:5.1f}%", end="")

        # Step 4: Finalize
        done_frame = KISS.wrap_frame(PAYLOAD_ID_VR, VR_PID_DOWNLOAD_DONE, b'', 0x00)
        ser_data.write(done_frame)
        
        elapsed = time.monotonic() - start_time if start_time else 0
        log("OK", f"Transfer Complete! Rate: {(total_size*8)/elapsed/1000:.2f} kbps", C.GREEN)
        append_metrics(target_file, chunk_size, total_size, elapsed)
        
        ser_data.close()
        return True

    except Exception as e:
        log("FAIL", f"Test interrupted: {e}", C.RED)
        return False

def recv_kiss_frame(ser):
    """Strictly captures FEND (0xC0) delimited frames."""
    raw = b""
    while True:
        char = ser.read(1)
        if not char: return None
        raw += char
        if len(raw) > 1 and char == b'\xc0' and raw[0] == 0xc0:
            return raw

if __name__ == "__main__":
    print(f"\n{C.BOLD}{C.CYAN}=== Automated VR Image Transfer Test ==={C.RESET}\n")
    for filename in FILES_TO_TEST:
        for size in CHUNK_SIZES:
            run_test_cycle(filename, size)
            time.sleep(2) # Allow DUT to finish f_close and unmount