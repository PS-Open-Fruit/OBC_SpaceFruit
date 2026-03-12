import serial
import time
import sys
import os
from kiss_protocol import KISSProtocol as KISS
from config import CHUNK_SIZE

PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_IMAGE_REQUEST        = 0x02
VR_PID_IMAGE_DOWNLOAD       = 0x03
VR_PID_IMAGE_DOWNLOAD_DONE  = 0x05

PID_ACK                     = 0xAC

FILE_MODE_IDLE = 0xDE
FILE_MODE_DUMP = 0x00
FILE_MODE_WAIT_ACK = 0xAB
FILE_MODE_CHUNK = 0x01

current_dir = os.path.dirname(os.path.abspath(__file__))

# Safely read the comport file
try:
    with open("comport.txt", 'r') as file:
        SERIAL_PORT = file.read().replace("\n", "")
except FileNotFoundError:
    SERIAL_PORT = '/dev/ttys005'
    
print(f"Using port: {SERIAL_PORT}")
BAUD_RATE = 115200
FILE_TO_SAVE = 'image2.jpg' 

# Global serial object
ser = None

# --- Transfer stats ---
transfer_start_time: float = 0.0
transfer_bytes_sent: int = 0

def connect_serial():
    """Attempts to connect to the serial port, retrying until successful."""
    global ser
    while True:
        try:
            if ser and ser.is_open:
                ser.close()
            
            print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
            print("Serial connection established successfully!")
            return
        except (serial.SerialException, OSError) as e:
            print(f"Connection failed: {e}. Retrying in 2 seconds...")
            time.sleep(2)

def send_data(data: bytes):
    """Wraps serial write with an auto-reconnect if the port drops."""
    global ser
    while True:
        try:
            ser.write(data)
            return
        except (serial.SerialException, OSError) as e:
            print(f"Connection lost during write: {e}")
            connect_serial()

def recv_whole_frame():
    """Reads a whole KISS frame, reconnecting if the port drops."""
    global ser
    recv_frame : bytes = b""

    while True:
        try:
            if ser.in_waiting:
                print("Reply")
                recv_frame += (ser.read(ser.in_waiting))
                try:
                    start = recv_frame.index(KISS.FEND)
                    end = recv_frame.index(KISS.FEND, start + 1)
                    return recv_frame
                except ValueError:
                    pass
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError) as e:
            print(f"Connection lost during read: {e}")
            connect_serial()
            recv_frame = b""

def print_transfer_summary(total_bytes: int, elapsed: float):
    """Prints a formatted summary of the completed transfer."""
    if elapsed <= 0:
        elapsed = 0.001  # Avoid division by zero

    rate_bps  = (total_bytes * 8) / elapsed          # bits per second
    rate_kbps = rate_bps / 1_000                      # kilobits per second
    rate_KBps = total_bytes / elapsed / 1_024          # kibibytes per second

    print("\n" + "=" * 50)
    print("         FILE TRANSFER SUMMARY")
    print("=" * 50)
    print(f"  Total data sent : {total_bytes:,} bytes ({total_bytes / 1_024:.2f} KiB)")
    print(f"  Transfer time   : {elapsed:.3f} seconds")
    print(f"  Data rate       : {rate_kbps:.1f} kbps  ({rate_KBps:.2f} KiB/s)")
    print("=" * 50 + "\n")

def getCaptureRequest():
    global transfer_start_time, transfer_bytes_sent

    PAYLOAD_VR = 0x01
    PAYLOAD_VR_PID_CAPTURE = 0x01
    file_mode = FILE_MODE_IDLE
    current_chunk_id = 0
    current_file_id = 0
    
    while True:
        if (file_mode != FILE_MODE_IDLE):
            if (file_mode == FILE_MODE_DUMP):
                transfer_content : bytes = b''
                with open(FILE_TO_SAVE,"rb") as image:
                    data = image.read()
                    print(len(data))
                    transfer_content = data[current_chunk_id * CHUNK_SIZE:(current_chunk_id + 1) * CHUNK_SIZE]
                    print(current_chunk_id)
                    print(f"{current_chunk_id * CHUNK_SIZE}.  {len(data)}")

                    if (current_chunk_id * CHUNK_SIZE <= len(data)):
                        print(f"sending chunk {current_chunk_id}")

                        # Start the timer on the very first chunk
                        if current_chunk_id == 0:
                            transfer_start_time = time.monotonic()
                            transfer_bytes_sent = 0
                            print("[TIMER] Transfer started.")

                        reply_frame = KISS.wrap_image_chunk(current_file_id, current_chunk_id, transfer_content, VR_PID_IMAGE_DOWNLOAD)
                        send_data(reply_frame)
                        transfer_bytes_sent += len(transfer_content)

                        file_mode = FILE_MODE_WAIT_ACK
                        print(f"Waiting for chunk {current_chunk_id} ack")
                    else:
                        print(f"sending file done")
                        reply_frame = KISS.wrap_frame(PAYLOAD_ID_VR,VR_PID_IMAGE_DOWNLOAD_DONE,b'',0x00)
                        send_data(reply_frame)

                        # Stop timer and print summary
                        elapsed = time.monotonic() - transfer_start_time
                        print_transfer_summary(transfer_bytes_sent, elapsed)

                        file_mode = FILE_MODE_IDLE
                        current_chunk_id = 0
                        current_file_id = 0
                    
        buf = recv_whole_frame()
        frame = KISS.unwrap_frame(buf)
        
        if (frame['type'] == 'generic'):
            if frame['payload_id'] == PAYLOAD_ID_VR:
                if frame['pid'] == VR_PID_GET_STATUS:
                    print("Still no reply")
                elif frame['pid'] == PID_ACK:
                    print("PID ACK")
                    if file_mode == FILE_MODE_WAIT_ACK:
                        file_mode = FILE_MODE_DUMP
                        print(f"ACK File Transfer {current_chunk_id}")
                        current_chunk_id = current_chunk_id + 1

                elif frame['pid'] == VR_PID_GET_IMAGE_CAPTURE:
                    print("Capture command")
                    reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
                    send_data(reply)

                elif frame['pid'] == VR_PID_IMAGE_REQUEST:
                    filePath = f"{current_dir}/{FILE_TO_SAVE}"
                    print(f"File Path {filePath} {os.path.getsize(filePath)}")
                    reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
                    send_data(reply)
                    print(f"Frame : {frame}")
                    
                    if (frame['data_len'] == 3):
                        _file_id = frame['data'][0]
                        _chunk_id = frame['data'][1:3]
                        current_file_id = _file_id
                        requested_chunk = int.from_bytes(_chunk_id, "big") 
                        
                        if (requested_chunk == 0xFFFF):
                            current_chunk_id = 0
                            file_mode = FILE_MODE_DUMP
                        else:
                            requested_chunk = requested_chunk
                            file_mode = FILE_MODE_CHUNK

def run():
    connect_serial()
    getCaptureRequest()

if __name__ == "__main__":
    run()