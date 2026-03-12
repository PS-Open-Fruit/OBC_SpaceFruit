import serial
import time
import sys
import os
from config import CHUNK_SIZE
from kiss_protocol import KISSProtocol as KISS

PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_GET_IMAGE_REQUEST    = 0x02
VR_PID_GET_IMAGE_DOWNLOAD   = 0x03

PID_ACK                     = 0xAC

current_dir = os.path.dirname(os.path.abspath(__file__))

SERIAL_PORT = '/dev/ttys004'
BAUD_RATE = 115200
FILE_TO_SAVE = 'out.jpg' 

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
except Exception as e:
    print(f"Error opening serial port: {e}")

# Add this near the top of your file, outside of any functions
serial_buffer = bytearray()

# def recv_whole_frame(timeout_sec=2.0) -> bytes | None:
#     """
#     Reads from the serial port into a persistent buffer until a valid 
#     KISS frame (bounded by FENDs) is found, or until it times out.
#     """
#     global serial_buffer
#     start_time = time.time()

#     while True:
#         # 1. Pull all currently available bytes into our persistent buffer
#         if ser.in_waiting > 0:
#             serial_buffer.extend(ser.read(ser.in_waiting))
        
#         # 2. Search the buffer for a complete frame
#         try:
#             # Find the start FEND
#             start_idx = serial_buffer.index(KISS.FEND)
            
#             # Find the end FEND (starting the search one byte after start_idx)
#             end_idx = serial_buffer.index(KISS.FEND, start_idx + 1)
            
#             # Edge Case: Back-to-back FENDs (0xC0 0xC0). 
#             # KISS uses these to flush the line. They are empty frames.
#             if end_idx == start_idx + 1:
#                 serial_buffer.pop(start_idx) # Discard the extra FEND
#                 continue # Keep looking

#             # Extract the exact frame, including both start and end FENDs
#             frame = bytes(serial_buffer[start_idx : end_idx + 1])
            
#             # Remove the extracted frame AND any corrupted junk bytes that 
#             # might have arrived before the start_idx
#             del serial_buffer[:end_idx + 1]
            
#             return frame
            
#         except ValueError:
#             # .index() raises ValueError if it can't find the FEND byte.
#             # This means we only have a partial frame. We pass and keep reading.
#             pass
        
#         # 3. Timeout check to prevent deadlocks
#         if (time.time() - start_time) > timeout_sec:
#             return None
        
#         # Small sleep to yield CPU context and prevent locking up the thread
#         time.sleep(0.01)

def recv_whole_frame():
    whole_kiss = False
    recv_frame : bytes = b""

    while not whole_kiss:
        if ser.in_waiting:
            print("Reply")
            recv_frame += (ser.read(ser.in_waiting))
            try:
                start = recv_frame.index(KISS.FEND)
                end = recv_frame.index(KISS.FEND,start + 1)
                whole_kiss = True
            except:
                pass
    return recv_frame

def sendRequestTransferCommand():
    file_request_structure = b''
    file_request_structure += 0x00.to_bytes(1,"big") # file id
    file_request_structure += 0xFFFF.to_bytes(2,"big") # chunk id, 0xFFFF means dump everything
    req = KISS.wrap_frame(PAYLOAD_ID_VR,VR_PID_GET_IMAGE_REQUEST,file_request_structure)
    ser.write(req)

    while True:
        recv_frame = recv_whole_frame()
        decoded = KISS.unwrap_frame(recv_frame)
        if (decoded['type'] == 'generic'):
            if decoded['payload_id'] == PAYLOAD_ID_VR:
                print("VR Payload Reply\r\n")
                if decoded['pid'] == PID_ACK:
                    print("Acknowledged\r\n")

        if (decoded['type'] == 'image'):
            if decoded['pid'] == VR_PID_GET_IMAGE_DOWNLOAD:
                _file_id = decoded['file_id']
                _chunk_id = decoded['chunk_id']
                _content = decoded["content"]
                print(f"chunk_id : {_chunk_id}")
                if (_chunk_id == 0):
                    file = open(f"{_file_id}.jpg","wb")
                else:
                    file = open(f"{_file_id}.jpg","a+b")
                file.write(_content)
                file.close()
                print(f"Write {_file_id} {_chunk_id} {len(_content)}\r\n")
                print("ACK to VR\r\n")
                ack = KISS.wrap_frame(PAYLOAD_ID_VR,PID_ACK,b'')
                ser.write(ack)

def run():
    # print("\033c")
    time.sleep(1)
    sendRequestTransferCommand()

run()


