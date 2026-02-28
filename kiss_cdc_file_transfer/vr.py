import serial
import time
import sys
import os
from kiss_protocol import KISSProtocol as KISS
from config import CHUNK_SIZE


PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_GET_IMAGE_REQUEST    = 0x02
VR_PID_GET_IMAGE_DOWNLOAD   = 0x03

PID_ACK                     = 0xAC

FILE_MODE_IDLE = 0xDE
FILE_MODE_DUMP = 0x00
FILE_MODE_WAIT_ACK = 0xAB
FILE_MODE_CHUNK = 0x01

current_dir = os.path.dirname(os.path.abspath(__file__))

SERIAL_PORT = '/dev/ttys005'
BAUD_RATE = 115200
# FILE_TO_SAVE = 'file.txt' 
FILE_TO_SAVE = '02191444.jpg' 

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



try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
except Exception as e:
    print(f"Error opening serial port: {e}")

def getCaptureRequest():
    PAYLOAD_VR = 0x01
    PAYLOAD_VR_PID_CAPTURE = 0x01
    file_mode = FILE_MODE_IDLE
    current_chunk_id = 0
    current_file_id = 0
    while (True):
        # print(f"file_mode {file_mode:02X}")

        if (file_mode != FILE_MODE_IDLE):
            if (file_mode == FILE_MODE_DUMP):
                # print("Dump File Mode")
                transfer_content : bytes = b''
                with open(FILE_TO_SAVE,"rb") as image:
                    data = image.read()
                    print(len(data))
                    transfer_content = data[current_chunk_id * CHUNK_SIZE:(current_chunk_id + 1) * CHUNK_SIZE]
                    print(current_chunk_id)
                    print(f"{current_chunk_id * CHUNK_SIZE}.  {len(data)}")

                    if (current_chunk_id * CHUNK_SIZE <= len(data)):
                        print("sending")
                        # print(f"transfer content ep 2 : {transfer_content}")
                        reply_frame = KISS.wrap_image_chunk(current_file_id,current_chunk_id,transfer_content,VR_PID_GET_IMAGE_DOWNLOAD)
                        ser.write(reply_frame)
                        file_mode = FILE_MODE_WAIT_ACK
                        print("Waiting for ack")
                    else:
                        print("sending done")
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
                    if file_mode == FILE_MODE_WAIT_ACK:
                        file_mode = FILE_MODE_DUMP
                        print(f"ACK File Transfer {current_chunk_id}")
                        current_chunk_id = current_chunk_id + 1

                elif frame['pid'] == VR_PID_GET_IMAGE_CAPTURE:
                    print("Capture command")
                    reply = KISS.wrap_frame(PAYLOAD_ID_VR,PID_ACK,b'')
                    ser.write(reply)

                elif frame['pid'] == VR_PID_GET_IMAGE_REQUEST:
                    filePath = f"{current_dir}/{FILE_TO_SAVE}"
                    print(f"File Path {filePath} {os.path.getsize(filePath)}")
                    reply = KISS.wrap_frame(PAYLOAD_ID_VR,PID_ACK,b'')
                    ser.write(reply)
                    print(f"Frame : {frame}")
                    if (frame['data_len'] == 3):
                        _file_id = frame['data'][0]
                        _chunk_id = frame['data'][1:3]
                        current_file_id = _file_id
                        requested_chunk = int.from_bytes(_chunk_id,"big") 
                        if (requested_chunk == 0xFFFF):
                            current_chunk_id = 0
                            file_mode = FILE_MODE_DUMP
                        else:
                            requested_chunk = requested_chunk
                            file_mode = FILE_MODE_CHUNK



            # print(frame)
    # ser.write(frame)
    # print(f"0x{frame:02X}")

def run():
    getCaptureRequest()

run()


