import serial
import time
import sys
import os
from config import CHUNK_SIZE
import kiss_protocol

PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_GET_IMAGE_REQUEST    = 0x02
VR_PID_GET_IMAGE_DOWNLOAD   = 0x03

PID_ACK                     = 0xAC

current_dir = os.path.dirname(os.path.abspath(__file__))

SERIAL_PORT = '/dev/ttys005'
BAUD_RATE = 115200
FILE_TO_SAVE = 'out.jpg' 

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
except Exception as e:
    print(f"Error opening serial port: {e}")

def sendRequestTransferCommand():
    frame = kiss_protocol.KISSProtocol.wrap_frame(PAYLOAD_ID_VR,VR_PID_GET_IMAGE_REQUEST,0x00.to_bytes(1))
    # frame = kiss_protocol.KISSProtocol.wrap_frame(PAYLOAD_ID_VR,VR_PID_GET_IMAGE_CAPTURE,b'')
    print(len(frame))
    ser.write(frame)
    while True:
        while (ser.in_waiting):
            # print("\033c")
            # print("\r\n")
            print(ser.in_waiting)
            recv = ser.read(ser.in_waiting)
            # print(ser.read(ser.in_waiting))
            decoded = kiss_protocol.KISSProtocol.unwrap_frame(recv)
            if (decoded['type'] == 'generic'):
                if decoded['payload_id'] == PAYLOAD_ID_VR:
                    print("VR Payload Reply\r\n")
                    if decoded['pid'] == PID_ACK:
                        print("Acknowledged\r\n")
            if (decoded['type'] == 'image'):
                if decoded['pid'] == VR_PID_GET_IMAGE_DOWNLOAD:
                    # "type": "image",
                    # "command": command,
                    # "payload_id": payload_id,
                    # "pid": pid,
                    # "file_id": file_id,
                    # "chunk_id": chunk_id,
                    # "content": content
                    _file_id = decoded['file_id']
                    _chunk_id = decoded['chunk_id']
                    _content = decoded["content"]
                    with open(f"{_file_id}.txt","a+b") as file:
                        if (_chunk_id == 0):
                            file.seek(0)
                        #     file.seek(_chunk_id * CHUNK_SIZE)
                        file.write(_content)
                        print(f"Write {_file_id} {_chunk_id} {len(_content)}\r\n")
                        # time.sleep(0.2)
                    # time.sleep(1)
                    print("ACK to VR\r\n")
                    ack = kiss_protocol.KISSProtocol.wrap_frame(PAYLOAD_ID_VR,PID_ACK,b'')
                    ser.write(ack)

    # print(f"0x{frame:02X}")

def run():
    print("\033c")
    time.sleep(1)
    sendRequestTransferCommand()

run()


