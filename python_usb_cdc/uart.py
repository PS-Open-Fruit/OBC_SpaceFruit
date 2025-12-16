import serial
import time

# กำหนดพอร์ตและ baudrate ตามที่ใช้งานจริง
ser = serial.Serial(
    port='COM5',      # หรือ '/dev/ttyUSB0' บน Linux/Raspberry Pi
    baudrate=9600,
    timeout=1
)

try:
    while True:
        ser.write(b'A')   # ส่งตัวอักษร A (ต้องเป็น byte)
        print("Sent: A")
        time.sleep(2)     # หน่วงเวลา 5 วินาที
except KeyboardInterrupt:
    print("Stopped by user")
finally:
    ser.close()
