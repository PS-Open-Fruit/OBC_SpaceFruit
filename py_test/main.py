import serial

ser = serial.Serial("/dev/ttyACM0")

ser.write(b"abcdefghijklmno")

ser.close()