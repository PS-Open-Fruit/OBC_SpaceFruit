import serial
import time
import csv
import os

# ตั้งค่า Serial port
ser = serial.Serial('COM6', baudrate=9600, timeout=1)

# ไฟล์ CSV สำหรับเก็บผล
csv_file = "./python_usb_cdc/test_tx.csv"

# ถ้าไฟล์ยังไม่ถูกสร้าง ให้สร้างพร้อม header
if not os.path.exists(csv_file):
    with open(csv_file, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Message", "Elapsed_time_sec"])

# ส่งข้อความ "try_more" ก่อนเริ่มวัด
ser.write(b"try_more\n")
print("ส่ง try_more ไปแล้ว รอข้อความตอบกลับ...")

start_time = None
end_time = None
message = b""

while True:
    data = ser.read(1)
    if data:
        if start_time is None:
            # เริ่มจับเวลาเมื่อเจอ byte แรกของข้อความตอบกลับ
            start_time = time.time()

        message += data

        if data == b'\n':  # สมมติว่าข้อความจบด้วย newline
            end_time = time.time()
            elapsed = end_time - start_time
            text = message.decode(errors='ignore').strip()
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

            # แสดงผลบนหน้าจอ
            print(f"[{timestamp}] ข้อความ: {text} | ใช้เวลา: {elapsed:.6f} วินาที")

            # บันทึกลง CSV
            with open(csv_file, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, text, f"{elapsed:.6f}"])

            # reset สำหรับข้อความถัดไป
            message = b""
            start_time = None
            end_time = None
