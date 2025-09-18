import serial
import time
import csv
import os
import random
import string

# ---------- CONFIG ----------
PORT = "COM6"
BAUDRATE = 9600
TIMEOUT = 2
CSV_FILE = "./python_usb_cdc/test_tx.csv"
TEST_SIZES = [32, 64, 128, 256]
NUM_TRIALS = 5
DELAY_BETWEEN = 3  # วินาที
# ----------------------------

def random_message(size: int) -> str:
    chars = string.ascii_letters + string.digits
    return "".join(random.choice(chars) for _ in range(size))

# สร้างไฟล์ CSV ถ้ายังไม่มี
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp",
            "Size",
            "SentMessage",
            "ReceivedMessage",
            "Elapsed_time_sec",
            "Success"
        ])

# เปิด Serial port
ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)

for size in TEST_SIZES:
    for trial in range(NUM_TRIALS):
        # สร้างข้อความสุ่มและ encode
        sent_msg = random_message(size)
        data = (sent_msg + "\n").encode()

        # เคลียร์ buffer ก่อนส่ง
        ser.reset_input_buffer()

        # ส่งข้อความ
        start_time = time.time()
        ser.write(data)

        # รอข้อความกลับ
        received_msg = ser.readline().decode(errors="ignore").strip()
        end_time = time.time()
        elapsed = end_time - start_time

        success = (received_msg == sent_msg)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        # แสดงผลใน console
        print(f"[{timestamp}] Size={size} | Time={elapsed:.6f}s | Match={success}")
        print(f"   Sent:     {sent_msg}")
        print(f"   Received: {received_msg}")

        # บันทึกลง CSV
        with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                size,
                sent_msg,
                received_msg,
                f"{elapsed:.6f}",
                success
            ])

        # หน่วง 3 วินาที
        time.sleep(DELAY_BETWEEN)

ser.close()
print(f"\nบันทึกผลทั้งหมดลง {CSV_FILE} เรียบร้อยแล้ว!")
