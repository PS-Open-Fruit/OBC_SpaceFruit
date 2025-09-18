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
CSV_FILE = "./python_usb_cdc/test_rx.csv"
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

ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)

for size in TEST_SIZES:
    for trial in range(NUM_TRIALS):
        msg = random_message(size)
        data = (msg + "\n").encode()

        ser.reset_input_buffer()
        start_time = time.time()
        ser.write(data)

        # รับข้อความกลับ
        received = ser.readline().decode(errors="ignore").strip()
        end_time = time.time()
        elapsed = end_time - start_time

        success = (received == msg)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        print(f"[{timestamp}] Size={size} | Time={elapsed:.6f}s | Match={success}")
        print(f"   Sent:     {msg}")
        print(f"   Received: {received}")

        # บันทึกลง CSV
        with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                size,
                msg,
                received,
                f"{elapsed:.6f}",
                success
            ])

        time.sleep(DELAY_BETWEEN)

ser.close()
