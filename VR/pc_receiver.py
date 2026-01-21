import serial
import struct
import time
import sys

# ==========================================
# 1. Config
# ==========================================
# Change this to your ST-Link Virtual COM Port
PC_PORT = 'COM9' 
BAUDRATE = 115200

# ==========================================
# 2. Main Receiver Logic
# ==========================================

def run_image_downloader():
    try:
        ser = serial.Serial(PC_PORT, BAUDRATE, timeout=1.0)
        print(f"✅ Connected to PC UART Receiver on {PC_PORT}")
    except Exception as e:
        print(f"❌ Error: {e}")
        return

    print("--- 📡 Waiting for Image Download... ---")
    
    downloading = False
    current_img_data = bytearray()
    expected_size = 0
    current_chunk = 0
    
    while True:
        try:
            # We are reading line-by-line because main.c uses printf() with \r\n
            # This is slow/inefficient for binary but works for this specific demo setup
            line = ser.readline()
            
            if not line: raise TimeoutError # Just loop if empty
            
            # Decode text line (ignore decoding errors for partial binary noise)
            text = line.decode('utf-8', errors='ignore').strip()
            
            # 1. Detect Start of Download
            if "[OBC] Image Captured! Size:" in text:
                # Parse Size: "[OBC] Image Captured! Size: 153000 bytes..."
                try:
                    parts = text.split("Size: ")[1].split(" ")
                    expected_size = int(parts[0])
                    print(f"\n📥 DETECTED NEW IMAGE! Expected Size: {expected_size/1024:.2f} KB")
                    
                    downloading = True
                    current_img_data = bytearray()
                    current_chunk = 0
                    start_time = time.time()
                except:
                    print(f"⚠️ Failed to parse size from: {text}")

            # 2. Detect Chunk Data
            elif text.startswith("[DATA]"):
                if not downloading:
                    print("\n⚠️ Missed Start Header! Auto-starting download (Unknown Size)...")
                    downloading = True
                    expected_size = 0 
                    current_img_data = bytearray()
                    start_time = time.time()
                
                # Example: "[DATA] FF D8 FF E0 ..."
                try:
                    hex_str = text.replace("[DATA]", "").strip()
                    chunk_bytes = bytes.fromhex(hex_str)
                    current_img_data.extend(chunk_bytes)
                    
                    # Calculate Stats
                    elapsed = time.time() - start_time
                    speed = (len(current_img_data) / 1024) / elapsed if elapsed > 0.5 else 0.0
                    
                    if expected_size > 0:
                        pct = len(current_img_data) / expected_size * 100.0
                        # Draw Bar: [==========          ]
                        bar_len = 20
                        filled = int(bar_len * pct / 100)
                        bar = "=" * filled + " " * (bar_len - filled)
                        
                        print(f"\r   [{bar}] {pct:.1f}% | {len(current_img_data)/1024:.1f} KB | {speed:.1f} KB/s    ", end="")
                    else:
                         print(f"\r   Downloading... {len(current_img_data)/1024:.1f} KB | {speed:.1f} KB/s    ", end="")
                         
                except ValueError:
                    print(f"⚠️ Corrupt Hex Data: {text[:20]}...")

            # 3. Detect End
            elif "[OBC] Download Complete!" in text:
                print(f"\n✅ DOWNLOAD FINISHED!")
                filename = f"received_image_{int(time.time())}.jpg"
                with open(filename, "wb") as f:
                    f.write(current_img_data)
                print(f"💾 Saved to: {filename}")
                downloading = False

            # Else: Print debug logs
            else:
                if text: print(f"DEV: {text}")

        except Exception as e:
             if "Timeout" not in str(e):
                print(f"Error reading UART: {e}")
             pass 

if __name__ == "__main__":
    run_image_downloader()
