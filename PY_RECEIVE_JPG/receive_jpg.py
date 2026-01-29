import serial
import time
import base64
import os
import sys
import zlib

# Configuration matches STM32 LPUART1
PORT = "COM5"
BAUDRATE = 209700 # Updated to match STM32 configuration
OUTPUT_DIR = "received_images"

def main():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    print(f"Connecting to {PORT} at {BAUDRATE}...")
    try:
        # Use 7-bit simulation if needed, but pyserial handles bytes. 
        # Since we send Base64 (ASCII), standard 8-N-1 or 7-N-1 works fine.
        ser = serial.Serial(PORT, BAUDRATE, timeout=2)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    print("Port opened. Waiting 3 seconds to stabilize...")
    time.sleep(3)
    
    print("Waiting for data...")
    
    receiving = False
    f_out = None
    crc_calculated = 0
    crc_received = None
    save_path = ""

    try:
        total_size = 0
        current_bytes_written = 0
        start_time = 0

        while True:
            if ser.in_waiting > 0:
                # Read raw line
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                except:
                    continue
                
                if not line:
                    continue

                if line.startswith("START:"):
                    # Format: START:filename:size
                    parts = line.split(":")
                    if len(parts) >= 3:
                        original_filename = parts[1]
                        try:
                            total_size = int(parts[2])
                        except:
                            total_size = 0
                        
                        save_name = "receive_jpg.jpg"
                        print(f"\n[RX] Start Receiving: {original_filename} -> {save_name} (Size: {total_size})")
                        
                        save_path = os.path.join(OUTPUT_DIR, save_name)
                        f_out = open(save_path, "wb")
                        
                        current_bytes_written = 0
                        crc_calculated = 0 # Initial CRC value
                        crc_received = None # Reset received CRC
                        start_time = time.time()
                        receiving = True
                
                elif line.startswith("CHECKSUM:"):
                     # Format: CHECKSUM:0xABC12345
                     try:
                         val_str = line.split(":")[1]
                         crc_received = int(val_str, 16)
                         print(f"\n[RX] Received Checksum: 0x{crc_received:08X}")
                     except:
                         print("\n[RX] Warning: Could not parse checksum")

                elif line == "EXPORT_COMPLETE":
                    if receiving and f_out:
                        f_out.close()
                        f_out = None
                        
                        end_time = time.time()
                        duration = end_time - start_time
                        
                        print(f"\n[RX] Transfer Complete.")
                        print(f"Total Bytes Saved: {current_bytes_written}")
                        if duration > 0:
                            speed = (current_bytes_written / 1024) / duration
                            print(f"Time: {duration:.2f} s")
                            print(f"Speed: {speed:.2f} KB/s")
                        
                        # Verify Size
                        if total_size > 0 and current_bytes_written < total_size:
                             print(f"Warning: Lost {total_size - current_bytes_written} bytes ({100 - (current_bytes_written/total_size)*100:.1f}%) during transfer.")
                        
                        # Verify CRC
                        print(f"Calculated CRC (zlib): 0x{crc_calculated:08X}")
                        
                        if crc_received is not None:
                            if crc_received == crc_calculated:
                                print("CRC CHECK: PASS (Standard zlib match)")
                            else:
                                print(f"CRC CHECK: FAIL (Expected: 0x{crc_received:08X}, Got: 0x{crc_calculated:08X})")
                                print("Note: Mismatch might be due to CRC variant differences (Reflected vs Non-reflected).")
                                
                    receiving = False
                
                elif receiving:
                    # Decode Line Immediately
                    try:
                        # Attempt decode
                        chunk_data = base64.b64decode(line)
                        f_out.write(chunk_data)
                        current_bytes_written += len(chunk_data)
                        
                        # Update CRC
                        crc_calculated = zlib.crc32(chunk_data, crc_calculated)

                        # UI Update
                        if total_size > 0:
                             pct = (current_bytes_written / total_size) * 100
                             if current_bytes_written % 4096 == 0: # Update less frequently
                                 sys.stdout.write(f"\rProgress: {pct:.1f}%")
                                 sys.stdout.flush()
                                 
                    except Exception as e:
                        # If a single line is corrupted, we skip it but keep the file open
                        pass
            else:
                 # Only sleep if no data waiting, to maximize throughput
                 time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
