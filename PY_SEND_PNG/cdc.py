import serial
import serial.tools.list_ports
import time
import os
import sys

# Configuration
# For USB CDC, Baudrate is ignored by the device, but needed for pyserial to open the port.
BAUDRATE = 115200 
TIMEOUT = 2
CHUNK_SIZE = 4096 # Send in 4KB chunks
TARGET_FILE = 'Screenshot 2025-12-03 135930.png'

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def send_file(serial_port, file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    file_size = os.path.getsize(file_path)
    print(f"Opening USB CDC port {serial_port}...")
    
    try:
        with serial.Serial(serial_port, BAUDRATE, timeout=TIMEOUT) as ser:
            print(f"Sending '{file_path}' ({file_size} bytes) via USB CDC...")
            
            # Allow some time for connection
            time.sleep(1) 
            
            start_time = time.time()
            bytes_sent = 0
            
            with open(file_path, 'rb') as f:
                while True:
                    chunk = f.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    
                    ser.write(chunk)
                    # ser.flush()
                    bytes_sent += len(chunk)
                    
                    # Calculate progress
                    progress = (bytes_sent / file_size) * 100
                    sys.stdout.write(f"\rProgress: {progress:.1f}% ({bytes_sent}/{file_size} bytes)")
                    sys.stdout.flush()
                    
                    # Delay to prevent buffer overflow on the device
                    # Even though USB is fast, SD Card write speed via SPI is the bottleneck.
                    # 4KB write takes ~10-50ms depending on card/driver.
                    time.sleep(0.05) 
                    
            end_time = time.time()
            duration = end_time - start_time
            speed = (bytes_sent / 1024) / duration if duration > 0 else 0
            
            print(f"\n\nDone!")
            print(f"Wrote: {bytes_sent} bytes")
            print(f"Time: {duration:.2f} s")
            print(f"Speed: {speed:.2f} KB/s")
            
            # Read and print responses/debug from device
            print("\nListening for Device response (Ctrl+C to stop)...")
            start_listen = time.time()
            while (time.time() - start_listen) < 5: 
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"[Device]: {line}")
                    except:
                        pass
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"\nSerial Error: {e}")
    except Exception as e:
        print(f"\nError: {e}")

def main():
    # 1. Port Selection (Hardcoded to COM3)
    selected_port = 'COM3'
    print(f"Using Port: {selected_port}")

    # 3. Send File directly
    print(f"\nTarget File: {TARGET_FILE}")
    send_file(selected_port, TARGET_FILE)

if __name__ == "__main__":
    main()
