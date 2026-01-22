import serial
import serial.tools.list_ports
import time
import os
import sys
from datetime import datetime

# Configuration
# For USB CDC, Baudrate is ignored by the device, but needed for pyserial to open the port.
BAUDRATE = 115200 
TIMEOUT = 2
CHUNK_SIZE = 4096 # Send in 4KB chunks
TARGET_FILE = 'Panorama_of_3mb.jpg'

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
            
            # --- Protocol: 1. Send Filename (With Fallback) ---
            filename = os.path.basename(file_path)
            # Use MMDDHHMM.jpg to ensure 8.3 compatibility (Max 8 chars filename)
            timestamp_name = datetime.now().strftime("%m%d%H%M.jpg")
            attempts = [(filename, "Original"), (timestamp_name, "Timestamp Fallback")]
            
            transfer_started = False
            
            for name_to_send, label in attempts:
                header = f"START:{name_to_send}".encode('utf-8')
                print(f"Sending Command ({label}): {header}")
                ser.write(header)
                
                # Wait for ACK
                print("Waiting for ACK...")
                ack_received = False
                start_wait = time.time()
                while (time.time() - start_wait) < 5: 
                    if ser.in_waiting > 0:
                        resp = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                        if "OK" in resp:
                            ack_received = True
                            print(f"ACK Received for '{name_to_send}'! Starting Transfer...")
                            break
                        if "ERR" in resp:
                             print(f"Device rejected filename '{name_to_send}'.")
                             break # Try next or fail
                    time.sleep(0.01)
                
                if ack_received:
                    transfer_started = True
                    break
            
            if not transfer_started:
                print("Failed to initialize file transfer (Device rejected all attempts). Aborting.")
                return

            # --- Protocol: 3. Send Data ---
            start_time = time.time()
            bytes_sent = 0
            
            with open(file_path, 'rb') as f:
                while True:
                    chunk = f.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    
                    ser.write(chunk)
                    bytes_sent += len(chunk)
                    
                    # Calculate progress
                    progress = (bytes_sent / file_size) * 100
                    sys.stdout.write(f"\rProgress: {progress:.1f}% ({bytes_sent}/{file_size} bytes)")
                    sys.stdout.flush()
                    
                    # Delay to prevent buffer overflow
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
