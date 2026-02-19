import serial
import time
import sys
import os

# Add Shared to path
current_dir = os.path.dirname(os.path.abspath(__file__))
shared_path = os.path.abspath(os.path.join(current_dir, '..', 'Shared', 'Python'))
if shared_path not in sys.path:
    sys.path.append(shared_path)

try:
    from kiss_protocol import KISSProtocol
except ImportError:
    # Fallback if shared path is different or not found
    print("Warning: Could not import KISSProtocol. Checking relative path...")
    shared_path = os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'OBC_SpaceFruit', 'Shared', 'Python')
    sys.path.append(shared_path)
    try:
        from kiss_protocol import KISSProtocol
    except ImportError:
         print("Error: Could not import KISSProtocol.")
         sys.exit(1)

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
FILE_TO_SEND = '02191444.jpg' 

def send_image_to_sd(ser, filename):
    print(f"\n[UPLOAD] Starting upload of {filename} to STM32 SD Card...")
    filepath = os.path.join(current_dir, filename)
    
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        return

    try:
        with open(filepath, 'rb') as f:
            data = f.read()
            
        # 1. Send START command
        # Format expected by SD_SaveFiles: "START:<filename>"
        cmd = f"START:{filename}".encode('utf-8')
        ser.write(cmd)
        print(f"[TX] Sent Command: {cmd}")
        
        # Wait a bit for STM32 to open file and be ready (it might send OK, but we can just wait briefly)
        time.sleep(0.5) 
        
        # 2. Send Data
        chunk_size = 64 # Safe chunk size for USB CDC
        total_len = len(data)
        sent = 0
        
        while sent < total_len:
            end = min(sent + chunk_size, total_len)
            chunk = data[sent:end]
            ser.write(chunk)
            sent = end
            
            # Small delay to ensure STM32 buffer processing (SD write takes time)
            # sd_util.c writes to SD in the main loop or interrupt context? 
            # SD_SaveFiles is in main loop. USB interrupt fills buffer.
            # If we fill buffer faster than SD write, we might lose data if flow control isn't perfect.
            # 5ms delay per 64 bytes is ~12KB/s. Safe start.
            time.sleep(0.005) 
            
            if sent % 1024 == 0 or sent == total_len:
                sys.stdout.write(f"\r[TX] Sent {sent}/{total_len} bytes")
                sys.stdout.flush()
            
        print("\n[UPLOAD] Upload Complete. Waiting for timeout on STM32 side to close file.")
        
    except Exception as e:
        print(f"[ERROR] {e}")

def main():
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    print("Listening for Trigger (KISS CMD 0x12)...")
    
    rx_buffer = bytearray()
    
    while True:
        try:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                rx_buffer.extend(chunk)
                
                # Look for KISS Frame
                while KISSProtocol.FEND in rx_buffer:
                    try:
                        fend_idx = rx_buffer.index(KISSProtocol.FEND)
                        
                        if fend_idx == 0:
                             # Check if we have a second FEND
                             try:
                                 next_fend = rx_buffer.index(KISSProtocol.FEND, 1)
                                 frame = rx_buffer[0:next_fend+1]
                                 
                                 # Attempt Decode
                                 # We are looking for Command 0x12
                                 # KISS Command Byte is the first byte after FEND (if not escaped)
                                 # But KISSProtocol.unwrap_frame handles escaping.
                                 
                                 result = KISSProtocol.unwrap_frame(frame)
                                 if result:
                                     cmd, payload = result
                                     if cmd == 0x12:
                                         print(f"\n[RX] Trigger Received! (CMD 0x12)")
                                         # Clear buffer to ensure we don't process old data or re-trigger
                                         rx_buffer.clear() 
                                         
                                         # Send the Image
                                         send_image_to_sd(ser, FILE_TO_SEND)
                                         
                                         print("Resuming Listener...")
                                         break # Break inner loop to refill buffer fresh
                                 
                                 # Remove processed frame
                                 del rx_buffer[0:next_fend+1]
                                 
                             except ValueError:
                                 # No second FEND yet, wait for more data
                                 break
                        else:
                            # Trim garbage before FEND
                            del rx_buffer[0:fend_idx]
                            
                    except ValueError:
                        break
                        
            time.sleep(0.005)
            
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    ser.close()

if __name__ == "__main__":
    main()
