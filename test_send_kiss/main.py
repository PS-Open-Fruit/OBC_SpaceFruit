import serial
import time
import struct
import sys
import os
import threading

# Import KISS Protocol from Shared/Python
current_dir = os.path.dirname(os.path.abspath(__file__))
shared_path = os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'OBC_SpaceFruit', 'Shared', 'Python')
# Ensure correct path (adjusting for potential CWD differences)
# If script is in d:\github\OBC_SpaceFruit\test_send_kiss\main.py
# Shared is in d:\github\OBC_SpaceFruit\Shared\Python
shared_path = os.path.abspath(os.path.join(current_dir, '..', 'Shared', 'Python'))

if shared_path not in sys.path:
    sys.path.append(shared_path)

try:
    from kiss_protocol import KISSProtocol
except ImportError:
    print("Error: Could not import KISSProtocol. Check path.")
    sys.exit(1)

# Configuration
SERIAL_PORT = 'COM5' 
BAUD_RATE = 209700

# Global State
rx_buffer = bytearray()
image_chunks = {}
current_filename = ""
available_files = [] # List of filenames from List response
last_chunk_time = 0
running = True
window_mode = False
window_counter = 0
last_saved_chunk_time = 0

def send_command_12(ser):
    print("Sending Command 0x12...")
    cmd = 0x12
    # CRC calculated on [0x12]
    crc = KISSProtocol.calculate_crc(bytes([cmd]))
    print(f"CMD: {hex(cmd)}, CRC: {hex(crc)}")
    
    # Payload is just the CRC (Command is added by wrap_frame)
    payload_crc = struct.pack('<I', crc)
    
    # wrap_frame(payload, command) -> FEND [0x12] [CRC] FEND
    frame = KISSProtocol.wrap_frame(payload_crc, cmd)
    print(f"Sending Packet: {frame.hex().upper()}")
    
    ser.write(frame)

def send_command_start_file(ser, filename="01261311.jpg"):
    global window_mode
    print(f"Sending Command Start File (0x00 0x13) + File: {filename}...")
    cmd = 0x00
    sub_cmd = 0x13
    window_mode = False # Legacy mode
    
    # Encode and pad/truncate to exactly 12 bytes
    fname_bytes = filename.encode('utf-8')
    if len(fname_bytes) > 12:
        fname_bytes = fname_bytes[:12]
    elif len(fname_bytes) < 12:
        fname_bytes = fname_bytes + b'\x00' * (12 - len(fname_bytes))
    
    # CRC calculated on [0x00, 0x13] + [Filename 12 bytes]
    crc_data = bytes([cmd, sub_cmd]) + fname_bytes
    crc = KISSProtocol.calculate_crc(crc_data)
    print(f"CMD: {hex(cmd)} {hex(sub_cmd)}, File: {filename}, CRC: {hex(crc)}")
    
    # Payload is [0x13] + [Filename] + [CRC]
    payload = bytes([sub_cmd]) + fname_bytes + struct.pack('<I', crc)
    
    # wrap_frame -> FEND [0x00] [0x13] [Filename] [CRC] FEND
    frame = KISSProtocol.wrap_frame(payload, cmd)
    print(f"Sending Packet: {frame.hex().upper()}")
    
    ser.write(frame)

def send_command_windowed_start(ser, filename="01261311.jpg"):
    global window_mode, window_counter
    print(f"Sending Windowed Start (0x00 0x12) + File: {filename}...")
    cmd = 0x00
    sub_cmd = 0x12
    
    fname_bytes = filename.encode('utf-8')
    if len(fname_bytes) > 12:
        fname_bytes = fname_bytes[:12]
    elif len(fname_bytes) < 12:
        fname_bytes = fname_bytes + b'\x00' * (12 - len(fname_bytes))
    
    crc_data = bytes([cmd, sub_cmd]) + fname_bytes
    crc = KISSProtocol.calculate_crc(crc_data)
    print(f"CMD: {hex(cmd)} {hex(sub_cmd)}, File: {filename}, CRC: {hex(crc)}")
    
    payload = bytes([sub_cmd]) + fname_bytes + struct.pack('<I', crc)
    frame = KISSProtocol.wrap_frame(payload, cmd)
    print(f"Sending Packet: {frame.hex().upper()}")
    
    ser.write(frame)
    window_mode = True
    window_counter = 0

def send_command_list_files(ser):
    print("Sending Command List Files (0x00 0x10)...")
    cmd = 0x00
    sub_cmd = 0x10
    
    # CRC calculated on [0x00, 0x10]
    crc_data = bytes([cmd, sub_cmd])
    crc = KISSProtocol.calculate_crc(crc_data)
    print(f"CMD: {hex(cmd)} {hex(sub_cmd)}, CRC: {hex(crc)}")
    
    # Payload is [0x10] + [CRC]
    payload = bytes([sub_cmd]) + struct.pack('<I', crc)
    
    frame = KISSProtocol.wrap_frame(payload, cmd)
    print(f"Sending Packet: {frame.hex().upper()}")
    ser.write(frame)

def send_ack(ser):
    cmd = 0x00
    sub_cmd = 0xAC
    crc_data = bytes([cmd, sub_cmd])
    crc = KISSProtocol.calculate_crc(crc_data)
    
    payload = bytes([sub_cmd]) + struct.pack('<I', crc)
    frame = KISSProtocol.wrap_frame(payload, cmd)
    # print(f"[TX] ACK Sent: {frame.hex().upper()}")
    ser.write(frame)

def save_image():
    global image_chunks, current_filename
    if not image_chunks: return
    
    filename = f"received_{current_filename}" if current_filename else f"received_image_{int(time.time())}.jpg"
    print(f"\n[INFO] Saving {len(image_chunks)} chunks to {filename}...")
    
    sorted_ids = sorted(image_chunks.keys())
    
    with open(filename, "wb") as f:
        for cid in sorted_ids:
            f.write(image_chunks[cid])
            
    print(f"[SUCCESS] Saved {filename} ({os.path.getsize(filename)} bytes)\n")
    # Do NOT clear image_chunks here to strictly overwrite file with growing data.
    # image_chunks.clear() 

def process_frame(ser, frame):
    global current_filename, last_chunk_time, image_chunks, available_files, window_mode, window_counter
    
    result = KISSProtocol.unwrap_frame(frame)
    if not result: return
    
    cmd, payload = result
    
    # Debug Print
    if cmd != 0x00:
        print(f"[KISS CMD {hex(cmd)}] Payload: {payload.hex().upper()}")
    
    # Image Chunk Handling (CMD=0x00, SubCMD=0x13)
    if cmd == 0x00 and len(payload) > 17 and payload[0] == 0x13:
        try:
            # Check Protocol Mode
            if window_mode:
                # Format: [13] [Name 12] [ID 4] [Data...] [CRC 4]
                fname = payload[1:13].decode('utf-8', errors='ignore').strip('\x00')
                chunk_id = struct.unpack('<I', payload[13:17])[0]
                data_start = 17
            else:
                # Legacy Format: [13] [ID 4] [Name 12] [Data...] [CRC 4]
                chunk_id = struct.unpack('<I', payload[1:5])[0]
                fname = payload[5:17].decode('utf-8', errors='ignore').strip('\x00')
                data_start = 17

            data_len = len(payload) - data_start - 4
            data = payload[data_start:data_start+data_len]
            
            if current_filename != fname:
                if image_chunks: 
                     save_image() # Save previous file final state
                     image_chunks.clear() # Clear for new file
                     window_counter = 0
                current_filename = fname
                if fname: print(f"\n[START] Receving File: {fname}")

            if chunk_id not in image_chunks:
                image_chunks[chunk_id] = data
                
            last_chunk_time = time.time()
            
            # Count for ACK
            if window_mode:
                window_counter += 1
                if window_counter >= 10:
                    send_ack(ser)
                    window_counter = 0
            
            if chunk_id % 10 == 0:
                sys.stdout.write(f"\rRx Chunk: {chunk_id}, Bytes: {len(data)}   ")
                sys.stdout.flush()

        except Exception as e:
            print(f"\nError parsing chunk: {e}")

    # List Files Response (CMD=0x00, SubCMD=0x11)
    elif cmd == 0x00 and len(payload) > 5 and payload[0] == 0x11:
        print("\n--- File List ---")
        available_files = [] # Clear previous list
        try:
            # Payload: [0x11] [File1:16] ... [FileN:16] [CRC:4]
            data = payload[1:-4]
            num_files = len(data) // 16
            
            print(f"{'#':<4} {'Filename':<15} {'Size (Bytes)':<12}")
            print("-" * 35)
            
            for i in range(num_files):
                entry = data[i*16 : (i+1)*16]
                if len(entry) < 16: break
                
                fname = entry[:12].decode('utf-8', errors='ignore').strip('\x00')
                fsize = struct.unpack('<I', entry[12:16])[0]
                
                available_files.append(fname)
                print(f"{i+1:<4} {fname:<15} {fsize:<12}")
                
            print("-" * 35 + "\n")
            
        except Exception as e:
            print(f"Error parsing file list: {e}")
    
    # ... (Other logic) ...

    elif cmd == 0x00:
        # Other Data
        try:
            ascii_str = payload.decode('ascii', errors='ignore')
            if len(ascii_str) > 0 and ascii_str[0] not in ['\x11', '\x13']:
                 print(f"[STM32 MSG]: {ascii_str}")
        except:
            print(f"[STM32 DATA]: {payload.hex().upper()}")

def serial_listener(ser):
    global rx_buffer, last_chunk_time, running, last_saved_chunk_time
    
    while running:
        try:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                rx_buffer.extend(chunk)
                
                # Process Frames
                while KISSProtocol.FEND in rx_buffer:
                    try:
                        fend_idx = rx_buffer.index(KISSProtocol.FEND)
                    except ValueError:
                        break
                        
                    if fend_idx == 0:
                        try:
                             next_fend = rx_buffer.index(KISSProtocol.FEND, 1)
                             frame = rx_buffer[0:next_fend+1]
                             process_frame(ser, frame)
                             del rx_buffer[0:next_fend] 
                        except ValueError:
                            break
                    else:
                        del rx_buffer[0:fend_idx]
            
            # Check Timeout (Auto-save)
            # Only save if:
            # 1. We have data (image_chunks)
            # 2. Enough time has passed since last data (2.0s)
            # 3. We haven't already saved this state (last_chunk_time != last_saved_chunk_time)
            if image_chunks and (time.time() - last_chunk_time > 2.0) and (last_chunk_time != last_saved_chunk_time):
                save_image()
                last_saved_chunk_time = last_chunk_time
                print("\n[READY] Transfer complete. Enter next command:")
            
            time.sleep(0.005)
        except Exception as e:
            print(f"Serial Error: {e}")
            break

def main():
    global running, available_files
    try:
        port = SERIAL_PORT
        ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
        print(f"Opened {port} at {BAUD_RATE}")
        
        # Start Serial Thread
        t = threading.Thread(target=serial_listener, args=(ser,), daemon=True)
        t.start()
        
        while True:
            print("\nCommands:")
            print("1. Send C0 12 xx xx xx xx C0 (Forward to USB)")
            print("2. Send 0x00 0x13 (Start File Transfer - Legacy)")
            print("3. Send 0x00 0x10 (List Files)")
            print("4. Send 0x00 0x12 (Start Windowed Transfer)")
            print("q. Quit")
            
            choice = input("Enter choice: ").strip()
            
            if choice == '1':
                send_command_12(ser)
            elif choice == '2':
                send_command_start_file(ser)
            elif choice == '3':
                # List Files Logic (Same as before)
                available_files = [] # Clear
                send_command_list_files(ser)
                print("Waiting for file list...")
                
                # Wait for up to 3 seconds for response
                msg_received = False
                for _ in range(30):
                    if available_files:
                        msg_received = True
                        break
                    time.sleep(0.1)
                
                if msg_received:
                    time.sleep(0.5) 
                    while True:
                        sel = input(f"Enter File # to download (1-{len(available_files)}, q to cancel): ").strip()
                        if sel.lower() == 'q':
                            break
                        try:
                            idx = int(sel)
                            if 1 <= idx <= len(available_files):
                                target = available_files[idx-1]
                                print(f"Requesting '{target}' using Windowed Transfer (0x12)...")
                                send_command_windowed_start(ser, target)
                                break
                            else:
                                print("Invalid number.")
                        except ValueError:
                            print("Invalid input.")
                else:
                    print("No response or empty list.")

            elif choice == '4':
                 send_command_windowed_start(ser)
                 
            elif choice.lower() == 'q':
                running = False
                break
                
    except Exception as e:
        print(f"Error: {e}")
        print(f"Ensure '{SERIAL_PORT}' is correct and not in use.")

if __name__ == "__main__":
    main()
