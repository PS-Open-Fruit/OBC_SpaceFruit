
import serial
import time
import os
import sys

# ==========================================
# Configuration
# ==========================================
PORT = "COM5"      # Adjust your COM port
BAUDRATE = 209700  # Adjust your baudrate
OUTPUT_DIR = "received_images"
FILENAME = "output.jpg" 

# ==========================================
# Main Receiver Logic
# ==========================================
def receive_image_packets(output_filename):
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    filepath = os.path.join(OUTPUT_DIR, output_filename)
    
    received_chunks = {} 
    
    print(f"Connecting to {PORT} at {BAUDRATE}...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("Listening for packets...")
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    buffer = bytearray()
    
    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                for b in chunk:
                    if b == 0xC0: # FEND
                        if len(buffer) > 0:
                            # Process Frame
                            process_packet(buffer, received_chunks)
                            buffer = bytearray()
                    elif b == 0xDB: # ESC
                        # Read next byte for escaped value
                        # Note: In a real stream, next byte might not be here yet.
                        # This simple loop assumes it is or we wait.
                        # Ideally, state machine.
                        # For now, let's just peek/wait a tiny bit.
                        nb_data = ser.read(1)
                        if nb_data:
                            nb = nb_data[0]
                            if nb == 0xDC: buffer.append(0xC0)
                            elif nb == 0xDD: buffer.append(0xDB)
                            else: buffer.append(nb) 
                    else:
                        buffer.append(b)
            
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open: ser.close()
        
        # Save File
        if received_chunks:
            print(f"Saving {len(received_chunks)} chunks to {filepath}")
            # Sort by ID
            sorted_ids = sorted(received_chunks.keys())
            
            with open(filepath, "wb") as f:
                for cid in sorted_ids:
                    f.write(received_chunks[cid])
            print(f"Saved: {filepath}")
        else:
            print("No chunks received.")

def process_packet(data, received_chunks):
    # Packet Format: [CMD(1)] [ID(4)] [NAME(12)] [DATA(64)] [CRC(4)]
    # Total = 85 bytes
    
    if len(data) < 85: return
    
    cmd = data[0]
    chunk_id = int.from_bytes(data[1:5], 'little')
    img_data = data[17:17+64] # Fixed 64 bytes
    
    if cmd == 0x13:
        if chunk_id not in received_chunks:
            received_chunks[chunk_id] = img_data
            print(f"RX Chunk #{chunk_id}")



if __name__ == "__main__":
    receive_image_packets(FILENAME)
