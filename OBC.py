import serial
import struct
import time
import random

# Import your custom KISS protocol class
from Shared.Python.kiss_protocol import KISSProtocol

# --- CONFIGURATION ---
PORT = 'COM5'  # Change to your virtual or real COM port
BAUD = 9600

# --- TELEMETRY IDs (PIDs) ---
PIDs = {
    0x01: "Solar Cell Voltage",
    0x02: "Solar Cell Current",
    0x03: "Battery Voltage",
    0x04: "Battery Current",
    0x05: "Battery Temperature"
}

def build_custom_payload(payload_id: int, pid: int, seq_num: int, data: bytes) -> bytes:
    """Builds the internal payload: [PayloadID][PID][SeqNum][DataLen][Data][CRC32]"""
    data_len = len(data)
    # Pack Header: > (Big-Endian), B (1-byte), H (2-byte)
    header = struct.pack('>BBBH', payload_id, pid, seq_num, data_len)
    content = header + data
    
    # Calculate and append CRC32
    crc = KISSProtocol.calculate_crc(content)
    return content + struct.pack('>I', crc)

def parse_custom_payload(payload_bytes: bytes):
    """Validates CRC and extracts the custom payload fields."""
    if len(payload_bytes) < 9:  # Min size: 5 (header) + 0 (data) + 4 (crc)
        return None
        
    content = payload_bytes[:-4]
    received_crc = struct.unpack('>I', payload_bytes[-4:])[0]
    calculated_crc = KISSProtocol.calculate_crc(content)
    
    if received_crc != calculated_crc:
        print("CRC Error!")
        return None
        
    payload_id, pid, seq_num, data_len = struct.unpack('>BBBH', content[:5])
    data = content[5:5+data_len]
    return payload_id, pid, seq_num, data

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"OBC Started on {PORT}. Waiting for Ground Station requests...")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    seq_counter = 0
    
    while True:
        byte = ser.read(1)
        if byte:
            if byte == FEND_BYTE:
                rx_buffer += byte
                # If buffer has a start FEND, command byte, and end FEND, attempt unwrap
                if len(rx_buffer) >= 3:
                    unwrapped = KISSProtocol.unwrap_frame(bytes(rx_buffer))
                    if unwrapped:
                        cmd, payload_bytes = unwrapped
                        parsed = parse_custom_payload(payload_bytes)
                        
                        if parsed:
                            p_id, pid, seq, data = parsed
                            
                            # Handle GS Request (Command 0x01)
                            if cmd == 0x01:
                                print("\n[OBC] Received Telemetry Request. Sending Data Burst...")
                                
                                for sensor_pid, name in PIDs.items():
                                    seq_counter = (seq_counter + 1) % 256
                                    dummy_val = random.uniform(5.0, 10.0)
                                    data_bytes = struct.pack('>f', dummy_val)
                                    
                                    # Build internal payload, then wrap in KISS frame
                                    custom_payload = build_custom_payload(0x00, sensor_pid, seq_counter, data_bytes)
                                    tx_frame = KISSProtocol.wrap_frame(custom_payload, command=0x00)
                                    
                                    ser.write(tx_frame)
                                    print(f"  -> Sent {name} (PID: 0x{sensor_pid:02X}, Seq: {seq_counter})")
                                    time.sleep(0.05)
                                    
                            # Handle GS ACK (Command 0xAC)
                            elif cmd == 0xAC:
                                print(f"[OBC] Received ACK for SeqNum up to: {seq}")
                                
                # Reset buffer and start with FEND for the next potential frame
                rx_buffer = bytearray(FEND_BYTE)
            else:
                # Only append if we've initialized the buffer with a FEND
                if len(rx_buffer) > 0:
                    rx_buffer += byte

if __name__ == '__main__':
    main()