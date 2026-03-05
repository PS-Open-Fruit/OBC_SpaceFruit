import serial
import struct
import time

# Import your custom KISS protocol class
from Shared.Python.kiss_protocol import KISSProtocol

# --- CONFIGURATION ---
PORT = 'COM4'  # Change to your virtual or real COM port
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
    data_len = len(data)
    header = struct.pack('>BBBH', payload_id, pid, seq_num, data_len)
    content = header + data
    crc = KISSProtocol.calculate_crc(content)
    return content + struct.pack('>I', crc)

def parse_custom_payload(payload_bytes: bytes):
    if len(payload_bytes) < 9: 
        return None
    content = payload_bytes[:-4]
    received_crc = struct.unpack('>I', payload_bytes[-4:])[0]
    calculated_crc = KISSProtocol.calculate_crc(content)
    if received_crc != calculated_crc:
        return None
    payload_id, pid, seq_num, data_len = struct.unpack('>BBBH', content[:5])
    data = content[5:5+data_len]
    return payload_id, pid, seq_num, data

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Ground Station Started on {PORT}.")
    
    FEND_BYTE = bytes([KISSProtocol.FEND])
    rx_buffer = bytearray()
    last_request_time = 0
    highest_seq_received = -1
    packets_received_in_window = 0
    
    while True:
        # 1. Master Polling: Ask for data every 5 seconds
        if time.time() - last_request_time > 5.0:
            print("\n[GS] Requesting Telemetry from OBC...")
            custom_payload = build_custom_payload(0x00, 0xFF, 0x00, b'') 
            req_frame = KISSProtocol.wrap_frame(custom_payload, command=0x01)
            ser.write(req_frame)
            
            last_request_time = time.time()
            highest_seq_received = -1
            packets_received_in_window = 0

        # 2. Process Incoming Bytes
        byte = ser.read(1)
        if byte:
            if byte == FEND_BYTE:
                rx_buffer += byte
                if len(rx_buffer) >= 3:
                    unwrapped = KISSProtocol.unwrap_frame(bytes(rx_buffer))
                    if unwrapped:
                        cmd, payload_bytes = unwrapped
                        parsed = parse_custom_payload(payload_bytes)
                        
                        if parsed:
                            p_id, pid, seq, data = parsed
                            
                            # Handle Data Frame (Command 0x00)
                            if cmd == 0x00:
                                value = struct.unpack('>f', data)[0]
                                sensor_name = PIDs.get(pid, f"Unknown PID 0x{pid:02X}")
                                print(f"  <- Received: {sensor_name} = {value:.2f} (Seq: {seq})")
                                
                                highest_seq_received = max(highest_seq_received, seq)
                                packets_received_in_window += 1
                                
                                # Send ACK when window is complete
                                if packets_received_in_window == 5:
                                    print(f"[GS] Window complete. Sending ACK for SeqNum {highest_seq_received}")
                                    ack_payload = build_custom_payload(0x00, 0x00, highest_seq_received, b'')
                                    ack_frame = KISSProtocol.wrap_frame(ack_payload, command=0xAC)
                                    ser.write(ack_frame)
                                    
                rx_buffer = bytearray(FEND_BYTE)
            else:
                if len(rx_buffer) > 0:
                    rx_buffer += byte

if __name__ == '__main__':
    main()