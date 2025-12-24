#!/usr/bin/env python3
"""
CAN Protocol File Transfer Test
Sends a file over CAN with checksum verification

Features:
- File fragmentation into 8-byte CAN messages
- CRC32 checksum for data integrity
- File transfer protocol with header
- Sender and receiver simulation
"""

import struct
import time
import zlib
from typing import List, Tuple, Optional
from canbus import CANBus


# Protocol Constants
CAN_MSG_SIZE = 8  # CAN messages are max 8 bytes

# Message Types (first byte of CAN message)
MSG_TYPE_READY = 0x00      # Ready check: [TYPE]
MSG_TYPE_READY_ACK = 0x06  # Ready acknowledgement: [TYPE]
MSG_TYPE_START = 0x01      # File transfer start: [TYPE, TOTAL_SIZE_4bytes, FILENAME_LEN]
MSG_TYPE_FILENAME = 0x81   # Filename continuation: [TYPE, FILENAME_BYTES...]
MSG_TYPE_DATA = 0x02       # Data chunk: [TYPE, SEQ_NUM_4bytes, DATA...] (3 bytes max data)
MSG_TYPE_END = 0x03        # Transfer end: [TYPE, CRC32_4bytes]
MSG_TYPE_END_ACK = 0x07    # Transfer complete ACK: [TYPE]
MSG_TYPE_ACK = 0x04        # Acknowledgement: [TYPE, SEQ_NUM_2bytes or 0xFFFF for END]
MSG_TYPE_ERROR = 0x05      # Error: [TYPE, ERROR_CODE]


class FileTransferSender:
    """Sends a file over CAN with checksums"""
    
    def __init__(self, can: CANBus, can_id: int = 0x100):
        self.can = can
        self.can_id = can_id
        self.ack_id = can_id + 1
    
    def send_file(self, filename: str) -> bool:
        """
        Send a file over CAN
        
        Args:
            filename: Path to file to send
            
        Returns:
            True if successful, False on error
        """
        try:
            # Read file
            with open(filename, 'rb') as f:
                file_data = f.read()
            
            file_size = len(file_data)
            print(f"\n[SENDER] Sending file: {filename} ({file_size} bytes)")
            
            # Calculate CRC32 checksum
            crc32 = zlib.crc32(file_data) & 0xffffffff
            print(f"[SENDER] CRC32: 0x{crc32:08X}")
            
            # Step 0: Check if receiver is ready
            print(f"[SENDER] Checking if receiver is ready...")
            if not self._wait_for_receiver_ready(timeout=5.0):
                print("[SENDER] Receiver not ready - aborting")
                return False
            print(f"[SENDER] Receiver is ready!")
            
            # Step 1: Send START message with file info
            if not self._send_start(filename, file_size):
                print("[SENDER] START failed")
                return False
            
            # Step 2: Send file data in chunks
            sequence = 0
            bytes_sent = 0
            start_time = time.time()
            
            while bytes_sent < file_size:
                chunk_size = min(3, file_size - bytes_sent)  # 3 bytes data + 5 bytes header (type + seq)
                chunk = file_data[bytes_sent:bytes_sent + chunk_size]
                
                if not self._send_data_chunk(sequence, chunk):
                    print(f"[SENDER] DATA chunk {sequence} failed")
                    return False
                
                bytes_sent += chunk_size
                sequence += 1
                
                # Progress with speed metrics (every 500 chunks)
                if sequence % 500 == 0 or bytes_sent == file_size:
                    elapsed = time.time() - start_time
                    speed = bytes_sent / elapsed if elapsed > 0 else 0  # bytes per second
                    progress = (bytes_sent / file_size) * 100
                    remaining_bytes = file_size - bytes_sent
                    eta = remaining_bytes / speed if speed > 0 else 0
                    
                    print(f"[SENDER] Sent {bytes_sent}/{file_size} bytes ({progress:.1f}%) | Speed: {speed:.1f} B/s | ETA: {eta:.1f}s")
                
                # Small pause every 50 messages to let receiver drain buffer
                if sequence % 50 == 0:
                    time.sleep(0.01)  # 10ms pause
            
            # Step 3: Send END message with CRC32
            if not self._send_end(crc32):
                print("[SENDER] END failed")
                return False
            
            # Step 4: Wait for final ACK from receiver
            print(f"[SENDER] Waiting for receiver to acknowledge...")
            if self._wait_for_end_ack(timeout=10.0):
                print(f"[SENDER] ✓ File transfer complete - ACK received!")
                return True
            else:
                print(f"[SENDER] ✗ No ACK received from receiver")
                return False
            
        except FileNotFoundError:
            print(f"[SENDER] File not found: {filename}")
            return False
        except Exception as e:
            print(f"[SENDER] Error: {e}")
            return False
    
    def _send_start(self, filename: str, file_size: int) -> bool:
        """Send START message (header only) followed by filename"""
        try:
            filename_bytes = filename.encode('utf-8')[:200]  # Truncate if too long
            
            # Send START header: [TYPE, SIZE_4bytes, FILENAME_LEN]
            data = [MSG_TYPE_START]
            data.extend(struct.pack('>I', file_size))  # Big-endian 4-byte size
            data.append(len(filename_bytes))
            
            self.can.send(self.can_id, data)  # 6 bytes total (1+4+1), fits in one CAN message
            
            # Send filename in continuation frames if needed
            for i in range(0, len(filename_bytes), 7):  # 7 bytes per frame (1 byte header + 7 data)
                chunk = filename_bytes[i:i + 7]
                frame = [MSG_TYPE_FILENAME]
                frame.extend(chunk)
                self.can.send(self.can_id, frame)
            
            print(f"[SENDER] START sent - filename: {filename}, size: {file_size}")
            return True
        except Exception as e:
            print(f"[SENDER] START error: {e}")
            return False
    
    def _send_data_chunk(self, sequence: int, chunk: bytes) -> bool:
        """Send DATA chunk with sequence number"""
        try:
            data = [MSG_TYPE_DATA]
            data.extend(struct.pack('>I', sequence))  # Big-endian 4-byte sequence
            data.extend(chunk)
            
            self.can.send(self.can_id, data[:CAN_MSG_SIZE])
            time.sleep(0.001)  # 1ms minimum delay to prevent receiver buffer overflow
            
            return True
        except Exception as e:
            print(f"[SENDER] DATA error: {e}")
            return False
    
    def _send_end(self, crc32: int) -> bool:
        """Send END message with CRC32"""
        try:
            data = [MSG_TYPE_END]
            data.extend(struct.pack('>I', crc32))  # Big-endian 4-byte CRC32
            
            self.can.send(self.can_id, data)
            print(f"[SENDER] END sent - CRC32: 0x{crc32:08X}")
            return True
        except Exception as e:
            print(f"[SENDER] END error: {e}")
            return False
    
    def _wait_for_receiver_ready(self, timeout: float = 5.0) -> bool:
        """Wait for receiver to confirm it's ready"""
        try:
            # Send READY message
            self.can.send(self.can_id, [MSG_TYPE_READY])
            
            # Wait for READY_ACK
            start_time = time.time()
            while time.time() - start_time < timeout:
                result = self.can.receive(timeout=0.1)
                if result:
                    can_id, data = result
                    if can_id == self.ack_id and len(data) > 0 and data[0] == MSG_TYPE_READY_ACK:
                        return True
            
            return False
        except Exception as e:
            print(f"[SENDER] READY check error: {e}")
            return False
    
    def _wait_for_end_ack(self, timeout: float = 10.0) -> bool:
        """Wait for receiver to confirm transfer complete"""
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                result = self.can.receive(timeout=0.1)
                if result:
                    can_id, data = result
                    if can_id == self.ack_id and len(data) > 0 and data[0] == MSG_TYPE_END_ACK:
                        return True
            
            return False
        except Exception as e:
            print(f"[SENDER] END ACK error: {e}")
            return False


class FileTransferReceiver:
    """Receives a file over CAN and verifies checksum"""
    
    def __init__(self, can: CANBus, can_id: int = 0x100):
        self.can = can
        self.can_id = can_id
        self.ack_id = can_id + 1
        
        self.reset()
    
    def reset(self):
        """Reset receiver state"""
        self.state = 'IDLE'  # IDLE, START, FILENAME, DATA, DONE
        self.filename = None
        self.filename_buffer = bytearray()
        self.expected_filename_len = 0
        self.file_size = 0
        self.file_data = bytearray()
        self.expected_crc32 = 0
        self.sequence = 0
    
    def receive_file(self, timeout: float = 60.0) -> Tuple[bool, Optional[str]]:
        """
        Receive file with timeout
        
        Returns:
            (success, error_message)
        """
        start_time = time.time()
        last_message_time = time.time()
        self.reset()
        
        print(f"\n[RECEIVER] Waiting for file transfer...")
        
        while time.time() - start_time < timeout:
            messages = self.can.receive_all()
            
            for can_id, data in messages:
                if can_id != self.can_id:
                    continue
                
                if not data:
                    continue
                
                last_message_time = time.time()  # Reset timeout on each message
                msg_type = data[0]
                
                # Parse message based on type
                if msg_type == MSG_TYPE_READY:
                    if not self._handle_ready():
                        return False, "READY response failed"
                
                elif msg_type == MSG_TYPE_START:
                    if not self._handle_start(data):
                        return False, "START parse error"
                
                elif msg_type == MSG_TYPE_FILENAME:
                    # Handle FILENAME regardless of state (in case START was missed)
                    if not self._handle_filename(data):
                        return False, "FILENAME parse error"
                
                elif msg_type == MSG_TYPE_DATA:
                    # Only process DATA if we're in DATA state
                    if self.state == 'DATA':
                        if not self._handle_data(data, start_time):
                            return False, "DATA parse error"
                
                elif msg_type == MSG_TYPE_END:
                    if not self._handle_end(data):
                        return False, "END parse error"
                    # Send ACK back to sender
                    if not self._send_end_ack():
                        print("[RECEIVER] Warning: Failed to send final ACK")
                    return True, None
            
            # Check if no messages for 10 seconds (connection lost)
            if time.time() - last_message_time > 10.0:
                return False, f"No data received for 10 seconds (got {len(self.file_data)}/{self.file_size} bytes)"
            
            time.sleep(0.001)  # Minimal sleep - consume messages as fast as possible!
        
        return False, f"Timeout waiting for file (got {len(self.file_data)}/{self.file_size} bytes)"
    
    def _handle_start(self, data: List[int]) -> bool:
        """Handle START message (header only)"""
        try:
            if len(data) < 6:
                print("[RECEIVER] START message too short")
                return False
            
            file_size = struct.unpack('>I', bytes(data[1:5]))[0]
            filename_len = data[5]
            
            self.file_size = file_size
            self.expected_filename_len = filename_len
            self.filename_buffer = bytearray()
            self.state = 'FILENAME'  # Wait for filename frames
            self.sequence = 0
            
            print(f"[RECEIVER] START received - expecting {filename_len} bytes filename, {file_size} bytes data")
            return True
        except Exception as e:
            print(f"[RECEIVER] START error: {e}")
            return False
    
    def _handle_ready(self) -> bool:
        """Handle READY message from sender"""
        try:
            print("[RECEIVER] READY message received")
            # Send back READY_ACK on ack_id so sender can receive it
            self.can.send(self.ack_id, [MSG_TYPE_READY_ACK])
            return True
        except Exception as e:
            print(f"[RECEIVER] READY handler error: {e}")
            return False
    
    def _handle_filename(self, data: List[int]) -> bool:
        """Handle FILENAME continuation message"""
        try:
            # Initialize if not already done (in case START was missed)
            if self.state == 'IDLE':
                self.state = 'FILENAME'
                self.filename_buffer = bytearray()
            
            filename_chunk = data[1:]  # Skip type byte
            self.filename_buffer.extend(filename_chunk)
            
            # Check if we have complete filename
            if self.expected_filename_len > 0 and len(self.filename_buffer) >= self.expected_filename_len:
                self.filename = self.filename_buffer[:self.expected_filename_len].decode('utf-8')
                self.state = 'DATA'
                print(f"[RECEIVER] Filename received: {self.filename}")
            
            return True
        except Exception as e:
            print(f"[RECEIVER] FILENAME error: {e}")
            return False
    
    def _handle_data(self, data: List[int], start_time: float = None) -> bool:
        """Handle DATA message"""
        try:
            if len(data) < 5:
                print("[RECEIVER] DATA message too short")
                return False
            
            sequence = struct.unpack('>I', bytes(data[1:5]))[0]
            chunk = data[5:]
            
            # Check sequence (warning only, don't fail)
            if sequence != self.sequence:
                print(f"[RECEIVER] Warning: Expected sequence {self.sequence}, got {sequence}")
            
            self.file_data.extend(chunk)
            self.sequence = sequence + 1
            
            progress = (len(self.file_data) / self.file_size) * 100 if self.file_size > 0 else 0
            # Print progress every 500 chunks to reduce spam
            if sequence % 500 == 0 or progress >= 99.9:
                elapsed = time.time() - start_time if start_time else 0
                speed = len(self.file_data) / elapsed if elapsed > 0 else 0
                remaining_bytes = self.file_size - len(self.file_data)
                eta = remaining_bytes / speed if speed > 0 else 0
                
                print(f"[RECEIVER] DATA {sequence} - received {len(self.file_data)}/{self.file_size} bytes ({progress:.1f}%) | Speed: {speed:.1f} B/s | ETA: {eta:.1f}s")
            
            return True
        except Exception as e:
            print(f"[RECEIVER] DATA error: {e}")
            return False
    
    def _handle_end(self, data: List[int]) -> bool:
        """Handle END message and verify CRC32"""
        try:
            if len(data) < 5:
                print("[RECEIVER] END message too short")
                return False
            
            expected_crc32 = struct.unpack('>I', bytes(data[1:5]))[0]
            actual_crc32 = zlib.crc32(bytes(self.file_data)) & 0xffffffff
            
            print(f"[RECEIVER] END received")
            print(f"[RECEIVER] Expected CRC32: 0x{expected_crc32:08X}")
            print(f"[RECEIVER] Actual CRC32:   0x{actual_crc32:08X}")
            
            if expected_crc32 == actual_crc32:
                print(f"[RECEIVER] ✓ CRC32 MATCH - File transfer successful!")
                self.state = 'DONE'
                return True
            else:
                print(f"[RECEIVER] ✗ CRC32 MISMATCH - Data corruption detected!")
                return False
                
        except Exception as e:
            print(f"[RECEIVER] END error: {e}")
            return False
    
    def save_file(self, output_filename: str) -> bool:
        """Save received file to disk"""
        try:
            with open(output_filename, 'wb') as f:
                f.write(self.file_data)
            print(f"[RECEIVER] File saved: {output_filename}")
            return True
        except Exception as e:
            print(f"[RECEIVER] Save error: {e}")
            return False
    
    def _send_end_ack(self) -> bool:
        """Send final ACK after successful transfer"""
        try:
            print("[RECEIVER] Sending final ACK...")
            self.can.send(self.ack_id, [MSG_TYPE_END_ACK])
            time.sleep(0.1)  # Give sender time to receive it
            return True
        except Exception as e:
            print(f"[RECEIVER] END_ACK error: {e}")
            return False




def sender_mode(filename: str, interface: str = 'COM6', bitrate: int = 250000):
    """Run as SENDER - send file over CAN"""
    try:
        if not os.path.exists(filename):
            print(f"✗ File not found: {filename}")
            return False
        
        print("=" * 60)
        print("CAN FILE TRANSFER - SENDER MODE")
        print("=" * 60)
        print(f"File:      {filename}")
        print(f"Interface: {interface}")
        print(f"Bitrate:   {bitrate} bps")
        print("=" * 60)
        
        can = CANBus(interface, bitrate=bitrate)
        sender = FileTransferSender(can, can_id=0x100)
        
        success = sender.send_file(filename)
        
        print("\n[SENDER] Waiting for receiver to acknowledge...")
        time.sleep(3)
        
        can.close()
        return success
        
    except Exception as e:
        print(f"✗ Sender error: {e}")
        import traceback
        traceback.print_exc()
        return False


def receiver_mode(interface: str = 'COM6', bitrate: int = 250000):
    """Run as RECEIVER - receive file over CAN"""
    try:
        print("=" * 60)
        print("CAN FILE TRANSFER - RECEIVER MODE")
        print("=" * 60)
        print(f"Interface: {interface}")
        print(f"Bitrate:   {bitrate} bps")
        print("=" * 60)
        print("\n[RECEIVER] Waiting for file transfer...")
        print("[RECEIVER] (Make sure sender is running!)")
        
        can = CANBus(interface, bitrate=bitrate)
        receiver = FileTransferReceiver(can, can_id=0x100)
        
        success, error = receiver.receive_file(timeout=120.0)  # 2 minutes timeout
        
        if success:
            output_filename = f"received_{receiver.filename}"
            receiver.save_file(output_filename)
            
            with open(output_filename, 'rb') as f:
                received_size = len(f.read())
            
            print(f"\n✓ FILE TRANSFER SUCCESSFUL!")
            print(f"  Filename:  {receiver.filename}")
            print(f"  Size:      {received_size} bytes")
            print(f"  Saved to:  {output_filename}")
        else:
            print(f"\n✗ Receiver failed: {error}")
        
        can.close()
        return success
        
    except Exception as e:
        print(f"✗ Receiver error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    import sys
    import os
    
    if len(sys.argv) < 2:
        # Interactive mode
        print("=" * 60)
        print("CAN FILE TRANSFER TEST")
        print("=" * 60)
        print("\nUsage:")
        print("  SENDER:   python test_file_transfer.py send <filename> [interface] [bitrate]")
        print("  RECEIVER: python test_file_transfer.py recv [interface] [bitrate]")
        print("\nExamples:")
        print("  python test_file_transfer.py send myfile.bin COM6 250000")
        print("  python test_file_transfer.py recv COM6 250000")
        print("\nDefault: interface=COM6, bitrate=250000")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == 'send':
        if len(sys.argv) < 3:
            print("✗ Usage: python test_file_transfer.py send <filename> [interface] [bitrate]")
            sys.exit(1)
        
        filename = sys.argv[2]
        interface = sys.argv[3] if len(sys.argv) > 3 else 'COM6'
        bitrate = int(sys.argv[4]) if len(sys.argv) > 4 else 250000
        
        sender_mode(filename, interface, bitrate)
    
    elif mode == 'recv' or mode == 'receive':
        interface = sys.argv[2] if len(sys.argv) > 2 else 'COM6'
        bitrate = int(sys.argv[3]) if len(sys.argv) > 3 else 250000
        
        receiver_mode(interface, bitrate)
    
    else:
        print(f"✗ Unknown mode: {mode}")
        print("Use 'send' or 'recv'")
        sys.exit(1)
