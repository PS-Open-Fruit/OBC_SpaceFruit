#!/usr/bin/env python3
"""
USB-CDC Reliability Test - Pi Side
Runs on Raspberry Pi with USB gadget mode
Receives packets and echoes them back for reliability testing
"""

import serial
import sys
import time
import struct

class USBCDCEcho:
    def __init__(self, port='/dev/ttyGS0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.stats = {
            'packets_received': 0,
            'packets_echoed': 0,
            'bytes_received': 0,
            'bytes_sent': 0,
            'errors': 0,
            'crc_errors': 0
        }
    
    def connect(self):
        """Open serial connection"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.5)  # Wait for port to stabilize
            print(f"[INFO] Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect: {e}")
            return False
    
    def calculate_checksum(self, data):
        """Calculate XOR checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF
    
    def echo_loop(self):
        """Main echo loop - receives and echoes back"""
        print("[INFO] Starting echo loop... (Ctrl+C to stop)")
        print("[INFO] Waiting for packets on /dev/ttyGS0")
        
        try:
            while True:
                # Wait for START byte
                start_time = time.time()
                while True:
                    byte = self.ser.read(1)
                    if not byte:
                        # Timeout waiting for data
                        if time.time() - start_time > 10:
                            print("[WARN] No data received for 10 seconds")
                            start_time = time.time()
                        continue
                    if byte[0] == 0xAA:  # START byte
                        break
                
                # Read header (packet number + length)
                header = self.ser.read(4)
                if len(header) < 4:
                    print("[ERROR] Incomplete header")
                    self.stats['errors'] += 1
                    continue
                
                packet_num = int.from_bytes(header[0:2], 'big')
                data_len = int.from_bytes(header[2:4], 'big')
                
                # Safety check for data length
                if data_len > 2048:
                    print(f"[ERROR] Invalid data length: {data_len}")
                    self.stats['errors'] += 1
                    continue
                
                # Read data and checksum
                data = self.ser.read(data_len)
                checksum_byte = self.ser.read(1)
                
                if len(data) < data_len or len(checksum_byte) < 1:
                    print(f"[ERROR] Incomplete packet (got {len(data)}/{data_len} bytes)")
                    self.stats['errors'] += 1
                    continue
                
                # Verify checksum
                expected_checksum = self.calculate_checksum(header + data)
                if checksum_byte[0] != expected_checksum:
                    print(f"[ERROR] Checksum mismatch on packet {packet_num}")
                    self.stats['crc_errors'] += 1
                    continue
                
                self.stats['packets_received'] += 1
                self.stats['bytes_received'] += len(header) + len(data) + 1
                
                # Echo the packet back
                try:
                    response = bytearray()
                    response.append(0xAA)  # START
                    response.extend(header)
                    response.extend(data)
                    response.append(checksum_byte[0])
                    
                    self.ser.write(response)
                    self.stats['packets_echoed'] += 1
                    self.stats['bytes_sent'] += len(response)
                    
                    if self.stats['packets_received'] % 10 == 0:
                        print(f"[OK] Echoed packet #{packet_num} ({len(data)} bytes)")
                
                except Exception as e:
                    print(f"[ERROR] Failed to echo packet {packet_num}: {e}")
                    self.stats['errors'] += 1
        
        except KeyboardInterrupt:
            print("\n[INFO] Echo loop stopped by user")
        except Exception as e:
            print(f"[ERROR] Unexpected error: {e}")
        finally:
            self.print_stats()
    
    def print_stats(self):
        """Print statistics"""
        print("\n" + "="*60)
        print("STATISTICS")
        print("="*60)
        print(f"Packets received:  {self.stats['packets_received']}")
        print(f"Packets echoed:    {self.stats['packets_echoed']}")
        print(f"Bytes received:    {self.stats['bytes_received']}")
        print(f"Bytes sent:        {self.stats['bytes_sent']}")
        print(f"CRC errors:        {self.stats['crc_errors']}")
        print(f"Other errors:      {self.stats['errors']}")
        print("="*60 + "\n")
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Disconnected")

def main():
    print("""
    ╔════════════════════════════════════════════════════════╗
    ║     USB-CDC Echo Server - Raspberry Pi Side            ║
    ║        Echoing on /dev/ttyGS0 for testing              ║
    ╚════════════════════════════════════════════════════════╝
    """)
    
    # Try to open the serial port
    echo = USBCDCEcho('/dev/ttyGS0', baudrate=115200)
    
    if not echo.connect():
        print("[ERROR] Failed to connect to /dev/ttyGS0")
        print("[INFO] Make sure USB gadget mode is enabled on Pi")
        sys.exit(1)
    
    try:
        echo.echo_loop()
    finally:
        echo.disconnect()

if __name__ == '__main__':
    main()
