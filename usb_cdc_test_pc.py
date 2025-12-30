"""
USB-CDC Reliability Test - PC Side (Windows)
Tests the connection between PC (COM13) and Pi (/dev/ttyGS0)
Monitors for data loss and connection reliability
"""

import serial
import time
import sys
from datetime import datetime
from collections import deque

class USBCDCTest:
    def __init__(self, port, baudrate=115200, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.stats = {
            'packets_sent': 0,
            'packets_received': 0,
            'bytes_sent': 0,
            'bytes_received': 0,
            'errors': 0,
            'crc_errors': 0,
            'timeout_errors': 0,
            'data_loss': 0
        }
        self.rx_buffer = deque(maxlen=1000)
        
    def connect(self):
        """Establish serial connection"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")
    
    def calculate_checksum(self, data):
        """Calculate simple XOR checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF
    
    def send_packet(self, packet_num, data):
        """Send a test packet with header, data, and checksum"""
        try:
            # Packet format: [START(1)] [PKT_NUM(2)] [LENGTH(2)] [DATA] [CHECKSUM(1)]
            packet = bytearray()
            packet.append(0xAA)  # START byte
            packet.extend(packet_num.to_bytes(2, 'big'))
            
            data_bytes = data if isinstance(data, bytes) else data.encode()
            packet.extend(len(data_bytes).to_bytes(2, 'big'))
            packet.extend(data_bytes)
            
            checksum = self.calculate_checksum(packet[1:])
            packet.append(checksum)
            
            self.ser.write(packet)
            self.stats['packets_sent'] += 1
            self.stats['bytes_sent'] += len(packet)
            
            return packet
        except Exception as e:
            print(f"✗ Send error: {e}")
            self.stats['errors'] += 1
            return None
    
    def receive_packet(self, timeout=None):
        """Receive and verify a test packet"""
        try:
            if timeout:
                old_timeout = self.ser.timeout
                self.ser.timeout = timeout
            
            # Wait for START byte
            while True:
                byte = self.ser.read(1)
                if not byte:
                    self.stats['timeout_errors'] += 1
                    return None
                if byte[0] == 0xAA:
                    break
            
            # Read header
            header = self.ser.read(4)
            if len(header) < 4:
                self.stats['timeout_errors'] += 1
                return None
            
            packet_num = int.from_bytes(header[0:2], 'big')
            data_len = int.from_bytes(header[2:4], 'big')
            
            # Read data and checksum
            data = self.ser.read(data_len)
            checksum_byte = self.ser.read(1)
            
            if len(data) < data_len or len(checksum_byte) < 1:
                self.stats['timeout_errors'] += 1
                self.stats['data_loss'] += 1
                return None
            
            # Verify checksum
            expected_checksum = self.calculate_checksum(
                header + data
            )
            
            if checksum_byte[0] != expected_checksum:
                self.stats['crc_errors'] += 1
                print(f"  ✗ Checksum mismatch (packet {packet_num})")
                return None
            
            self.stats['packets_received'] += 1
            self.stats['bytes_received'] += len(header) + len(data) + 1
            
            if timeout:
                self.ser.timeout = old_timeout
            
            return {
                'num': packet_num,
                'data': data,
                'checksum': checksum_byte[0]
            }
        except Exception as e:
            print(f"✗ Receive error: {e}")
            self.stats['errors'] += 1
            return None
    
    def test_echo(self, num_packets=100, packet_size=64):
        """Test: Send packets and receive echoes"""
        print(f"\n{'='*60}")
        print(f"ECHO TEST: {num_packets} packets × {packet_size} bytes")
        print(f"{'='*60}")
        
        start_time = time.time()
        
        for i in range(num_packets):
            # Create test pattern (counting sequence)
            data = bytes([j % 256 for j in range(packet_size)])
            
            self.send_packet(i, data)
            
            # Try to receive echo
            rx = self.receive_packet(timeout=2)
            
            if rx:
                if rx['data'] == data:
                    if (i + 1) % 10 == 0:
                        print(f"✓ Packet {i+1}/{num_packets} OK")
                else:
                    print(f"✗ Packet {i+1}: Data mismatch!")
                    self.stats['data_loss'] += 1
            else:
                print(f"✗ Packet {i+1}: No echo received")
                self.stats['data_loss'] += 1
            
            time.sleep(0.01)  # Small delay between packets
        
        elapsed = time.time() - start_time
        self.print_stats(elapsed)
    
    def test_throughput(self, duration=10, block_size=256):
        """Test: Maximum throughput"""
        print(f"\n{'='*60}")
        print(f"THROUGHPUT TEST: {duration} seconds")
        print(f"{'='*60}")
        
        start_time = time.time()
        pkt_num = 0
        
        while time.time() - start_time < duration:
            data = bytes([pkt_num % 256] * block_size)
            self.send_packet(pkt_num, data)
            pkt_num += 1
            
            # Try to receive some responses
            while True:
                rx = self.receive_packet(timeout=0.1)
                if not rx:
                    break
        
        elapsed = time.time() - start_time
        throughput = (self.stats['bytes_sent'] / elapsed) / 1024
        print(f"\nThroughput: {throughput:.2f} KB/s")
        self.print_stats(elapsed)
    
    def test_stress(self, iterations=10):
        """Test: Stress test with varying packet sizes"""
        print(f"\n{'='*60}")
        print(f"STRESS TEST: {iterations} iterations")
        print(f"{'='*60}")
        
        start_time = time.time()
        
        # Test with various packet sizes
        sizes = [1, 16, 64, 128, 256, 512, 1024]
        
        for iteration in range(iterations):
            for size in sizes:
                data = bytes([iteration % 256] * size)
                self.send_packet(iteration * 100 + size, data)
                
                rx = self.receive_packet(timeout=1)
                if not rx or rx['data'] != data:
                    self.stats['data_loss'] += 1
                    print(f"✗ Iteration {iteration+1}, size {size}: Failed")
                else:
                    if iteration == 0:
                        print(f"✓ Size {size} bytes: OK")
        
        elapsed = time.time() - start_time
        self.print_stats(elapsed)
    
    def print_stats(self, elapsed):
        """Print test statistics"""
        print(f"\n{'='*60}")
        print(f"TEST STATISTICS (elapsed: {elapsed:.2f}s)")
        print(f"{'='*60}")
        print(f"Packets sent:       {self.stats['packets_sent']}")
        print(f"Packets received:   {self.stats['packets_received']}")
        print(f"Bytes sent:         {self.stats['bytes_sent']}")
        print(f"Bytes received:     {self.stats['bytes_received']}")
        print(f"Data loss count:    {self.stats['data_loss']}")
        print(f"Checksum errors:    {self.stats['crc_errors']}")
        print(f"Timeout errors:     {self.stats['timeout_errors']}")
        print(f"Other errors:       {self.stats['errors']}")
        
        if self.stats['packets_sent'] > 0:
            loss_rate = (self.stats['data_loss'] / self.stats['packets_sent']) * 100
            print(f"Data loss rate:     {loss_rate:.2f}%")
        
        if elapsed > 0:
            print(f"Throughput:         {(self.stats['bytes_sent']/elapsed)/1024:.2f} KB/s")
        
        print(f"{'='*60}\n")

def main():
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║     USB-CDC Reliability Test - PC Side (Windows)         ║
    ║          Testing COM13 ↔ Pi /dev/ttyGS0                  ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    # Create tester
    tester = USBCDCTest('COM13', baudrate=115200)
    
    if not tester.connect():
        sys.exit(1)
    
    try:
        # Run tests
        tester.test_echo(num_packets=100, packet_size=64)
        time.sleep(1)
        
        tester.test_echo(num_packets=50, packet_size=256)
        time.sleep(1)
        
        tester.test_stress(iterations=5)
        
    except KeyboardInterrupt:
        print("\n\n✓ Test interrupted by user")
    finally:
        tester.disconnect()

if __name__ == '__main__':
    main()
