import serial
import time
import hashlib
import argparse
import os
import sys

# Default configuration
DEFAULT_BAUDRATE = 115200 # Note: USB-CDC usually ignores baudrate, but good to have
CHUNK_SIZE = 4096 # 4KB chunks

def calculate_md5(data):
    return hashlib.md5(data).hexdigest()

def run_sender(port, baudrate, data_size_mb, iterations):
    print(f"--- Serial Throughput Test: SENDER ---")
    print(f"Port: {port}, Baud: {baudrate}, Size: {data_size_mb} MB, Iterations: {iterations}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Opened serial port {port}")
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    for i in range(iterations):
        print(f"\n--- Iteration {i+1}/{iterations} ---")
        
        # Generate random data
        data_size = data_size_mb * 1024 * 1024
        print(f"Generating {data_size_mb} MB of random data...")
        data = os.urandom(data_size)
        data_hash = calculate_md5(data)
        print(f"Data generated. MD5: {data_hash}")

        # Handshake
        print("Waiting for receiver...")
        # Clear input buffer
        ser.reset_input_buffer()
        
        # Send header: "TEST:<SIZE>:<HASH>\n"
        header = f"TEST:{data_size}:{data_hash}\n"
        ser.write(header.encode('utf-8'))
        print(f"Sent header: {header.strip()}")

        # Wait for ACK
        try:
            ack = ser.readline().decode('utf-8').strip()
            if ack != "ACK":
                print(f"Error: Did not receive ACK. Received: '{ack}'")
                # Try to recover or abort? Abort for now.
                return
            print("Received ACK. Starting transmission...")
        except Exception as e:
            print(f"Handshake failed: {e}")
            return

        # Send data
        start_time = time.time()
        bytes_sent = 0
        
        try:
            while bytes_sent < data_size:
                chunk = data[bytes_sent:bytes_sent + CHUNK_SIZE]
                ser.write(chunk)
                bytes_sent += len(chunk)
                # Optional: Print progress
                if bytes_sent % (1024 * 1024) == 0:
                    sys.stdout.write(f"\rSent: {bytes_sent / (1024*1024):.1f} / {data_size_mb} MB")
                    sys.stdout.flush()
            
            # Flush to ensure all data is written out
            ser.flush()
            end_time = time.time()
            print(f"\nTransmission complete.")
            
            # Wait for result from receiver
            print("Waiting for verification...")
            result = ser.readline().decode('utf-8').strip()
            print(f"Receiver reported: {result}")
            
            if result.startswith("SUCCESS"):
                print("TEST PASSED: Data verified successfully.")
                # Parse throughput from receiver if available, or calc local
                duration = end_time - start_time
                throughput = (data_size * 8) / (duration * 1000000) # Mbps
                print(f"Sender measured time: {duration:.4f}s")
                print(f"Sender calculated speed: {throughput:.2f} Mbps (approx)")
            else:
                print("TEST FAILED: Data verification failed on receiver.")

        except Exception as e:
            print(f"Transmission error: {e}")
            break
            
    ser.close()

def run_receiver(port, baudrate):
    print(f"--- Serial Throughput Test: RECEIVER ---")
    print(f"Port: {port}, Baud: {baudrate}")
    print("Waiting for sender...")

    try:
        ser = serial.Serial(port, baudrate, timeout=10) # Long timeout for waiting
        print(f"Opened serial port {port}")
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    try:
        while True:
            # Wait for header
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            if line.startswith("TEST:"):
                parts = line.split(":")
                if len(parts) == 3:
                    expected_size = int(parts[1])
                    expected_hash = parts[2]
                    print(f"Test request received. Size: {expected_size} bytes, Hash: {expected_hash}")
                    
                    # Clear any garbage before starting
                    ser.reset_input_buffer()

                    # Send ACK
                    ser.write(b"ACK\n")
                    print("Sent ACK. Receiving data...")
                    
                    # Read data
                    received_data = bytearray()
                    start_time = time.time()
                    
                    while len(received_data) < expected_size:
                        # Read as much as available
                        chunk = ser.read(min(CHUNK_SIZE, expected_size - len(received_data)))
                        if not chunk:
                            print("Timeout waiting for data.")
                            break
                        received_data.extend(chunk)
                        
                        if len(received_data) % (1024 * 1024) == 0:
                            sys.stdout.write(f"\rReceived: {len(received_data) / (1024*1024):.1f} MB")
                            sys.stdout.flush()
                    
                    end_time = time.time()
                    print("\nReception complete.")
                    
                    # Verify
                    print("Verifying data...")
                    received_hash = calculate_md5(received_data)
                    
                    duration = end_time - start_time
                    if duration == 0: duration = 0.001 # Avoid div by zero
                    throughput_mbps = (len(received_data) * 8) / (duration * 1000000)
                    throughput_kbs = len(received_data) / (duration * 1024)
                    
                    print(f"Time: {duration:.4f}s")
                    print(f"Speed: {throughput_mbps:.2f} Mbps ({throughput_kbs:.2f} KB/s)")
                    print(f"Expected Hash: {expected_hash}")
                    print(f"Received Hash: {received_hash}")
                    
                    if received_hash == expected_hash:
                        print("SUCCESS: Checksum matches.")
                        ser.write(f"SUCCESS:{throughput_mbps:.2f}\n".encode('utf-8'))
                    else:
                        print("FAILURE: Checksum mismatch.")
                        ser.write(f"FAILURE:{received_hash}\n".encode('utf-8'))
                    
                    # Reset for next test
                    print("Ready for next test.\n")
                    received_data = bytearray()
                else:
                    print(f"Invalid header format: {line}")

    except KeyboardInterrupt:
        print("\nStopping receiver.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial Port Throughput and Reliability Tester")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM13 or /dev/ttyGS0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help=f"Baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--role", choices=["sender", "receiver"], required=True, help="Role: sender or receiver")
    parser.add_argument("--size", type=int, default=1, help="Data size in MB (for sender, default: 1)")
    parser.add_argument("--iterations", type=int, default=1, help="Number of iterations (for sender, default: 1)")

    args = parser.parse_args()

    if args.role == "sender":
        run_sender(args.port, args.baud, args.size, args.iterations)
    else:
        run_receiver(args.port, args.baud)
