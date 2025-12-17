#!/usr/bin/env python3
"""
Example: PC testing with USB CAN adapter
Simulates OBC communication for testing
"""

from canbus import CANBus
import time

def main():
    # Initialize USB CAN adapter on PC
    print("Starting PC CAN interface...")
    can = CANBus('COM6', bitrate=250000)  # Change COM6 to your port
    
    msg_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Fast receive - get all buffered messages at once
            messages = can.receive_all(timeout=0.1, max_messages=100)
            
            if messages:
                for can_id, data in messages:
                    msg_count += 1
                    print(f"[RX] ID=0x{can_id:X}, Data={[hex(b) for b in data]}")
                
                # Show throughput stats every second
                elapsed = time.time() - start_time
                if elapsed >= 1.0:
                    print(f"[STATS] {msg_count} msgs in {elapsed:.2f}s = {msg_count/elapsed:.1f} msg/s")
                    msg_count = 0
                    start_time = time.time()
            else:
                print("Waiting for messages...")
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        can.close()

if __name__ == '__main__':
    main()
