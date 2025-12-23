#!/usr/bin/env python3
"""
Example: PC with USB CAN adapter - simple receiver

Just call can.receive_all() in a loop - that's it!
Library handles all buffering and threading automatically.
"""

from canbus import CANBus
import time

def main():
    print("Starting PC CAN receiver...")
    can = CANBus('COM6', bitrate=250000)  # Change COM port as needed
    
    # Main loop - super simple!
    while True:
        # Bulk receive all available messages
        messages = can.receive_all(max_msgs=100)
        
        for can_id, data in messages:
            print(f"RX: ID=0x{can_id:03X} Data={[f'{b:02X}' for b in data]}")
            
            # Optional: Send ACK for specific messages
            if can_id == 0x100:
                can.send_ack(can_id, data)
                print(f"     ↳ ACK sent")
        
        time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")

