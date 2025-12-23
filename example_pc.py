#!/usr/bin/env python3
"""
Example: PC with USB CAN adapter - receiver with ACK
Confirms data reception back to sender

No error handling needed - library handles everything!
"""

from canbus import CANBus
import time

def main():
    # Initialize USB CAN adapter (library handles all errors internally)
    print("Starting PC CAN interface with ACK...")
    can = CANBus('COM6', bitrate=250000)  # Change COM port as needed
    
    # Simple infinite loop - no try/except needed!
    while True:
        # Bulk receive for efficiency
        messages = can.receive_all(timeout=0.1, max_msgs=100)
        
        for can_id, data in messages:
            print(f"[RX] ID=0x{can_id:X}, Data={[f'{b:#04x}({b})' for b in data]}")
            
            # Send ACK for telemetry messages (ID 0x100)
            if can_id == 0x100:
                # Just call send_ack - library handles all errors
                can.send_ack(can_id, data)
                print(f"     ↳ ACK sent")
        
        time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")

