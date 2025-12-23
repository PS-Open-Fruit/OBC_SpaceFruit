#!/usr/bin/env python3
"""
Example: Raspberry Pi Zero 2W - Simple CAN sender with ACK confirmation

Just call:
  can.send_with_ack(id, data)    # Send and wait for ACK
  can.send(id, data)             # Send and forget (queues if link down)
  can.receive_all()              # Get buffered messages

Library automatically handles:
- Queuing if link is down
- Draining queue when link recovers
- Detecting link failures
- Background buffering for RX
- NO buffer overflow!

No error handling needed - library handles everything!
"""

from canbus import CANBus
import time

def main():
    print("Starting Pi CAN sender with ACK confirmation...")
    can = CANBus('can0', bitrate=250000, queue_size=1000)
    
    counter = 0
    ack_failures = 0
    
    # Main loop - just send and receive, that's it!
    while True:
        # Create telemetry data
        telemetry = [
            0x01,                    # Subsystem ID
            counter & 0xFF,          # Counter low byte
            (counter >> 8) & 0xFF,   # Counter high byte
            0xAA,                    # Status
        ]
        
        # Option 1: Send and wait for ACK (guarantees delivery)
        success = can.send_with_ack(
            can_id=0x100,
            data=telemetry,
            ack_id=0x101,
            timeout=0.5,
            max_retries=1
        )
        
        if success:
            print(f"TX+ACK: ID=0x100 Data={[f'{b:02X}' for b in telemetry]} - CONFIRMED")
            ack_failures = 0
        else:
            print(f"TX FAIL: ID=0x100 Data={[f'{b:02X}' for b in telemetry]} - NO ACK")
            ack_failures += 1
            if ack_failures >= 3:
                print(f"[ALERT] Link appears DOWN - {ack_failures} consecutive failures")
        
        # Periodically show queue status
        if counter % 5 == 0:
            can.check_link()
            stats = can.get_stats()
            if stats['queue_length'] > 0:
                print(f"[QUEUE] {stats['queue_length']} messages waiting to send")
        
        counter += 1
        time.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")
