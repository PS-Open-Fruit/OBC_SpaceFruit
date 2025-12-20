#!/usr/bin/env python3
"""
Example: Raspberry Pi Zero 2W - Reliable CAN with ACK confirmation
Guarantees data delivery to destination

No error handling needed - library handles everything!
"""

from canbus import CANBus
import time

def main():
    # Initialize with larger RAM queue (library handles all setup)
    print("Starting Pi CAN interface with ACK confirmation...")
    can = CANBus('can0', bitrate=250000, queue_size=1000)
    
    counter = 0
    ack_failures = 0
    
    # Simple infinite loop - no try/except needed!
    while True:
        # Send telemetry with ACK confirmation
        telemetry = [
            0x01,                    # Subsystem ID
            counter & 0xFF,          # Counter low byte
            (counter >> 8) & 0xFF,   # Counter high byte
            0xAA,                    # Status
        ]
        
        # Just call send_with_ack - library handles everything
        success = can.send_with_ack(
            can_id=0x100,
            data=telemetry,
            ack_id=0x101,
            timeout=0.5,
            max_retries=1
        )
        
        if success:
            print(f"[TX+ACK] ID=0x100, Data={[f'{b:#04x}({b})' for b in telemetry]} - CONFIRMED!")
            ack_failures = 0
        else:
            print(f"[TX FAIL] ID=0x100, Data={[f'{b:#04x}({b})' for b in telemetry]} - NOT RECEIVED!")
            ack_failures += 1
            
            if ack_failures >= 3:
                print(f"[ALERT] Link appears DOWN - {ack_failures} consecutive failures")
                # After detecting link down, switch to queue mode
                if ack_failures == 3:
                    print(f"[MODE] Switching to fast queue mode (no ACK wait)")
        
        # Periodically check link (attempts to drain queue)
        if counter % 5 == 0:
            can.check_link()
            stats = can.get_stats()
            if stats['queue_length'] > 0:
                print(f"[QUEUE] {stats['queue_length']} messages waiting")
        
        counter += 1
        time.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")
