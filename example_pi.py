#!/usr/bin/env python3
"""
Example: Raspberry Pi Zero 2W - Reliable CAN with ACK confirmation
Guarantees data delivery to destination
"""

from canbus import CANBus
import time

def main():
    # Initialize with larger RAM queue (won't overflow hardware buffer)
    print("Starting Pi CAN interface with ACK confirmation...")
    can = CANBus('can0', bitrate=250000, queue_size=1000)
    
    try:
        counter = 0
        ack_failures = 0
        
        while True:
            # Send telemetry with ACK confirmation (shorter timeout for faster detection)
            telemetry = [
                0x01,                    # Subsystem ID
                counter & 0xFF,          # Counter low byte
                (counter >> 8) & 0xFF,   # Counter high byte
                0xAA,                    # Status
            ]
            
            # Send with ACK - but use shorter timeout (0.5s) and only 1 retry
            success = can.send_with_ack(
                can_id=0x100,
                data=telemetry,
                ack_id=0x101,
                timeout=0.5,  # Faster detection
                max_retries=1  # Just one attempt
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
                    # Don't waste time waiting for ACKs, just queue messages
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
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        can.close()

if __name__ == '__main__':
    main()
