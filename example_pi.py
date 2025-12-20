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
            # Send telemetry with ACK confirmation
            telemetry = [
                0x01,                    # Subsystem ID
                counter & 0xFF,          # Counter low byte
                (counter >> 8) & 0xFF,   # Counter high byte
                0xAA,                    # Status
            ]
            
            # Send with ACK - guarantees PC received it!
            success = can.send_with_ack(
                can_id=0x100,
                data=telemetry,
                ack_id=0x101,
                timeout=2.0,
                max_retries=3
            )
            
            if success:
                print(f"[TX+ACK] ID=0x100, Data={[f'{b:#04x}({b})' for b in telemetry]} - CONFIRMED!")
                ack_failures = 0
            else:
                print(f"[TX FAIL] ID=0x100, Data={[f'{b:#04x}({b})' for b in telemetry]} - NOT RECEIVED!")
                ack_failures += 1
                
                if ack_failures >= 3:
                    print(f"[ALERT] Link appears DOWN - {ack_failures} consecutive failures")
            
            # Listen for commands (non-blocking)
            result = can.receive(timeout=0.1)
            if result:
                cmd_id, data = result
                print(f"[RX] Command: ID=0x{cmd_id:X}, Data={[f'{b:#04x}({b})' for b in data]}")
                # Send ACK to confirm we received the command
                can.send_ack(cmd_id, data)
            
            # Periodically check link (attempts to drain queue)
            if counter % 20 == 0:
                can.check_link()
            
            counter += 1
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        can.close()

if __name__ == '__main__':
    main()
