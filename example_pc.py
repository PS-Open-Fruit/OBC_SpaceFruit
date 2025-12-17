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
    
    try:
        while True:
            # Listen for telemetry from Pi
            result = can.receive(timeout=2.0)
            if result:
                can_id, data = result
                print(f"[RX] Telemetry from VR: ID=0x{can_id:X}, Data={[hex(b) for b in data]}")
                
                # Send command to Pi
                command = [0x01, 0x42]  # Example command
                if can.send(0x200, command):
                    print(f"[TX] Command sent to VR: {[hex(b) for b in command]}")
                
                # Wait for acknowledgment
                ack = can.receive(timeout=1.0)
                if ack:
                    ack_id, ack_data = ack
                    print(f"[RX] ACK from VR: {ack_data}")
            else:
                print("Waiting for VR subsystem...")
            
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        can.close()

if __name__ == '__main__':
    main()
