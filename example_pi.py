#!/usr/bin/env python3
"""
Example: VR Subsystem on Raspberry Pi Zero 2W
Sends telemetry and receives commands from OBC
"""

from canbus import CANBus
import time

def main():
    # Initialize CAN bus on Pi
    print("Starting VR Subsystem CAN interface...")
    can = CANBus('can0', bitrate=250000)
    
    try:
        counter = 0
        
        while True:
            # Send telemetry data to OBC
            telemetry = [
                0x01,  # VR subsystem ID
                counter & 0xFF,  # Counter low byte
                (counter >> 8) & 0xFF,  # Counter high byte
                0xAA,  # Status byte (example)
            ]
            
            if can.send(0x100, telemetry):
                print(f"[TX] Telemetry sent: {telemetry}")
            
            # Listen for commands from OBC
            result = can.receive(timeout=0.1)
            if result:
                cmd_id, data = result
                print(f"[RX] Command from OBC: ID=0x{cmd_id:X}, Data={data}")
                
                # Echo back acknowledgment
                ack = [0xFF, data[0] if data else 0x00]
                can.send(0x101, ack)
            
            counter += 1
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        can.close()

if __name__ == '__main__':
    main()
