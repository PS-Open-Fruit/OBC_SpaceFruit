# Quick Start Guide - Reliable CAN Bus Library

## Installation

```bash
pip install python-can pyserial
```

## For Raspberry Pi Zero 2W (Satellite)

### 1. Copy the library to your Pi
```bash
scp canbus.py pi@raspberrypi:~/
scp example_pi.py pi@raspberrypi:~/
```

### 2. Run the example
```bash
ssh pi@raspberrypi
python3 example_pi.py
```

### 3. Production code template (Fire-and-Forget)
```python
from canbus import CANBus
import time

# Initialize CAN (1000 message queue for long outages)
can = CANBus('can0', bitrate=250000, queue_size=1000)

try:
    counter = 0
    while True:
        # YOUR CODE: Read sensors
        temp = read_temperature()
        voltage = read_voltage()
        
        # Send telemetry (automatically queued if link down)
        telemetry = [
            0x01,                    # Subsystem ID
            int(temp) & 0xFF,        # Temperature
            int(voltage * 10) & 0xFF,# Voltage (0.1V resolution)
            counter & 0xFF           # Message counter
        ]
        can.send(0x100, telemetry)
        
        # Listen for commands (non-blocking, from RX buffer)
        cmd = can.receive(timeout=0.1)
        if cmd:
            cmd_id, data = cmd
            # YOUR CODE: Handle command
            handle_command(cmd_id, data)
        
        # Health check every 5 seconds (fast recovery)
        if counter % 5 == 0:
            can.check_link()
            stats = can.get_stats()
            
            # Log warnings
            if stats['queue_length'] > 100:
                print(f"WARNING: {stats['queue_length']} msgs queued!")
            if stats['total_dropped'] > 0:
                print(f"ERROR: {stats['total_dropped']} msgs dropped!")
        
        counter += 1
        time.sleep(1.0)
        
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    can.close()
```

### 4. Production code template (With ACK Confirmation)
```python
from canbus import CANBus
import time

# Initialize CAN
can = CANBus('can0', bitrate=250000, queue_size=1000)

try:
    counter = 0
    consecutive_ack_failures = 0
    
    while True:
        # Read sensors
        temp = read_temperature()
        voltage = read_voltage()
        
        telemetry = [
            0x01,
            int(temp) & 0xFF,
            int(voltage * 10) & 0xFF,
            counter & 0xFF
        ]
        
        # Send with ACK confirmation (0.5s timeout, 1 retry)
        success = can.send_with_ack(0x100, telemetry, timeout=0.5, max_retries=1)
        
        if success:
            consecutive_ack_failures = 0
        else:
            consecutive_ack_failures += 1
            print(f"ACK failed! Consecutive failures: {consecutive_ack_failures}")
            
            # After 3 ACK failures, switch to queue mode
            if consecutive_ack_failures >= 3:
                print("Switching to queue mode (link appears down)")
                can._link_ok = False
                can._flush_hardware_buffer()
                consecutive_ack_failures = 0
        
        # Check link every 5 seconds (drain queue if needed)
        if counter % 5 == 0:
            can.check_link()
        
        counter += 1
        time.sleep(1.0)
        
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    can.close()
```

## For PC (Ground Station)

### 1. Find your COM port
Windows: Device Manager → Ports (COM & LPT)  
Linux: `ls /dev/ttyUSB*`

### 2. Run the receiver
```bash
python example_pc.py
# Edit COM port in script first!
```

### 3. Production code template (With Automatic ACK)
```python
from canbus import CANBus
import time

# Connect to USB CAN adapter
can = CANBus('COM6', bitrate=250000)  # Change COM6 to your port

try:
    while True:
        # Bulk receive for efficiency (from background RX thread buffer)
        messages = can.receive_all(timeout=0.1, max_msgs=100)
        
        for can_id, data in messages:
            # YOUR CODE: Process telemetry
            if can_id == 0x100:
                subsys_id = data[0]
                temp = data[1]
                voltage = data[2] / 10.0
                counter = data[3]
                
                print(f"Telemetry: Temp={temp}°C, V={voltage}V, Count={counter}")
                
                # Send ACK (fire-and-forget, non-blocking)
                try:
                    can.send_ack(can_id, data)
                except Exception:
                    pass  # Don't let ACK failures stop processing
                
                # YOUR CODE: Store to database, display, etc.
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    can.close()
```

## Troubleshooting

### Pi: Interface won't start
```bash
# Manual setup
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000 restart-ms 100
sudo ip link set can0 txqueuelen 10
sudo ip link set can0 up

# Verify
ip -details link show can0
```

### Pi: Check if CAN HAT is detected
```bash
dmesg | grep -i can
# Should see MCP2515 messages
```

### PC: Wrong COM port
```python
# List all available ports
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"{port.device}: {port.description}")
```

### Both: No messages received
1. **Check bitrate** - Must match on both ends (250000 recommended)
2. **Check wiring** - CAN_H to CAN_H, CAN_L to CAN_L
3. **Check termination** - 120Ω resistor at each end of bus
4. **Test loopback** - Send and receive on same device first

### Both: Messages being dropped
```python
stats = can.get_stats()
if stats['total_dropped'] > 0:
    print(f"Increase queue_size! (currently {stats['queue_max']})")
    # Recreate with larger queue
    can.close()
    can = CANBus('can0', bitrate=250000, queue_size=5000)
```

## Testing the Connection

### Test 1: Loopback (same device)
```python
from canbus import CANBus

can = CANBus('can0', bitrate=250000)  # or 'COM6' for PC

# Send
can.send(0x123, [0xAA, 0xBB, 0xCC])

# Receive (won't work on same bus without another device, but tests send)
result = can.receive(timeout=1.0)

can.close()
```

### Test 2: Pi → PC communication
```bash
# On Pi
python3 example_pi.py

# On PC
python example_pc.py

# You should see messages on PC from Pi
```

### Test 3: Link failure simulation
```bash
# Run interactive test
python test_reliable.py interactive

# Watch the queue fill when you disconnect cable
# Watch it drain when you reconnect
```

## Key Concepts

### Link States
- **UP**: CAN messages sending successfully
- **DOWN**: 3+ consecutive send failures

### Queue Behavior
- **Link UP**: Messages sent immediately, queue drains
- **Link DOWN**: Messages queued in RAM (doesn't touch hardware buffer)
- **Link RECOVERS**: Queue automatically drains (~100 msgs/sec)

### When to call `check_link()`
```python
# Call periodically (every 5-10 seconds)
if counter % 10 == 0:
    can.check_link()  # Attempts to drain queue
```

### When to check stats
```python
# Check for warnings
stats = can.get_stats()

# Queue building up?
if stats['queue_length'] > 500:
    print("WARNING: Link may be down")

# Messages dropped?
if stats['total_dropped'] > 0:
    print(f"ERROR: {stats['total_dropped']} msgs lost (increase queue_size)")
```

## Best Practices

✅ Use `queue_size=1000` or higher for satellite applications  
✅ Call `check_link()` every 5-10 seconds  
✅ Monitor `total_dropped` - if > 0, increase queue size  
✅ Use `receive_all()` on PC for better throughput  
✅ Use bitrate=250000 (good balance of speed and reliability)  
✅ Always use `try/finally` to ensure `close()` is called  

❌ Don't set `queue_size=0` (unlimited) - may run out of RAM  
❌ Don't call `check_link()` too often (< 1 second) - wastes CPU  
❌ Don't ignore `total_dropped` counter - means data loss!  
❌ Don't use bitrate > 500000 on long cables (unreliable)  

## Next Steps

1. ✅ Get basic send/receive working
2. ✅ Test link failure handling (disconnect cable)
3. ✅ Monitor statistics in production
4. ✅ Tune `queue_size` based on link outage duration
5. ✅ Add your application logic

**You're ready to deploy! 🚀**
