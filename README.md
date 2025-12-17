# VR CAN Bus Library

Simple and reliable CAN communication library for Raspberry Pi Zero 2W and PC.

## Features

- ✅ Works on both Pi (SocketCAN/can0) and PC (USB CAN adapter/COM port)
- ✅ Automatic retry for reliable transmission
- ✅ Send/receive raw bytes or strings
- ✅ Multi-frame support for large messages
- ✅ Simple API

## Installation

```bash
pip install python-can
```

## Quick Start

### On Raspberry Pi (can0)

```python
from canbus import CANBus

# Create CAN bus instance (auto-configures can0)
can = CANBus('can0', bitrate=250000)

# Send message
can.send(0x123, [0x01, 0x02, 0x03, 0x04])

# Receive message
result = can.receive(timeout=1.0)
if result:
    can_id, data = result
    print(f"Received ID: 0x{can_id:X}, Data: {data}")

# Close
can.close()
```

### On PC (USB CAN adapter)

```python
from canbus import CANBus

# Create CAN bus instance for COM port
can = CANBus('COM6', bitrate=250000)

# Send message
can.send(0x123, [0x01, 0x02, 0x03, 0x04])

# Receive message
result = can.receive(timeout=1.0)
if result:
    can_id, data = result
    print(f"Received ID: 0x{can_id:X}, Data: {data}")

# Close
can.close()
```

### Using Context Manager

```python
from canbus import CANBus

with CANBus('can0') as can:
    # Send
    can.send(0x123, [0x01, 0x02, 0x03])
    
    # Receive
    result = can.receive(timeout=2.0)
    if result:
        can_id, data = result
        print(f"Got: {data}")
# Automatically closed
```

### Send/Receive Strings

```python
from canbus import CANBus

with CANBus('can0') as can:
    # Send string (automatically split into frames)
    can.send_string(0x200, "Hello from VR subsystem!")
    
    # Receive string (automatically reassemble frames)
    text = can.receive_string(timeout=5.0)
    if text:
        print(f"Received: {text}")
```

## API Reference

### `CANBus(interface, bitrate=100000, auto_setup=True)`

Create CAN bus instance.

- `interface`: 'can0' for Pi, 'COM6' (or other) for PC
- `bitrate`: CAN bitrate in bps (default: 100000)
- `auto_setup`: Auto-configure interface on Pi (default: True)

### `send(can_id, data, extended=False, max_retries=3)`

Send CAN message with retry.

- `can_id`: CAN ID (0x000-0x7FF standard, up to 0x1FFFFFFF extended)
- `data`: List of 0-8 bytes
- `extended`: Use extended ID format
- `max_retries`: Retry attempts
- Returns: `True` if successful

### `receive(timeout=1.0)`

Receive CAN message.

- `timeout`: Timeout in seconds (None for blocking)
- Returns: `(can_id, data)` tuple or `None` if timeout

### `send_string(can_id, text, extended=False)`

Send string as multi-frame message.

- `can_id`: CAN ID
- `text`: String to send
- Returns: `True` if successful

### `receive_string(timeout=5.0, max_frames=100)`

Receive multi-frame string message.

- `timeout`: Total timeout
- `max_frames`: Max frames to collect
- Returns: Reconstructed string or `None`

### `close()`

Close CAN connection and shutdown interface.

## Examples

### Pi Example (VR Subsystem)

```python
from canbus import CANBus
import time

# Initialize CAN on Pi
can = CANBus('can0', bitrate=100000)

try:
    # Send telemetry to OBC
    while True:
        can.send(0x100, [0x01, 0x02, 0x03, 0x04, 0x05])
        
        # Listen for commands from OBC
        result = can.receive(timeout=0.1)
        if result:
            cmd_id, data = result
            print(f"Command from OBC: ID=0x{cmd_id:X}, Data={data}")
        
        time.sleep(0.5)
finally:
    can.close()
```

### PC Example (Testing/Development)

```python
from canbus import CANBus

# Connect to USB CAN adapter
can = CANBus('COM6', bitrate=100000)

try:
    # Send command to Pi
    can.send(0x200, [0xAA, 0xBB, 0xCC])
    print("Command sent")
    
    # Wait for response
    result = can.receive(timeout=2.0)
    if result:
        can_id, data = result
        print(f"Response: ID=0x{can_id:X}, Data={[hex(b) for b in data]}")
    else:
        print("No response")
finally:
    can.close()
```

## Testing Connection

### On Pi:
```bash
# Terminal 1: Monitor CAN traffic
candump can0

# Terminal 2: Run Python script
python3 your_script.py
```

### On PC:
Just run your Python script with the COM port configured.

## Notes

- Default bitrate is 100kbps (100000)
- Maximum 8 bytes per CAN frame
- Automatic retry on send failure (default: 3 attempts)
- String messages automatically split/reassemble across multiple frames
- First byte of multi-frame messages is sequence number

## Troubleshooting

**Pi: "Network is down"**
- Check if CAN HAT is properly connected
- Try manual setup: `sudo ip link set can0 type can bitrate 100000 && sudo ip link set can0 up`

**PC: "Cannot find COM port"**
- Verify COM port in Device Manager
- Check USB CAN adapter drivers installed
- Use correct COM port number

**No messages received**
- Check both devices use same bitrate
- Verify CAN_H and CAN_L connections
- Check termination resistors (120Ω at each end)

## License

See LICENSE file in project root.
