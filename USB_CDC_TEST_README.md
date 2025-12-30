# USB-CDC Reliability Testing Guide

## Overview
This testing setup allows you to reliably test the USB-CDC (Virtual COM port) connection between:
- **PC (Windows)**: COM13
- **Raspberry Pi**: /dev/ttyGS0 (USB gadget mode)

The tests will detect data loss, checksum errors, and measure throughput.

---

## Prerequisites

### Windows PC
```bash
# Install pyserial if not already installed
pip install pyserial
```

### Raspberry Pi
```bash
# Install pyserial on Pi
sudo pip install pyserial

# Ensure USB gadget mode is enabled
# Check if /dev/ttyGS0 exists
ls -la /dev/ttyGS0
```

---

## Raspberry Pi Setup

### Enable USB Gadget Serial (if not already enabled)

1. **Edit `/boot/cmdline.txt`** (or `/boot/firmware/cmdline.txt` on newer RPi OS):
   ```bash
   sudo nano /boot/cmdline.txt
   ```
   
   Look for the end of the line and add USB gadget serial:
   ```
   dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=PARTUUID=... rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait modules-load=dwc2,g_serial
   ```

2. **Edit `/etc/modules`**:
   ```bash
   sudo nano /etc/modules
   ```
   
   Add these lines:
   ```
   dwc2
   g_serial
   ```

3. **Reboot**:
   ```bash
   sudo reboot
   ```

4. **Verify USB serial gadget is active**:
   ```bash
   ls -la /dev/ttyGS0
   dmesg | grep "usb_gadget\|g_serial"
   ```

---

## Running the Tests

### Step 1: Start Pi Echo Server
On the Raspberry Pi, run:
```bash
python3 usb_cdc_test_pi.py
```

You should see:
```
[INFO] Connected to /dev/ttyGS0 at 115200 baud
[INFO] Starting echo loop... (Ctrl+C to stop)
[INFO] Waiting for packets on /dev/ttyGS0
```

### Step 2: Run PC Tests
On Windows PC, run:
```bash
python usb_cdc_test_pc.py
```

The test will:
1. **Echo Test**: Send 100 packets of 64 bytes, verify echoes
2. **Echo Test**: Send 50 packets of 256 bytes, verify echoes
3. **Stress Test**: Test with various packet sizes (1 to 1024 bytes)

### Example Output

**PC Output:**
```
============================================================
ECHO TEST: 100 packets × 64 bytes
============================================================
✓ Packet 10/100 OK
✓ Packet 20/100 OK
...
============================================================
TEST STATISTICS (elapsed: 2.45s)
============================================================
Packets sent:       100
Packets received:   100
Bytes sent:         7200
Bytes received:     7200
Data loss count:    0
Checksum errors:    0
Timeout errors:     0
Other errors:       0
Data loss rate:     0.00%
Throughput:         2934.69 KB/s
============================================================
```

**Pi Output:**
```
[OK] Echoed packet #0 (64 bytes)
[OK] Echoed packet #10 (64 bytes)
...
============================================================
STATISTICS
============================================================
Packets received:  150
Packets echoed:    150
Bytes received:    10800
Bytes sent:        10800
CRC errors:        0
Other errors:      0
============================================================
```

---

## Understanding the Test Results

### Key Metrics

| Metric | What It Means |
|--------|--------------|
| **Data loss rate** | % of packets that didn't arrive or were corrupted (should be 0%) |
| **Checksum errors** | Packets with CRC mismatches (indicates corruption) |
| **Timeout errors** | Packets that took too long (connection lag) |
| **Throughput** | KB/s transfer rate |

### What's Being Tested

1. **Packet Integrity**: Each packet has a checksum that verifies no bits were flipped
2. **Packet Loss**: Counts missing packets
3. **Echo Verification**: Received data must exactly match sent data
4. **Various Sizes**: Tests from 1 byte to 1024 bytes to find breaking points
5. **Sustained Transfer**: Throughput test measures consistent data transfer

---

## Troubleshooting

### "Failed to connect to COM13"
- Check Device Manager to confirm COM13 exists
- Verify USB cable is connected to STM32 board
- Check STM32 firmware has USB CDC enabled

### Pi shows "Connection refused" or "Cannot open /dev/ttyGS0"
- Verify USB gadget mode is enabled: `ls -la /dev/ttyGS0`
- Check dmesg for USB gadget errors: `dmesg | tail -20`
- Reboot if modules weren't loaded properly

### Data loss detected
- **Hardware issue**: Check USB cable quality
- **Buffer overflow**: Pi may not be reading fast enough
- **Baud rate**: Try 9600 if 115200 is unstable
- **Cable length**: Long cables can cause issues; try shorter cable

### "Checksum errors"
- Indicates data corruption on the wire
- Try different USB port on PC
- Check for EM interference near USB cable
- Test with shorter cable

### Timeout errors (slow response)
- Pi's CPU might be overloaded
- Try reducing test intensity (smaller packet sizes)
- Check Pi temperature: `vcgencmd measure_temp`
- Ensure no other USB gadgets are running

---

## Advanced Testing

### Test Specific Scenarios

**Edit `usb_cdc_test_pc.py`** for custom tests:

```python
# In main() function, modify test parameters:
tester.test_echo(num_packets=1000, packet_size=512)  # More packets
tester.test_stress(iterations=20)  # More iterations
```

### Monitor Real-time Data

**On Pi**, open second terminal:
```bash
# Watch serial data in hex
sudo strace -e write python3 usb_cdc_test_pi.py
```

**On PC**, capture to file:
```python
# Add to PC test:
with open('usb_test_log.txt', 'w') as f:
    f.write(str(tester.stats))
```

---

## Reliability Benchmarks

### Expected Results (Good Connection)
- **Data loss rate**: 0%
- **Checksum errors**: 0
- **Throughput**: 1-2 MB/s (depends on USB version and STM32 implementation)

### Expected Results (Marginal Connection)
- **Data loss rate**: < 1%
- **Checksum errors**: 0-5
- **Throughput**: 100-500 KB/s

### Critical Issues
- **Data loss > 5%**: Connection unreliable
- **Checksum errors > 10**: Electrical noise issue
- **Frequent timeouts**: Pi not keeping up

---

## Notes

- Both scripts use **XOR checksums** for quick verification
- Packet format: `[START] [PKT_NUM:2] [LEN:2] [DATA] [CHECKSUM]`
- Default baudrate: **115200** (can be changed in both scripts)
- Tests run for several seconds; allow full completion for accurate stats
- Press **Ctrl+C** to stop either script cleanly

---

## Additional Resources

- [USB CDC Class Driver Spec](https://www.usb.org/document-library)
- [STM32L496 USB Implementation](https://www.st.com/)
- [Raspberry Pi USB Gadget Mode](https://www.raspberrypi.com/documentation/computers/os.html)
