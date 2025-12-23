# CAN File Transfer Test

Fast, reliable file transfer over CAN bus with CRC32 integrity checking.

## Features

- **Speed**: ~2.6 KB/s at 250 kbps bitrate
- **Reliability**: CRC32 checksums verify data integrity
- **Handshaking**: READY/ACK protocol ensures receiver is online before transfer
- **Progress Reporting**: Real-time speed and ETA display
- **5-byte chunks**: Optimized for 8-byte CAN frame limitation

## Usage

### Start Receiver (Pi)
```bash
python test_file_transfer.py recv can0 250000
```

### Start Sender (PC)
```bash
python test_file_transfer.py send LICENSE COM6 250000
```

**Arguments:**
- `interface`: `COM6` (PC/USB adapter) or `can0` (Pi/SocketCAN)
- `bitrate`: 250000 (default), 500000, 1000000, etc.

## Protocol

| Phase | Message | From | To | Content |
|-------|---------|------|----|----|
| **Handshake** | READY | Sender | Receiver | Check if ready |
| | READY_ACK | Receiver | Sender | Ready to receive |
| **Transfer** | START | Sender | Receiver | File size + name length |
| | FILENAME | Sender | Receiver | File name (multi-frame) |
| | DATA | Sender | Receiver | 5-byte chunks (sequenced) |
| | END | Sender | Receiver | CRC32 checksum |
| **Complete** | END_ACK | Receiver | Sender | Transfer confirmed |

## CAN IDs

- **Sender → Receiver**: 0x100
- **Receiver → Sender**: 0x101

## Example Output

**Sender:**
```
[SENDER] Checking if receiver is ready...
[SENDER] Receiver is ready!
[SENDER] Sent 17500/35823 bytes (48.9%) | Speed: 2678.4 B/s | ETA: 6.8s
[SENDER] ✓ File transfer complete - ACK received!
```

**Receiver:**
```
[RECEIVER] READY message received
[RECEIVER] Filename received: LICENSE
[RECEIVER] DATA 500 - received 2505/35823 bytes (7.0%) | Speed: 649.7 B/s | ETA: 51.3s
[RECEIVER] ✓ CRC32 MATCH - File transfer successful!
```

## Files

- `test_file_transfer.py`: Main test script
- `canbus.py`: Low-level CAN interface with auto-queueing
- `received_<filename>`: Downloaded file on receiver

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Receiver not ready" | Make sure receiver is running first |
| "CRC32 MISMATCH" | Check CAN connection/bitrate mismatch |
| "No data received" | Verify CAN interface is up: `ip link show can0` |
| Slow transfer | Reduce `time.sleep(0.001)` in `_send_data_chunk()` |
