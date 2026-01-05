# Serial Connection Benchmark

This tool tests the reliability and throughput of a serial connection (like USB-CDC) between two devices.

## Prerequisites

You need Python installed on both the PC and the Raspberry Pi (or other device).
Install the required library:

```bash
pip install -r requirements.txt
```

## Usage

You need to run the script on both devices, one as a **receiver** and one as a **sender**.

### 1. Start the Receiver (e.g., on the Pi)

If the Pi is the device side (Gadget), the port is likely `/dev/ttyGS0`.

```bash
python serial_benchmark.py --port /dev/ttyGS0 --role receiver
```

### 2. Start the Sender (e.g., on the PC)

If the PC is the host, find the COM port (e.g., COM13).

```bash
python serial_benchmark.py --port COM13 --role sender --size 10 --iterations 5
```

* `--size 10` sends 10 MB of data.
* `--iterations 5` runs the test 5 times in a row.

### 3. Observe Results

The sender will generate random data, calculate a checksum, and send it.
The receiver will read the data, verify the checksum, and calculate the throughput.
Both sides will report success/failure and the speed.

## Notes

- **Reliability**: The test uses MD5 checksums to ensure every byte is received correctly. Any data loss or corruption will result in a "FAILURE".
- **Throughput**: The speed is calculated based on the time it takes to transfer the data payload.
- **Baud Rate**: For USB-CDC (Virtual COM Port), the baud rate setting is often ignored by the driver and data is transferred at the maximum USB speed (Full Speed ~12Mbps or High Speed ~480Mbps). However, you can specify it with `--baud` if needed.
