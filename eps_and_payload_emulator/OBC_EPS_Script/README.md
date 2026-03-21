# 🛰️ EPS Simulator

This project contains Python implementations for communicating with the NBSPACE Electrical Power System (EPS). It includes two main components:

1. **OBC Flight Software:** Acts as the On-Board Computer to request telemetry and control switches.
2. **EPS Simulator:** Mocks the EPS hardware behavior for development and testing without physical hardware.

## 📂 Files

* `eps.py` - Emulates the EPS firmware (responds to UART commands).
* `obc.py` - The Flight Software that polls sensors and controls power rails.

## ⚙️ Prerequisites

* Python 3.x
* `pyserial` library

```bash
pip install pyserial

```

## 🔌 Setup (Virtual Serial Port)

Since this communicates over UART, you need a pair of connected serial ports to let the scripts talk to each other on the same computer.

**Windows:**

1. Install **com0com** or a similar Virtual COM Port driver.
2. Create a pair, e.g., `COM10` <-> `COM11`.

**Linux / macOS:**

1. Use `socat` to create a virtual pair:
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0

```


2. Note the output ports (e.g., `/dev/pts/3` and `/dev/pts/4`).

## 🚀 How to Run

**1. Start the EPS Simulator**
Open a terminal and run the simulator. Ensure the `COM_PORT` variable in the script matches one end of your pair (e.g., `COM10`).

```bash
python eps_simulator.py

```

*Output:* `✅ EPS Simulator Online... Waiting for commands...`

**2. Start the OBC Flight Software**
Open a **new** terminal and run the OBC script. Ensure the `OBC_PORT` variable matches the **other** end of the pair (e.g., `COM11`).

```bash
python obc_flight_sw.py

```

## 📡 Protocol Overview

The system uses a KISS-like framing structure with SLIP escaping.

* **Frame:** `0xC0` (Start) | `0x00` (Header) | `Payload` | `0xC0` (End)
* **Escaping:** `0xC0`  `0xDB 0xDC`, `0xDB`  `0xDB 0xDD`

| Command ID | Function |
| --- | --- |
| **0x01** | Get Solar/Battery Data (V/I) |
| **0x02** | Get Output Power Data (V/I) |
| **0x03** | Get Switch Status |
| **0x04** | Get Battery Temperature |
| **0x05** | Set Switch State (ON/OFF) |