# OBC-GS Telemetry Protocol (KISS + Sliding Window)

## Overview
This repository contains the Python prototyping code for a custom ground-to-satellite RF communication link. It is designed specifically for an STM32L4-based On-Board Computer (OBC) operating within a CubeSat architecture (such as the SpaceFruit / KNACKSAT-4 project). 

The protocol facilitates reliable, high-throughput telemetry transfer over a slow, half-duplex RF433 module (specifically the AS-32) using a Master/Slave polling architecture combined with a Sliding Window Automatic Repeat reQuest (ARQ) mechanism.

## Hardware Constraints & Protocol Design
The AS-32 RF module is half-duplex and susceptible to over-the-air collisions and buffer overruns. To mitigate this, the protocol operates on two distinct logical layers:
1. **Application Layer (Master/Slave):** The Ground Station (GS) acts as the Master. The OBC never transmits spontaneously. It only sends data when explicitly requested by the GS.
2. **Link Layer (Sliding Window):** Once the GS requests data, the OBC blasts a sequence of packets (the window). The GS must reply with an Acknowledge (ACK) packet indicating the highest in-order sequence number received, allowing the OBC to slide its window forward.

## Frame Architecture
The communication is wrapped in the standard KISS (Keep It Simple, Stupid) protocol for byte-boundary framing, allowing recovery from dropped bytes in the RF link.

### Full Byte Structure
| FEND | Command | PayloadID | PID | SeqNum | DataLen | Data   | CRC32 | FEND |
| :--- | :------ | :-------- | :-- | :----- | :------ | :----- | :---- | :--- |
| 1B   | 1B      | 1B        | 1B  | 1B     | 2B      | < 64kB | 4B    | 1B   |
| 0xC0 | KISS    | Dest/Src  | Type| Window | Size    | Payload| Check | 0xC0 |

*Note: The KISS layer escapes any internal `0xC0` (FEND) or `0xDB` (FESC) bytes before transmission.*

### Field Definitions
#### 1. KISS Layer
* **`FEND` (0xC0):** Frame End marker. Indicates the start and end of a packet.
* **`Command`:** Dictates the state machine action.
  * `0x00`: **Data Frame** (Contains telemetry/mission data).
  * `0x01`: **Request Frame** (GS polling the OBC for data).
  * `0xAC`: **ACK Frame** (GS acknowledging receipt of a sliding window sequence).

#### 2. Custom Payload Layer (Calculated in CRC)
* **`PayloadID`:** Routes the packet to the subsystem.
  * `0x00`: OBC System
  * `0x01`: VR Payload (Mission)
* **`PID` (Parameter ID):** Identifies the specific telemetry type within the `Data` field.
  * `0x01`: Solar Cell Voltage
  * `0x02`: Solar Cell Current
  * `0x03`: Battery Voltage
  * `0x04`: Battery Current
  * `0x05`: Battery Temperature
* **`SeqNum` (Sequence Number):** 8-bit rolling counter (`0x00` to `0xFF`) for the sliding window. Increments only for new Data Frames (`Command 0x00`).
* **`DataLen`:** 16-bit unsigned integer (Big-Endian) defining the size of the `Data` field.
* **`Data`:** The actual telemetry value (e.g., a 4-byte IEEE 754 Big-Endian float).
* **`CRC32`:** 32-bit Standard Ethernet/Zlib CRC calculated over the Custom Payload (`PayloadID` through `Data`). 

## State Machine Execution
**Scenario: GS requests telemetry from the OBC.**
1. **The Ask:** GS transmits `Command = 0x01`, `SeqNum = 0x00`, `DataLen = 0`.
2. **The Burst:** OBC receives the request. It reads the sensors and transmits a burst of packets (`Command = 0x00`), incrementing `SeqNum` for each distinct sensor (`PID`).
3. **The ACK:** GS receives the burst, verifies the CRCs, un-escapes the KISS framing, and notes the highest contiguous `SeqNum` received. It transmits an ACK (`Command = 0xAC`) with that `SeqNum`.
4. **The Slide:** OBC receives the ACK, clears those packets from its unacknowledged buffer, and prepares for the next GS request.

## File Structure
* **`Shared/Python/kiss_protocol.py`**: A static utility class handling FEND/FESC escaping, frame wrapping/unwrapping, and CRC-32 calculation.
* **`OBC.py`**: The Slave emulator. Listens for `0x01` requests, generates dummy float telemetry, packs it using `struct.pack('>f')`, and sends the sliding window burst.
* **`GS.py`**: The Master emulator. Initiates polling loops, parses incoming Data frames, unpacks telemetry values, and manages the transmission of `0xAC` window acknowledgments.

## Hardware Implementation Notes for C/C++ Porting
When porting this logic to the STM32L4:
* **Endianness:** Network byte order (Big-Endian) is used for `DataLen`, `Data` (floats), and `CRC32`. STM32 is Little-Endian; byteswapping (`__REV`) will be required.
* **Flow Control:** The AS-32's `AUX` pin must be tied to an EXTI interrupt to pause the UART DMA when the module's internal TX buffer is full or when it is transmitting over the air.