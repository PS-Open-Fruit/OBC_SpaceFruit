# OBC-GS Telemetry Protocol 

## Overview
This repository contains the Python prototyping code for a custom ground-to-satellite RF communication link. It is designed specifically for an STM32L4-based On-Board Computer (OBC) operating within a CubeSat architecture communicating over a half-duplex RF433 module.

The protocol encapsulates telemetry and file transfer logic into a custom layered architecture using KISS framing.

## Architecture Layers

The communication is structured into three contextual layers to handle data routing flawlessly across subsystems:

1. **Layer 1: KISS Framing (Link Layer)**
2. **Layer 2: Routing (Subsystem Layer)**
3. **Layer 3: Packed Data (Application Layer)**

### Full Byte Structure
| FEND | Command | SeqNum | PayloadID | PID | DataLen | Data   | CRC32 | FEND |
| :--- | :------ | :----- | :-------- | :-- | :------ | :----- | :---- | :--- |
| 1B   | 1B      | 1B     | 1B        | 1B  | 2B      | < 64kB | 4B    | 1B   |
| 0xC0 | KISS    | Window | Dest/Src  | Type| Size    | Payload| Check | 0xC0 |

*Note: The KISS layer escapes any internal `0xC0` (FEND) or `0xDB` (FESC) bytes before transmission.*

---

## Layer Definitions

### 1. Link Layer (KISS)
Handles standard packet delimiting overhead.
* **`FEND` (0xC0):** Frame End/Start marker.
* **`Command`:** Defines traffic direction natively without looking at payloads.
  * `0x00`: **Request Frame** (GS -> OBC)
  * `0x01`: **Data/Response Frame** (OBC -> GS)
  * `0xAC`: **ACK Frame** (Acknowledgment windowing)

### 2. Routing Layer (Custom Header)
* **`SeqNum`:** Rolling 8-bit counter ensuring sequential delivery.
* **`PayloadID` (Subsystem Dest/Src):** Serves as an internal router for the OBC.
  * `0x00`: **OBC native** (SD Card, Core Sensors)
  * `0x01`: **VR Payload** (Pi Zero 2W)
* **`PID` (Type):** The specific action requested within the subsystem context.
  * e.g., For `PayloadID 0x00`, `PID 0x00` means *List SD Files*. For `PayloadID 0x01`, `PID 0x00` means *Get Pi Status*.
* **`DataLen`:** 16-bit Big-Endian length of the following `Data` field.

### 3. Application Layer (Data Payload)
Instead of string parsing, Data payloads are strictly packed binary C-structs.

**Standard Data Types & Status Codes:**
*   `uint8_t` (1 byte, unsigned) - Python `struct` format: `B`
*   `uint16_t` (2 bytes, unsigned) - Python `struct` format: `H`
*   `uint32_t` (4 bytes, unsigned) - Python `struct` format: `I`
*   `int8_t` (1 byte, signed) - Python `struct` format: `b`

**Common Status Byte (returned in most responses):**
* `0x00`: Success (OK) / `0x01`: File Not Found / Generic Error / `0x02`: Device Busy

#### 1. SD Card Subsystem (PayloadID: `0x00`)

**Request List Files `(PID: 0x00)`**
*   **Request Data**: `[Empty]`
*   **Response Data**:
    *   `[NumFiles (uint8_t)]` -> Count of files
    *   *Followed by `NumFiles` entries of:* `[FileNameLength (uint8_t)]` + `[FileName (ascii array)]`

**Request File Info `(PID: 0x01)`**
*   **Request Data**: `[FileNameLength (uint8_t)]` + `[FileName (ascii array)]`
*   **Response Data**: `[Status (uint8_t)]` + `[FileSize (uint32_t)]` + `[CreatedTimestamp (uint32_t)]`

**Request File Data `(PID: 0x02)`**
*(Split into chunks to abide by KISS MTU constraints)*
*   **Request Data**: `[NameLen (uint8_t)]` + `[Name (ascii)]` + `[Offset (uint32_t)]` + `[ReadLength (uint16_t)]`
*   **Response Data**: `[Status (uint8_t)]` + `[Offset (uint32_t)]` + `[ActualDataLength (uint16_t)]` + `[Raw File Bytes]`

#### 2. VR Payload / Pi Zero (PayloadID: `0x01`)

**Request Pi Status `(PID: 0x00)`**
*   **Request Data**: `[Empty]`
*   **Response Data**: 16-byte fixed structure.
    *   `[Timestamp (uint32_t)]` -> Unix epoch time on the Pi.
    *   `[Uptime (uint32_t)]` -> Seconds since boot.
    *   `[CPULoadPercent (uint8_t)]` -> e.g., 25 for 25%.
    *   `[CPUTemp (int8_t)]` -> e.g., 45 for 45°C.
    *   `[RAMUsagePercent (uint8_t)]` -> e.g., 60 for 60%.
    *   `[DiskUsagePercent (uint8_t)]` -> e.g., 85 for 85%.
    *   `[CameraStatus (uint8_t)]` -> 0x00=Error, 0x01=Ready, 0x02=Busy capturing.
    *   *Reserved 3 padding bytes to align to 32 bits.*

**Request Capture `(PID: 0x01)`**
*   **Request Data**: `[Empty]`
*   **Response Data**: `[Status (uint8_t)]` + `[SavedFileNameLength (uint8_t)]` + `[SavedFileName (ascii array)]`

**Request Copy Image to SD `(PID: 0x02)`**
*   **Request Data**: `[FileNameLength (uint8_t)]` + `[FileName (ascii array)]`
*   **Response Data**: `[Status (uint8_t)]`


## How to run the Emulator

The GS and OBC scripts contain multithreaded emulators that allow you to test this protocol logic visually in the terminal. The output is **color-coded** to show exactly where the `Command`, `SeqNum`, `PayloadID`, and `Data` fields sit within the raw hex strings.

1.  Open two terminals.
2.  Run `python OBC.py` in one. It will wait for incoming commands.
3.  Run `python GS.py` in the other. It will launch an **Interactive CLI**.

**GS CLI Interface:**
```
--- Command Line Interface ---
Commands:
  0: Request List files (SD)
  1: Request File Info (SD)
  2: Request File Data (SD)
  3: Request Pi Status (VR)
  4: Request Capture (VR)
Type the number and press Enter to send.
```
*Depending on the command chosen (like requesting file info), the CLI will dynamically prompt you for the specific parameters (File Name, Chunk Offset) needed to build the packed binary request.*