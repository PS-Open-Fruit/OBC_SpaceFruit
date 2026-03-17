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
| 0xC0 | KISS    | Running number | Subsystem  | Type| Data Size    | Payload| Calculate | 0xC0 |

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
  * **`0x00`–`0x7F`:** Normal / read-only commands.
  * **`0x90`–`0xFF`:** **Dangerous / irreversible commands** (e.g., shutdown). Reserved range to prevent accidental execution.
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

#### 1. SD Card (PayloadID: `0x00`)

**Request Ping `(PID: 0x00)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (0 byte):**
    | Data |
    | :--- |
    | -    |

**Request List files (SD) `(PID: 0x01)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (Dynamic Length):**
    | Number of files (uint8) | Filename Length | Filename ASCII |
    | :--- | :--- | :--- |
    | 1B | 1B | N |

**Request File Info (SD) `(PID: 0x02)`**
*   **Request (Dynamic Length):**
    | Filename Length | Filename ASCII |
    | :--- | :--- |
    | 1B | N |
*   **Response (9 bytes):**
    | Status (0x00 = OK, 0x01 = Error) | File Size (uint32) | Created Timestamp (uint32) |
    | :--- | :--- | :--- |
    | 1B | 4B | 4B |

**Request File Data `(PID: 0x03)`**
*   **Request (Dynamic Length):**
    | Filename Length | Filename ASCII | FileOffset (uint32) | ChunkLength (uint16) |
    | :--- | :--- | :--- | :--- |
    | 1B | N | 4B | 2B |
*   **Response (Dynamic Length):**
    | Status (0x00 = OK, 0x01 = Error) | Echoed FileOffset (uint32) | Echoed ChunkLength (uint16) | Raw file byte of chunk |
    | :--- | :--- | :--- | :--- |
    | 1B | 4B | 2B | N |

#### 2. VR Payload / Pi Zero (PayloadID: `0x01`)

**Request Ping `(PID: 0x00)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (0 byte):**
    | Data |
    | :--- |
    | -    |

**Request Pi Status `(PID: 0x01)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (16 bytes):**
    | Timestamp (uint32) | Uptime (uint32) | CPU Load % (uint8) | CPU Temp (signed or un8) | RAM Usage (uint8) | Disk Usage (uint8) | Camera Status (0=Err, 1=Ready, 2=Busy, uint8) | Padding |
    | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
    | 4B | 4B | 1B | 1B | 1B | 1B | 1B | 3B |

**Request Capture `(PID: 0x02)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (Dynamic Length):**
    | Status (0x00 = OK, 0x01 = Error) | SavedFileNameLength (uint8) | SavedFileName ASCII |
    | :--- | :--- | :--- |
    | 1B | 1B | N |

**Request Copy Image to SD `(PID: 0x03)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (1 byte):**
    | Status (0x00 = OK, 0x01 = Error) |
    | :--- |
    | 1B |

**Request VR Shutdown `(PID: 0x90)`**
*   **Request (0 byte):**
    | Data |
    | :--- |
    | -    |
*   **Response (0 byte):** ACK only — the Pi sends an empty ACK frame then immediately executes `sudo shutdown -h now`. No further response will be received.
    | Data |
    | :--- |
    | -    |

#### 3. Core Sensors / EPS (PayloadID: `0x00`)

**Unsolicited EPS Beacon `(PID: 0x04)`**
*   **Request:** None (Broadcast periodically from OBC)
*   **Beacon Data (121 bytes Fixed C-Struct):**
    | RTC DateTime | 8x VI Sensors | 6x Out Sensors | 6x Out States | 2x Battery Temps | 1x TMP1075 Raw |
    | :--- | :--- | :--- | :--- | :--- | :--- |
    | 7B (date_time_t) | 48B (8 * 6B) | 36B (6 * 6B) | 18B (6 * 3B) | 8B (2 * 4B) | 4B (i32) |

## How to run the Emulator

The GS and OBC scripts contain multithreaded emulators that allow you to test this protocol logic visually in the terminal. The output is **color-coded** to show exactly where the `Command`, `SeqNum`, `PayloadID`, and `Data` fields sit within the raw hex strings.

1.  Open two terminals.
2.  Run `python OBC.py` in one. It will wait for incoming commands.
3.  Run `python GS.py` in the other. It will launch an **Interactive CLI**.

**GS CLI Interface:**
```
--- Ground Station CLI ---
Type 'help' for a list of available commands.
GS> help
Commands:
  ping <obc/vr>         - Ping subsystem
  list                  - List files on OBC SD card
  info <filename>       - Request file info
  download <filename>   - Download file from OBC
  status                - Request Pi Status
  capture               - Request Image Capture
  copy                  - Request Copy Image to SD
  shutdown              - Shutdown the VR Raspberry Pi
  exit                  - Exit Ground Station
```
## Automated Testing (Stress Test)

The `test_resume_downlink.py` script is a stress test designed to validate the reliability of the file download resume mechanism. It simulates multiple connection drops by repeatedly terminating and restarting the Ground Station process.

### Scenario: Aggressive Resume (Stress Test)
- **Objective**: Ensure 100% data integrity of a large file (e.g., `0.jpg`) across multiple forced interruptions.
- **Process**: Starts `GS.py`, starts download, kills `GS.py` every 15s, restarts, and resumes until finished.
- **Visual Feedback**: The terminal output is **color-coded** (Yellow/Green/Red) to highlight notes, success states, and failures for better readability.
- **Integrity**: Verified by MD5 hash comparison at the end. The script compares the MD5 of a **local reference file** (`sd_card/0.jpg`) with the downloaded file (`downloads/0.jpg`). 
    - *Note: If `sd_card/0.jpg` is missing from your PC, the test will still run and complete, but it will skip the MD5 verification step.*

### How to Run with Real OBC
1.  **Hardware setup**: Connect your STM32 OBC to your PC. Identify the COM port (e.g., `COM5`).
2.  **Run the test**: 
    ```powershell
    python test_resume_downlink.py --gs_port COM4 --kill_interval 15
    ```
    *Note: Adjust `--gs_port` to match your local Ground Station RF bridge port. The script assumes the real OBC is already listening on the other side of the RF link.*

### How to Run with Emulator
1.  **Terminal 1 (OBC)**:
    ```powershell
    python OBC.py --port COM5
    ```
2.  **Terminal 2 (Test Harness)**:
    ```powershell
    python test_resume_downlink.py --gs_port COM4 --kill_interval 15
    ```