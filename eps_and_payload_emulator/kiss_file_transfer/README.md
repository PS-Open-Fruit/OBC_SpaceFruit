## 📁 Directory and key files

- kiss_protocol.py : core KISS packet framing/unframing, CRC, escape/unescape, image chunk wrapper.
- main.py : sender/VR-side process that receives requests and sends image chunks.
- obc.py : requester/host-side process that asks for file transfer and writes chunks to disk.
- config.py : global constant `CHUNK_SIZE`.
- requirements.txt : dependencies (especially `pyserial`).
- `comport.txt` : optional serial port override.

---

## 🔧 Dependencies

Required for operation:

- Python 3.11+ (type hints in files)
- `pyserial` (essential for serial comms)
- `termcolor` (for log color, optional)
- `numpy`, `opencv-python` etc are in the repository requirements.txt but not required for core KISS path.

Install:

- `python -m pip install -r requirements.txt`
- Or minimum: `pip install pyserial`

---

## 🧩 kiss_protocol.py explained

### 1. Constants
- FEND=0xC0 start/end frame delim
- FESC=0xDB escape byte
- TFEND/TFESC for KISS escaping
- payload IDs / VR PIDs (control commands & data types)

### 2. CRC32 function
- Custom implementation (STM32/MPEG-2 style), CRC is over payload+header (no frame delimiters).

### 3. KISS escaping
- `escape()` / `unescape()` to encode/decode KISS control bytes inside payload.

### 4. Layers
- `encode_layer_kiss()` / `decode_layer_kiss()` for KISS framing (FEND, command, escaped payload).
- `wrap_frame(payload_id, pid, data)` generic frame:
  - `[FEND][CMD][payload_id][pid][len(2)][data][crc4][FEND]`
- `wrap_image_chunk(file_id, chunk_id, content, pid=0x02)`:
  - `[FEND][CMD][PAYLOAD_IMAGE][PID][file_id][chunk_id(2)][len(2)][content][crc32][FEND]`

### 5. `unwrap_frame(frame)`
- Validate FEND boundaries and length
- Unescape + parse command+payload
- CRC check
- Distinguish:
  - image frame (VR PID IMAGE_DOWNLOAD)
  - generic frame (all others)
- Returns structured dict with type, pid, chunk, content etc.

---

## 🔁 main.py (sender / VR module) flow

### Startup
- Serial port chosen from `comport.txt` or default ttys005.
- Config sets `IMAGES`, `CHUNK_SIZES`, `FILE_TO_SAVE`.

### `getCaptureRequest()`
State machine:
- `FILE_MODE_IDLE` (waiting).
- On `IMAGE_REQUEST`:
  - ACK back
  - decode request payload: `file_id`, `chunk_id`
  - if `chunk_id==0xFFFF` → full image transfer.
  - else single chunk request.
- If in `FILE_MODE_DUMP`:
  - reads image from disk
  - sends chunk via `KISS.wrap_image_chunk(...)`
  - sets `FILE_MODE_WAIT_ACK`
- On `PID_ACK` in WAIT_ACK:
  - increment chunk id, return to DUMP to send next.
- On all sent:
  - send `VR_PID_IMAGE_DOWNLOAD_DONE` generic frame.
  - print and append metrics on `test_log_automated.csv`.

### `run()`
- Loops through images and chunk sizes for automated stress test:
  - Connect serial port
  - send `TRIGGER_FRAME` to carrier side
  - call `getCaptureRequest()`

### IO helpers
- `connect_serial()`, `send_data()`, `recv_whole_frame()`
- `recv_whole_frame()` buffers until sees two FENDs.

---

## 📥 obc.py (host-side request/receiver)

### `sendRequestTransferCommand()`
- Builds image request (calls `KISS.wrap_frame`):
  - `payload_id=VR`, `pid=VR_PID_GET_IMAGE_REQUEST`
  - request payload: file_id + chunk_id (0xFFFF for full dump)
- Write request to serial.

### Receive loop
- `recv_whole_frame()` receives KISS frames and unwrap.
- if generic ack -> logs.
- if image chunk (`type == image`, pid=IMAGE_DOWNLOAD):
  - append chunk to `"{file_id}.jpg"` (new for chunk 0, append for subsequent)
  - send back `ACK` via wrapper frame.
- This loop continues indefinitely.

---

## 🪜 How to run

### On VR (sender) board emulator (in `kiss_file_transfer`)
1. Configure serial path:
   - edit `comport.txt` to actual port name (or adjust `SERIAL_PORT` in main.py).
2. put image(s) in folder named e.g., `testimg-1.jpg`.
3. Run:
   - `python main.py`

### On OBC host (receiver)
1. configure port in obc.py (default ttys004).
2. run:
   - `python obc.py`
3. output files:
   - `0.jpg`, `1.jpg`, ... if `file_id` varies.

---

## 💡 Behavior summary

- KISS link layer: protects frame delimiters with escaped `0xC0/0xDB`.
- Transport layer: `payload_id/pid` + length + CRC.
- Handshake is simple:
  - host sends IMAGE_REQUEST
  - VR sends ACK and then chunk(s)
  - host receives chunks + writes file + sends ACK per chunk
  - VR iterates chunk mode until done, then sends IMAGE_DOWNLOAD_DONE.

---

## 🧪 Testing and metrics

- main.py logs per-chunk progress bar and CSV metrics:
  - `chunk_size`, `total_bytes`, `elapsed_s`, `rate_kbps`, `rate_KBps`
- During automated run, cycling chunk sizes with repeat count `NUM_TEST`.

---

## 🛠️ Notes / gotchas

- `recv_whole_frame()` in main.py can return partial data if serial chunking not aligned; works only if contiguous FENDs appear.
- CRC check is strict; out-of-order or missing chunks is handled only by `ACK` loop.
- There is no explicit retransmission strategy beyond repeated command cycles.
- main.py uses a hardcoded second serial port for request trigger:
  - `"/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10OMHTZ-if00-port0"`
