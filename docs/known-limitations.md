# Known Limitations

This list covers the code as it stood at the end of the project. The issues are documented rather than fixed, so the repository still matches what was tested. If you reuse the code, these are good places to start improving it.

## OBC firmware

| Area | Limitation |
| :--- | :--- |
| System Status (`0x05`) | If the Pi doesn't answer within 1 s, the firmware builds a NAK but never sends it. Instead it transmits a leftover internal OBC→Pi frame, which the ground station cannot parse. The ground station sees no valid reply. |
| System Status (`0x05`) | The EPS status byte is always `0`, even when the EPS is not answering. |
| File Info (`0x02`) | The status byte is always `0`. A missing file still reports success, and the size field is undefined. |
| List files (`0x01`) | Copies up to 20 names into a fixed array without checking the limit, so more than 20 files on the card overflows the buffer. The reply must also fit in 256 bytes. When the card is empty there is no reply at all. |
| Uplink parsing | `commu_decode()` copies `DataLen` bytes into a 128-byte buffer without checking the length. An oversized request can overflow it. |
| Copy to SD (`0x03`) | The "finished" reply is also sent, with status `0`, when the USB transfer **times out**. The ground station can't tell a completed copy from an aborted one. |
| Copy to SD (`0x03`) | The source (Pi file 1) and destination (`0.jpg`) are fixed. Each copy overwrites the previous image. |
| Capture (`0x02`) | The OBC replies "OK" right away, before the Pi has taken the photo. |
| Download windows | The window counter isn't reset when a new File Data request arrives. If the previous download ended partway through a window, the first window of the next one is short. The ground station's 2 s retry recovers from it. |
| SeqNum | Not used to detect duplicate or out-of-order frames. The ground station relies on the chunk offset instead. |
| Beacon | If the EPS stops answering after answering once, the beacon keeps sending the **last** EPS data it received, without marking it as old. |
| Unused hardware | `USART1`, `USART3` and `CAN2` are set up but not used. The `TEMP_ALERT` and `RTC_INT` interrupt pins are configured but no handler uses them. |

## Python tools

| Area | Limitation |
| :--- | :--- |
| `OBC.py` System Status | The emulator returns an older 35-byte layout. `GS.py` and `Sniffer.py` expect the firmware's 26 bytes and print a length-mismatch warning. |
| `GS.py` Pi Status | The VR `0x01` reply decoder uses an older 16-byte layout. The firmware never sends this frame to the ground station, so in practice the code never runs. |
| `GS.py --test` | The option is accepted but does nothing. |
| `test_resume_downlink.py` | Says MD5 checking will be skipped if `sd_card/0.jpg` is missing, but then crashes reading that file's size. Keep a reference copy there. |

## VR payload (`vr.py`)

| Area | Limitation |
| :--- | :--- |
| Disk usage | Computed as `ceil(used / total)`, which gives `1` for any disk that isn't empty instead of a percentage. |
| Camera status | Always reports `1` (ready). It does not check the camera. |
| Configuration | Serial port, image paths and the boot-count file are hard-coded or read from small text files (`comport.txt`). |

## Hard-coded environment values

Change these before running the scripts on your own machines:

| File | Value |
| :--- | :--- |
| `GS.py`, `OBC.py`, `Sniffer.py`, tests | Default serial ports for each OS (override with `--port` / `--gs_port`) |
| `eps_and_payload_emulator/kiss_file_transfer/vr.py` | Boot-count file path, default serial port |
| `eps_and_payload_emulator/kiss_file_transfer/capturepicture.py` | Absolute output path of the photo |
| `eps_and_payload_emulator/OBC_EPS_Script/eps.py` | `COM_PORT` |
| `unison_sync.sh` | SSH user, host and remote path of the Pi |
| `check.py` | Source and destination image paths |
