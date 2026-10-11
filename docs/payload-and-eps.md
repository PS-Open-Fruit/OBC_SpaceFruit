# VR Payload and EPS Simulator

Everything outside the OBC firmware and the ground station is under [`eps_and_payload_emulator/`](../eps_and_payload_emulator/):

| Folder | Contents |
| :--- | :--- |
| [`kiss_file_transfer/`](../eps_and_payload_emulator/kiss_file_transfer/) | Software for the **VR payload** (Raspberry Pi Zero 2W), plus earlier image-transfer experiments |
| [`OBC_EPS_Script/`](../eps_and_payload_emulator/OBC_EPS_Script/) | **EPS simulator**, for running the OBC without the real power board |

---

## VR payload (Raspberry Pi Zero 2W)

The payload is a Raspberry Pi Zero 2W with a camera. It connects to the OBC by USB, and the OBC appears on the Pi as a serial port (USB CDC). The main program is **`vr.py`**. It waits for frames from the OBC and answers them. The frame format and PIDs are in [protocol.md §7](protocol.md#7-obc--vr-payload-link).

| Request from the OBC | What `vr.py` does |
| :--- | :--- |
| Ping `0x00` | Answers with an empty ping frame |
| Status `0x01` | Collects boot count, uptime, CPU load, CPU temperature, RAM and disk usage (`psutil`, `/sys/class/thermal`) and answers with 20 bytes |
| Capture `0x02` | Starts `capturepicture.py` in the background and sends an ACK |
| Image request `0x03` | Sends an ACK, then streams `source-img/testimg-<id>.jpg` in 8 KB chunks, waiting for an ACK after each one, and finishes with `0x88` |
| Shutdown `0x90` | Sends an ACK, waits 2 s, then runs `sudo systemctl poweroff` |

While it works, `vr.py` prints a colour log, a progress bar during transfers, and a summary at the end (bytes sent, time, kbps).

### Files in `kiss_file_transfer/`

| File | Purpose |
| :--- | :--- |
| `vr.py` | **Payload service** (the current version) |
| `capturepicture.py` | Takes a photo with `fswebcam` (`/dev/video0`, 640×480, skips 20 warm-up frames, no banner) |
| `kiss_protocol.py` | Pi-side KISS frames: `wrap_frame`, `wrap_image_chunk`, `unwrap_frame`, CRC-32/MPEG-2 |
| `config.py` | `CHUNK_SIZE = 8192` |
| `comport.txt` | Serial port path that `vr.py` opens |
| `requirements.txt` | Pinned packages. The service itself needs only `pyserial`, `psutil` and `uptime`. |
| `monitor_service.sh` | Follows the logs of a systemd unit named `payload-emulator.service` |
| `init_virtual_serial.sh` | Creates a `socat` virtual serial pair for testing on a PC |
| `main.py`, `obc.py`, `vrv2.py`, `archive/` | Earlier versions of the Pi ↔ OBC transfer, with older PID numbers. The folder's own [README](../eps_and_payload_emulator/kiss_file_transfer/README.md) describes the `main.py` / `obc.py` pair. |
| `auto_sweep.py`, `gemini.py`, `plotter.py` | Chunk-size sweep tests. They start transfers through a second "trigger" serial port (`triggerport.txt`), log throughput to CSV, and plot it. |
| `check.py`, `test_subproc.py`, `osi_protocol.py` | Small helpers and experiments |

### Setting up the Pi

1. **Install packages:**
   ```bash
   sudo apt install fswebcam
   python3 -m venv .venv && source .venv/bin/activate
   pip install pyserial psutil uptime        # or: pip install -r requirements.txt
   ```
2. **Connect to the OBC:** use the Pi's USB data port. The OBC shows up as
   `/dev/serial/by-id/usb-Openfruit_Aerospace_OBC_VCP_<serial>-if00`.
   Write that path into `comport.txt`.
3. **Check the hard-coded paths** and change them for your user and folder:
   - `vr.py` reads the Pi's boot count from a text file in the user's home directory. Create the file, or point `vr.py` somewhere else. The script that keeps this count up to date is not in this repository.
   - `capturepicture.py` saves to an absolute path ending in `payload_emulator/source-img/testimg-1.jpg`.
4. **Add images:** `vr.py` serves `source-img/testimg-0.jpg` to `testimg-3.jpg`. The OBC's Copy command always asks for **file 1**, which is the file `capturepicture.py` overwrites. (`*.jpg` is in `.gitignore`, so the folder isn't committed.)
5. **Allow shutdown:** the user running `vr.py` needs passwordless `sudo` for `systemctl poweroff`.
6. **Run it:** `python vr.py`. On the flight Pi it ran as the systemd service `payload-emulator.service`; the unit file is not in this repository. `./monitor_service.sh` follows its logs.

### Syncing code to the Pi

[`unison_sync.sh`](../unison_sync.sh) in the repository root keeps `kiss_file_transfer/` in step with a folder on the Pi over SSH. It uses [Unison](https://github.com/bcpierce00/unison), checks for changes every 2 s, and skips `.venv`. Change the SSH target and remote path to match your Pi.

```bash
./unison_sync.sh
```

---

## EPS simulator

The real EPS sends power telemetry over UART. Without the board, the OBC's beacon is only 11 bytes, because it has no EPS data to include. **`eps.py`** stands in for the EPS so you get full 121-byte beacons with realistic, randomised values.

| File | Purpose |
| :--- | :--- |
| `eps.py` | EPS simulator. Answers commands `0x01`–`0x06` (see [protocol.md §8](protocol.md#8-obc--eps-link)) with random V/I values, switch states that can be changed, and 25 °C battery temperatures |
| `obc.py` | Early Python OBC that polls the EPS. Useful for testing `eps.py` on its own. |
| `vr_simulator.py` | Early combined EPS and payload simulator, with older payload command IDs (`0x10`–`0x12`) |
| `virtual_com.sh`, `init_venv.sh` | `socat` virtual serial pair, and virtual environment setup |

To use it with the **real OBC**:

1. Connect a USB-UART adapter to the OBC's `USART2` (TX↔RX, RX↔TX, GND).
2. Set `COM_PORT` near the bottom of `eps.py` to that adapter's port. The baud rate is already 115200.
3. Run `python eps.py`. The OBC polls it once per second, and the next beacon includes the EPS block.

The folder's [README](../eps_and_payload_emulator/OBC_EPS_Script/README.md) uses older file names (`eps_simulator.py`, `obc_flight_sw.py`). The current files are `eps.py` and `obc.py`.
