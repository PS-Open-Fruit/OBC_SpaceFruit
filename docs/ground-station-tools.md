# Ground Station and Test Tools

The Python scripts in the repository root run on a PC. They talk to the OBC through an RF433 module (or a USB-UART adapter) at 9600 baud. With a virtual serial port pair they can also talk to the OBC emulator, so you can try the whole protocol without any hardware.

- [Setup](#setup)
- [GS.py: ground-station CLI](#gspy-ground-station-cli)
- [OBC.py: OBC emulator](#obcpy-obc-emulator)
- [Sniffer.py: traffic monitor](#snifferpy-traffic-monitor)
- [Trying it without hardware](#trying-it-without-hardware)
- [Automated tests](#automated-tests)
- [Shared Python modules](#shared-python-modules)

---

## Setup

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate    Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt      # pyserial, pandas, matplotlib
```

**Run every script from the repository root.** They import `Shared.Python.*` relative to the current directory.

Each script picks a default serial port for your OS. Override it with `--port` (or `--gs_port` in the test scripts).

| Script | Windows | Linux | macOS |
| :--- | :--- | :--- | :--- |
| `GS.py`, `Sniffer.py`, tests | `COM4` | `/dev/ttyUSB0` | `/dev/cu.usbserial-A10OMHTZ` |
| `OBC.py` | `COM5` | `/dev/ttyUSB1` | `/dev/cu.usbserial-XXXXXXXX` |

---

## GS.py: ground-station CLI

```bash
python GS.py --port COM4 --baud 9600
```

| Option | Default | Meaning |
| :--- | :--- | :--- |
| `--port` | see table above | Serial port of the ground-station radio |
| `--baud` | `9600` | Baud rate |
| `--test` | off | Accepted but has no effect yet |

Commands at the `GS>` prompt:

| Command | Request sent | What you see |
| :--- | :--- | :--- |
| `ping obc` | OBC `0x00` | Round-trip time in ms |
| `ping vr` | VR `0x00` | Round-trip time through the OBC to the Pi |
| `list` | OBC `0x01` | Files on the SD card |
| `info <file>` | OBC `0x02` | Status, size, and creation time (Unix) |
| `download <file>` | OBC `0x02`, then `0x03` repeatedly | Progress bar, speed and ETA. The file is saved to `downloads/<file>`. |
| `status` | OBC `0x05` | OBC boot count, USB/EPS status, Pi health |
| `capture` | VR `0x02` | Capture accepted |
| `copy` | VR `0x03` | Copy started, then copy finished |
| `shutdown` | VR `0x90` | Shutdown acknowledged |
| `help`, `exit` | – | – |

Beacons arrive in the background and are decoded and printed over the prompt. Every frame sent or received is also shown as raw hex, coloured by field:

<code>FEND</code> · <code>CMD</code> · <code>SEQ</code> · <code>PL_ID</code> · <code>PID</code> · <code>LEN</code> · <code>DATA</code> · <code>CRC</code> · <code>FEND</code>

### How `download` works

1. Sends **File Info** to get the total size.
2. If `downloads/<file>` already exists and is smaller than the total, **resumes** from its current size. Otherwise starts at offset 0.
3. Sends **File Data** with 350-byte chunks. The OBC answers with up to 5 chunks; each one is written to the file at its offset.
4. After 5 chunks, requests the next window from the new offset.
5. If nothing arrives for **2 s**, re-requests the same offset, up to **5 times**, before giving up.
6. Stops when a chunk is shorter than 350 bytes or the full size has arrived, then prints the total time and average speed.

Chunks with an unexpected offset are ignored. That way a late duplicate cannot corrupt the file.

---

## OBC.py: OBC emulator

Plays the OBC's part of the protocol on a PC, so you can develop and test `GS.py` without hardware.

```bash
python OBC.py --port COM5 --beacon --interval 5
```

| Option | Default | Meaning |
| :--- | :--- | :--- |
| `--port` | see table above | Serial port |
| `--baud` | `9600` | Baud rate |
| `--beacon` | off | Send a dummy 121-byte beacon periodically |
| `--interval` | `5.0` | Beacon period in seconds |

It uses two local folders, created automatically:

| Folder | Plays the role of |
| :--- | :--- |
| `sd_card/` | The OBC's SD card. `list`, `info` and `download` serve the files in it. |
| `pi_files/` | The Pi's storage. `copy` copies everything here into `sd_card/`. |

Differences from the real firmware:

- Every reply carries an incrementing SeqNum.
- `capture` returns a made-up file name instead of an empty one.
- `ping vr` and the Pi status are always answered with dummy data.
- `copy` replies once, after copying, instead of twice.
- **System Status returns a 35-byte, older extended layout.** `GS.py` expects the firmware's 26 bytes, so it prints a length-mismatch warning. See [known limitations](known-limitations.md).

---

## Sniffer.py: traffic monitor

A receive-only decoder. Connect it to a third radio, or tap the RX line, and it prints every frame it sees in either direction, with full field decoding and beacon contents. It never transmits.

```bash
python Sniffer.py --port COM6 --baud 9600
```

---

## Trying it without hardware

You need two serial ports connected to each other.

- **Windows:** install [com0com](https://com0com.sourceforge.net/) and create a pair, for example `COM4` ↔ `COM5`.
- **Linux/macOS:** `socat -d -d pty,raw,echo=0 pty,raw,echo=0` prints two `/dev/pts/N` paths.

```bash
# Terminal 1: emulated OBC with beacons
python OBC.py --port COM5 --beacon

# Terminal 2: ground station
python GS.py --port COM4
GS> list
GS> download 0.jpg
```

Put any file in `sd_card/` first so there is something to list and download.

---

## Automated tests

### `test_gs_cli.py`: smoke test

Starts `GS.py` and sends `ping obc`, `ping vr`, `list`, `copy`, `info <file>`, `status` and `capture` one by one, logging the output. Use it as a quick check after protocol changes.

```bash
python test_gs_cli.py --gs_port COM4 --filename 0.jpg
```

### `test_resume_downlink.py`: resume stress test

Checks that a download survives repeated connection drops:

1. Starts `GS.py` and runs `download 0.jpg`.
2. Kills `GS.py` every `--kill_interval` seconds and keeps it offline for a random 25–45 s.
3. Restarts it, and `GS.py` resumes from the partial file.
4. When the download finishes, compares the **MD5** of `downloads/0.jpg` with the local reference `sd_card/0.jpg`.

The script needs a local copy of the file at `sd_card/0.jpg`. With the emulator, that is the file being served. With real hardware, put the same image there.

```bash
python test_resume_downlink.py --gs_port COM4 --kill_interval 15
```

| Option | Default | Meaning |
| :--- | :--- | :--- |
| `--gs_port`, `--baud` | OS default, `9600` | Ground-station serial port |
| `--kill_interval` | `15` | Seconds between kills |
| `--csv` | `test_telemetry.csv` | Download progress over time |
| `--kills_csv` | `test_kills.csv` | Offline periods |

### `multiple_test_downlink.py`: repeated stress runs

Runs the resume test `--runs` times, each with a random kill interval between `--min_kill` and `--max_kill` seconds. Per-run CSVs and an `md5_summary_report.csv` are written to `downlink_stress_tests/`.

```bash
python multiple_test_downlink.py --runs 5 --min_kill 5 --max_kill 25
```

### `visualize_results.py`: plot a stress run

Plots bytes downloaded over time and shades each offline period.

```bash
python visualize_results.py --csv test_telemetry.csv --kills test_kills.csv --output test_results.png
```

### `check.py`: byte-by-byte comparison

Compares a downloaded image with its source, byte by byte, in a loop. The paths are hard-coded at the top of the file.

---

## Shared Python modules

| Module | Contents |
| :--- | :--- |
| [`Shared/Python/kiss_protocol.py`](../Shared/Python/kiss_protocol.py) | KISS `wrap_frame` / `unwrap_frame`, escaping, and the CRC-32/MPEG-2 that matches the STM32 hardware CRC |
| [`Shared/Python/beacon_helper.py`](../Shared/Python/beacon_helper.py) | `decode_beacon_packet()` and `print_decoded_beacon_data()` for the 121-byte beacon |
