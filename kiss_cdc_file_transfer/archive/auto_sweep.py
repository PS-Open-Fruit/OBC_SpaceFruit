"""
auto_sweep.py  —  Automated chunk-size sweep tester
=====================================================

For each (file, chunk_size) combination the script:
  1. Sends the raw trigger frame on the *trigger* serial port
  2. Waits for the DUT to send VR_PID_IMAGE_REQUEST on the *data* port
  3. ACKs the request, then drives the chunk loop reactively
  4. Appends one row to sweep_metrics.csv when the transfer completes

Ports
-----
  Data port    – comport.txt        (same port used by the normal tool)
  Trigger port – triggerport.txt    (new; the port that receives the command)

If either .txt file is missing a sensible default is used (see CONFIG section).

Trigger frame (sent verbatim for every new transfer):
  C0 00 01 02 00 03 00 FF FF 67 3C 6D B4 C0
"""

import serial
import time
import sys
import os
import csv
from datetime import datetime
from kiss_protocol import KISSProtocol as KISS

# ─────────────────────────────────────────────────────────────
#  ███  CONFIG — edit these as needed  ███
# ─────────────────────────────────────────────────────────────

# Files to test (must exist on disk relative to this script)
TEST_FILES: list[str] = [
    "testimg-1.jpg",
    "testimg-2.jpg",
    "testimg-3.jpg",
]

# Chunk sizes to sweep (bytes)
CHUNK_SIZES: list[int] = [
    32, 64, 128, 256, 512,
    1_024, 2_048, 4_096, 8_192, 16_384, 32_768,
]

# How long (s) to wait for the remote side to ACK after sending the trigger
ACK_TIMEOUT_S: float = 15.0

# How long (s) to wait for any individual chunk ACK during transfer
CHUNK_ACK_TIMEOUT_S: float = 30.0

# Output CSV
METRICS_CSV = "sweep_metrics.csv"
METRICS_FIELDNAMES = [
    "timestamp", "file", "file_bytes",
    "chunk_size", "total_chunks",
    "elapsed_s", "rate_kbps", "rate_KBps",
]

# Trigger frame bytes (as shown: C0 00 01 02 00 03 00 FF FF 67 3C 6D B4 C0)
TRIGGER_FRAME: bytes = bytes([
    0xC0, 0x00, 0x01, 0x02, 0x00, 0x03, 0x00, 0xFF, 0xFF,
    0x67, 0x3C, 0x6D, 0xB4, 0xC0,
])

TRIGGER_BAUD_RATE = 9600
BAUD_RATE = 115_200

# ─────────────────────────────────────────────────────────────
#  Port discovery
# ─────────────────────────────────────────────────────────────
def _read_port_file(filename: str, default: str) -> str:
    try:
        with open(filename) as f:
            return f.read().strip()
    except FileNotFoundError:
        return default

DATA_PORT    = _read_port_file("comport.txt",     "/dev/ttys005")
TRIGGER_PORT = _read_port_file("triggerport.txt", "/dev/ttys006")

# ─────────────────────────────────────────────────────────────
#  ANSI colour helpers
# ─────────────────────────────────────────────────────────────
class C:
    RESET   = "\033[0m";  BOLD  = "\033[1m";  DIM  = "\033[2m"
    RED     = "\033[31m"; GREEN = "\033[32m";  YELLOW = "\033[33m"
    BLUE    = "\033[34m"; MAGENTA = "\033[35m";CYAN  = "\033[36m"
    WHITE   = "\033[37m"
    BG_RED  = "\033[41m"; BG_GREEN = "\033[42m"; BG_YELLOW = "\033[43m"
    BG_BLUE = "\033[44m"; BG_CYAN  = "\033[46m"

# ─────────────────────────────────────────────────────────────
#  Structured logger
# ─────────────────────────────────────────────────────────────
class Log:
    _TAG_STYLES = {
        "INFO":  (C.CYAN,    C.BG_CYAN    + C.BLACK  if hasattr(C,"BLACK") else C.BG_CYAN),
        "OK":    (C.GREEN,   C.BG_GREEN   + "\033[30m"),
        "WARN":  (C.YELLOW,  C.BG_YELLOW  + "\033[30m"),
        "ERROR": (C.RED,     C.BG_RED     + C.WHITE),
        "SERIAL":(C.BLUE,    C.BG_BLUE    + C.WHITE),
        "XFER":  (C.MAGENTA, C.MAGENTA),
        "CMD":   (C.WHITE,   C.WHITE      + C.BOLD),
        "ACK":   (C.GREEN,   C.BG_GREEN   + "\033[30m"),
        "TEST":  (C.YELLOW,  C.BG_YELLOW  + "\033[30m"),
        "SWEEP": (C.MAGENTA, C.MAGENTA    + C.BOLD),
    }
    active_bar: "ProgressBar | None" = None

    @staticmethod
    def _ts() -> str:
        return C.DIM + time.strftime("%H:%M:%S") + C.RESET

    @classmethod
    def _print(cls, tag: str, msg: str):
        tc, bs = cls._TAG_STYLES.get(tag, (C.WHITE, C.WHITE))
        badge     = f"{bs} {tag:<5} {C.RESET}"
        formatted = f"  {cls._ts()}  {badge}  {tc}{msg}{C.RESET}"
        bar = cls.active_bar
        if bar and bar._pinned:
            sys.stdout.write(bar._RESTORE + "\033[1L" + "\033[2K" + formatted + "\n")
            sys.stdout.flush()
            sys.stdout.write(bar._SAVE)
            bar._render()
        else:
            print(formatted)

    @classmethod
    def info(cls, m):  cls._print("INFO",  m)
    @classmethod
    def ok(cls, m):    cls._print("OK",    m)
    @classmethod
    def warn(cls, m):  cls._print("WARN",  m)
    @classmethod
    def error(cls, m): cls._print("ERROR", m)
    @classmethod
    def serial(cls,m): cls._print("SERIAL",m)
    @classmethod
    def xfer(cls, m):  cls._print("XFER",  m)
    @classmethod
    def cmd(cls, m):   cls._print("CMD",   m)
    @classmethod
    def ack(cls, m):   cls._print("ACK",   m)
    @classmethod
    def test(cls, m):  cls._print("TEST",  m)
    @classmethod
    def sweep(cls,m):  cls._print("SWEEP", m)

# ─────────────────────────────────────────────────────────────
#  Progress bar
# ─────────────────────────────────────────────────────────────
class ProgressBar:
    BAR_WIDTH = 38
    _SAVE    = "\033[s"; _RESTORE = "\033[u"
    _CLEAR   = "\033[2K"; _HIDE = "\033[?25l"; _SHOW = "\033[?25h"

    def __init__(self, total_bytes: int, chunk_size: int):
        self.total        = total_bytes
        self.chunk_size   = chunk_size
        self.total_chunks = max(1, -(-total_bytes // chunk_size))
        self.current      = 0
        self.chunk_num    = 0
        self.start        = time.monotonic()
        self._pinned      = False

    def _pin(self):
        sys.stdout.write(self._HIDE + "\n" + self._SAVE)
        self._pinned = True
        self._render()

    def update(self, sent: int, chunk_num: int):
        self.current   = sent
        self.chunk_num = chunk_num
        if not self._pinned: self._pin()
        else:                self._render()

    def _render(self):
        pct    = min(self.current / self.total, 1.0) if self.total else 1.0
        filled = int(self.BAR_WIDTH * pct)
        bar    = C.GREEN + "█" * filled + C.DIM + "░" * (self.BAR_WIDTH - filled) + C.RESET
        elapsed = time.monotonic() - self.start
        rate    = (self.current * 8 / elapsed / 1_000) if elapsed > 0 else 0.0
        eta     = ((self.total - self.current) * 8 / (rate * 1_000)) if rate > 0 else 0.0
        line = (
            f"  {C.MAGENTA}{C.BOLD}XFER{C.RESET} [{bar}] "
            f"{C.BOLD}{pct*100:5.1f}%{C.RESET}  "
            f"{C.CYAN}{self.current/1_024:.1f}/{self.total/1_024:.1f} KiB{C.RESET}  "
            f"{C.YELLOW}{rate:.0f} kbps{C.RESET}  ETA {C.WHITE}{eta:.0f}s{C.RESET}  "
            f"{C.DIM}chunk {self.chunk_num}/{self.total_chunks}{C.RESET}"
        )
        sys.stdout.write(self._RESTORE + self._CLEAR + line)
        sys.stdout.flush()

    def finish(self):
        self.update(self.total, self.chunk_num)
        sys.stdout.write("\n" + self._SHOW)
        sys.stdout.flush()

# ─────────────────────────────────────────────────────────────
#  Protocol constants
# ─────────────────────────────────────────────────────────────
PAYLOAD_ID_VR              = 0x01
VR_PID_GET_STATUS          = 0x00
VR_PID_GET_IMAGE_CAPTURE   = 0x01
VR_PID_IMAGE_REQUEST       = 0x02
VR_PID_IMAGE_DOWNLOAD      = 0x03
VR_PID_IMAGE_DOWNLOAD_DONE = 0x05
PID_ACK                    = 0xAC

FILE_MODE_IDLE     = 0xDE
FILE_MODE_DUMP     = 0x00
FILE_MODE_WAIT_ACK = 0xAB
FILE_MODE_CHUNK    = 0x01
FILE_MODE_LABELS   = {
    FILE_MODE_IDLE: "IDLE", FILE_MODE_DUMP: "DUMP",
    FILE_MODE_WAIT_ACK: "WAIT_ACK", FILE_MODE_CHUNK: "CHUNK",
}

# ─────────────────────────────────────────────────────────────
#  Serial port handles  (opened once, reused across tests)
# ─────────────────────────────────────────────────────────────
data_ser:    serial.Serial | None = None
trigger_ser: serial.Serial | None = None

def open_ports():
    global data_ser, trigger_ser
    Log.serial(f"Data port    → {C.BOLD}{DATA_PORT}{C.RESET} @ {BAUD_RATE}")
    data_ser = serial.Serial(DATA_PORT, BAUD_RATE, timeout=CHUNK_ACK_TIMEOUT_S)
    Log.serial(f"Trigger port → {C.BOLD}{TRIGGER_PORT}{C.RESET} @ {TRIGGER_BAUD_RATE}")
    trigger_ser = serial.Serial(TRIGGER_PORT, TRIGGER_BAUD_RATE, timeout=ACK_TIMEOUT_S)
    Log.ok("Both serial ports open.")

def close_ports():
    for s in (data_ser, trigger_ser):
        try:
            if s and s.is_open:
                s.close()
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────
#  Low-level I/O (data port)
# ─────────────────────────────────────────────────────────────
def send_data(frame: bytes):
    data_ser.write(frame)

def recv_whole_frame(timeout_s: float = CHUNK_ACK_TIMEOUT_S) -> bytes | None:
    """Read from data_ser until a complete KISS frame arrives or timeout."""
    buf      = b""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        waiting = data_ser.in_waiting
        if waiting:
            buf += data_ser.read(waiting)
            try:
                start = buf.index(KISS.FEND)
                buf.index(KISS.FEND, start + 1)   # raises ValueError if no second FEND
                return buf
            except ValueError:
                pass
        else:
            time.sleep(0.005)
    return None  # timed out

# ─────────────────────────────────────────────────────────────
#  Trigger helper
# ─────────────────────────────────────────────────────────────
def send_trigger():
    """Send the raw IMAGE REQUEST trigger frame on the trigger serial port."""
    Log.cmd(
        f"Trigger → {C.BOLD}{TRIGGER_PORT}{C.RESET}  "
        f"[{' '.join(f'{b:02X}' for b in TRIGGER_FRAME)}]"
    )
    trigger_ser.write(TRIGGER_FRAME)
    trigger_ser.flush()


# ─────────────────────────────────────────────────────────────
#  Single transfer  (DUT-driven state machine)
# ─────────────────────────────────────────────────────────────
def run_single_transfer(file_path: str, chunk_size: int) -> dict:
    """
    Mirror getCaptureRequest() exactly:

      IDLE
        └─ recv VR_PID_GET_IMAGE_CAPTURE  → ACK it, stay IDLE
        └─ recv VR_PID_IMAGE_REQUEST      → ACK it, parse file_id/chunk_id
                                            0xFFFF  → DUMP from chunk 0
                                            other   → CHUNK (single chunk)
      DUMP  (active-send branch, entered on every loop iteration)
        ├─ chunk_start < file_size  → send chunk, WAIT_ACK
        └─ chunk_start >= file_size → send DOWNLOAD_DONE, return metrics
      WAIT_ACK
        └─ recv PID_ACK             → advance chunk_id, DUMP

    Timer starts on chunk 0 send, stops on DOWNLOAD_DONE send.
    Raises TimeoutError if any recv call exceeds its deadline.
    """
    with open(file_path, "rb") as fh:
        file_data = fh.read()
    total_bytes  = len(file_data)
    total_chunks = -(-total_bytes // chunk_size)   # ceiling division

    file_mode        = FILE_MODE_IDLE
    current_chunk_id = 0
    current_file_id  = 0

    progress:  ProgressBar | None = None
    bytes_sent = 0
    start_time = 0.0

    Log.info(
        f"Waiting for DUT IMAGE_REQUEST  "
        f"[file={os.path.basename(file_path)}  chunk_size={chunk_size}]"
    )

    while True:

        # ── DUMP: active-send branch (no recv needed first) ───
        if file_mode == FILE_MODE_DUMP:
            chunk_start = current_chunk_id * chunk_size
            chunk_end   = (current_chunk_id + 1) * chunk_size
            payload     = file_data[chunk_start:chunk_end]

            if chunk_start < total_bytes:
                # Start timer + progress bar on very first chunk
                if current_chunk_id == 0:
                    start_time = time.monotonic()
                    bytes_sent = 0
                    progress   = ProgressBar(total_bytes, chunk_size)
                    Log.active_bar = progress
                    Log.xfer(
                        f"Transfer start — {total_bytes:,} B  "
                        f"chunk_size={chunk_size}  chunks={total_chunks}"
                    )

                Log.xfer(f"Sending chunk {current_chunk_id}/{total_chunks - 1}")
                reply_frame = KISS.wrap_image_chunk(
                    current_file_id, current_chunk_id, payload, VR_PID_IMAGE_DOWNLOAD
                )
                send_data(reply_frame)
                bytes_sent += len(payload)

                if progress:
                    progress.update(bytes_sent, current_chunk_id)

                file_mode = FILE_MODE_WAIT_ACK
                Log.info(f"Waiting for chunk {current_chunk_id} ACK …")

            else:
                # All chunks sent → DOWNLOAD_DONE
                if progress:
                    progress.finish()
                    Log.active_bar = None
                    progress = None

                elapsed   = time.monotonic() - start_time
                rate_kbps = (bytes_sent * 8) / elapsed / 1_000 if elapsed > 0 else 0.0
                rate_KBps = bytes_sent / elapsed / 1_024        if elapsed > 0 else 0.0

                done_frame = KISS.wrap_frame(
                    PAYLOAD_ID_VR, VR_PID_IMAGE_DOWNLOAD_DONE, b'', 0x00
                )
                send_data(done_frame)
                Log.ok(
                    f"DOWNLOAD_DONE sent — {bytes_sent:,} B in {elapsed:.3f}s  "
                    f"({rate_kbps:.1f} kbps)"
                )
                return {
                    "timestamp":    datetime.now().isoformat(timespec="seconds"),
                    "file":         os.path.basename(file_path),
                    "file_bytes":   total_bytes,
                    "chunk_size":   chunk_size,
                    "total_chunks": total_chunks,
                    "elapsed_s":    round(elapsed, 4),
                    "rate_kbps":    round(rate_kbps, 2),
                    "rate_KBps":    round(rate_KBps, 3),
                }

        # ── Receive next frame ────────────────────────────────
        # Use a tighter timeout once the transfer has started
        timeout = CHUNK_ACK_TIMEOUT_S if file_mode == FILE_MODE_WAIT_ACK else ACK_TIMEOUT_S
        raw = recv_whole_frame(timeout_s=timeout)
        if raw is None:
            if progress:
                progress.finish()
                Log.active_bar = None
            state_name = FILE_MODE_LABELS.get(file_mode, f"0x{file_mode:02X}")
            raise TimeoutError(
                f"Receive timeout in state {state_name} "
                f"(chunk={current_chunk_id}, waited {timeout}s)"
            )

        frame = KISS.unwrap_frame(raw)

        if frame.get("type") != "generic":
            Log.warn(f"Non-generic frame — ignoring (type={frame.get('type')})")
            continue
        if frame.get("payload_id") != PAYLOAD_ID_VR:
            Log.warn(f"Unknown payload_id=0x{frame.get('payload_id', 0):02X} — ignoring")
            continue

        pid = frame.get("pid")

        # ── STATUS POLL — harmless, ignore quietly ────────────
        if pid == VR_PID_GET_STATUS:
            Log.info(f"Status poll received [state={FILE_MODE_LABELS.get(file_mode,'?')}]")

        # ── CAPTURE COMMAND — ACK and stay IDLE ──────────────
        elif pid == VR_PID_GET_IMAGE_CAPTURE:
            Log.cmd("CAPTURE command received")
            reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
            send_data(reply)
            Log.ok("ACK sent for CAPTURE")

        # ── IMAGE REQUEST — ACK, parse params, start transfer ─
        elif pid == VR_PID_IMAGE_REQUEST:
            file_size = total_bytes
            Log.cmd(
                f"IMAGE_REQUEST received  "
                f"[{os.path.basename(file_path)}  {file_size:,} B]"
            )

            # Always ACK the request first, exactly like getCaptureRequest
            reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
            send_data(reply)
            Log.ok("ACK sent for IMAGE_REQUEST")

            if frame.get("data_len") == 3:
                _file_id        = frame["data"][0]
                _chunk_id_bytes = frame["data"][1:3]
                current_file_id  = _file_id
                requested_chunk  = int.from_bytes(_chunk_id_bytes, "big")

                if requested_chunk == 0xFFFF:
                    Log.xfer(f"Full-file transfer requested (file_id={_file_id})")
                    current_chunk_id = 0
                    file_mode        = FILE_MODE_DUMP
                else:
                    Log.xfer(
                        f"Single-chunk request: chunk #{requested_chunk}  "
                        f"file_id={_file_id}"
                    )
                    current_chunk_id = requested_chunk
                    file_mode        = FILE_MODE_CHUNK
            else:
                Log.warn(
                    f"IMAGE_REQUEST unexpected data_len="
                    f"{frame.get('data_len')} — ignoring"
                )

        # ── CHUNK ACK — advance to next chunk ────────────────
        elif pid == PID_ACK:
            if file_mode == FILE_MODE_WAIT_ACK:
                Log.ack(f"Chunk #{current_chunk_id} ACK'd — advancing")
                current_chunk_id += 1
                file_mode = FILE_MODE_DUMP
            else:
                Log.warn(
                    f"Unexpected ACK in state "
                    f"{FILE_MODE_LABELS.get(file_mode, '?')} — ignoring"
                )

        # ── UNKNOWN PID ───────────────────────────────────────
        else:
            Log.warn(
                f"Unknown PID=0x{pid:02X} in state "
                f"{FILE_MODE_LABELS.get(file_mode, '?')} — ignoring"
            )

# ─────────────────────────────────────────────────────────────
#  CSV helper
# ─────────────────────────────────────────────────────────────
def append_metrics(row: dict):
    exists = os.path.isfile(METRICS_CSV)
    with open(METRICS_CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=METRICS_FIELDNAMES)
        if not exists:
            w.writeheader()
        w.writerow(row)
    Log.ok(
        f"CSV ← {row['file']}  chunk={row['chunk_size']}  "
        f"{row['rate_kbps']} kbps  {row['elapsed_s']}s"
    )

# ─────────────────────────────────────────────────────────────
#  Sweep summary banner
# ─────────────────────────────────────────────────────────────
def _sweep_banner(total_tests: int):
    n_files  = len(TEST_FILES)
    n_chunks = len(CHUNK_SIZES)
    print()
    print(C.BOLD + C.MAGENTA + "  ╔══════════════════════════════════════════════╗" + C.RESET)
    print(C.BOLD + C.MAGENTA + "  ║        VR Chunk-Size Automated Sweep         ║" + C.RESET)
    print(C.BOLD + C.MAGENTA + "  ╚══════════════════════════════════════════════╝" + C.RESET)
    print()
    Log.info(f"Data port    : {C.BOLD}{DATA_PORT}{C.RESET}")
    Log.info(f"Trigger port : {C.BOLD}{TRIGGER_PORT}{C.RESET}")
    Log.info(f"Files        : {n_files}  → {', '.join(TEST_FILES)}")
    Log.info(f"Chunk sizes  : {n_chunks}  → {CHUNK_SIZES}")
    Log.info(f"Total tests  : {C.BOLD}{total_tests}{C.RESET}  (files × chunk sizes)")
    Log.info(f"Output CSV   : {C.BOLD}{METRICS_CSV}{C.RESET}")
    print()

def _final_summary(results: list[dict], failures: list[dict]):
    passed = len(results)
    failed = len(failures)
    total  = passed + failed
    print()
    print(C.BOLD + C.GREEN + "  ╔══════════════════════════════════════════════╗" + C.RESET)
    print(C.BOLD + C.GREEN + "  ║             SWEEP COMPLETE                   ║" + C.RESET)
    print(C.BOLD + C.GREEN + "  ╠══════════════════════════════════════════════╣" + C.RESET)
    print(f"  ║  {C.CYAN}Tests run  {C.RESET}: {total:<35}  ║")
    print(f"  ║  {C.GREEN}Passed     {C.RESET}: {passed:<35}  ║")
    if failed:
        print(f"  ║  {C.RED}Failed     {C.RESET}: {failed:<35}  ║")
        for f in failures:
            print(f"  ║    {C.RED}✗ {f['file']} chunk={f['chunk_size']} — {f['reason']}{C.RESET}")
    print(f"  ║  {C.CYAN}Results    {C.RESET}: {METRICS_CSV:<35}  ║")
    print(C.BOLD + C.GREEN + "  ╚══════════════════════════════════════════════╝" + C.RESET)
    print()

# ─────────────────────────────────────────────────────────────
#  Main sweep loop
# ─────────────────────────────────────────────────────────────
def run_sweep():
    total_tests = len(TEST_FILES) * len(CHUNK_SIZES)
    _sweep_banner(total_tests)

    open_ports()
    time.sleep(1)
    results:  list[dict] = []
    failures: list[dict] = []
    test_num = 0

    try:
        for file_name in TEST_FILES:
            file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)

            if not os.path.isfile(file_path):
                Log.error(f"File not found — skipping: {file_path}")
                for cs in CHUNK_SIZES:
                    failures.append({"file": file_name, "chunk_size": cs, "reason": "file not found"})
                continue

            for chunk_size in CHUNK_SIZES:
                test_num += 1
                Log.sweep(
                    f"[{test_num}/{total_tests}]  "
                    f"file={C.BOLD}{file_name}{C.RESET}  "
                    f"chunk_size={C.BOLD}{chunk_size}{C.RESET}"
                )

                try:
                    # 1. Fire trigger on the trigger port — DUT will respond
                    #    with VR_PID_IMAGE_REQUEST on the data port
                    send_trigger()

                    # Small settle gap so the DUT has time to react
                    time.sleep(0.1)

                    # 2. Run the DUT-driven transfer state machine.
                    #    Blocks until DOWNLOAD_DONE is sent.
                    metrics = run_single_transfer(file_path, chunk_size)

                    # 3. Save result
                    append_metrics(metrics)
                    results.append(metrics)

                    # Brief pause between tests so device can reset
                    time.sleep(0.5)

                except (TimeoutError, ValueError, serial.SerialException, OSError) as exc:
                    Log.error(f"Test failed: {exc}")
                    failures.append({
                        "file":       file_name,
                        "chunk_size": chunk_size,
                        "reason":     str(exc),
                    })
                    # Flush serial buffers before next test
                    try:
                        data_ser.reset_input_buffer()
                        data_ser.reset_output_buffer()
                    except Exception:
                        pass
                    time.sleep(1.0)

    finally:
        close_ports()

    _final_summary(results, failures)


# ─────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\033c", end="")
    run_sweep()