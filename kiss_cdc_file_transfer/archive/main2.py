import serial
import time
import sys
import os
import csv
from datetime import datetime
from kiss_protocol import KISSProtocol as KISS

# ─────────────────────────────────────────────────────────────
#  Transfer metrics log
# ─────────────────────────────────────────────────────────────
TRIGGER_FRAME: bytes = bytes([
    0xC0, 0x00, 0x01, 0x02, 0x00, 0x03, 0x00, 0xFF, 0xFF,
    0x67, 0x3C, 0x6D, 0xB4, 0xC0,
])

METRICS_CSV = "test_log_automated.csv"
METRICS_FIELDNAMES = [
    "timestamp",
    "chunk_size",
    "total_bytes",
    "elapsed_s",
    "rate_kbps",
    "rate_KBps",
    "total_chunks",
    "file",
]

# ─────────────────────────────────────────────────────────────
#  Retry / timeout config
# ─────────────────────────────────────────────────────────────
INACTIVITY_TIMEOUT = 5.0    # seconds of silence before declaring a timeout
MAX_RETRIES        = 5       # max trigger retransmissions per transfer attempt
RETRY_DELAY        = 2.0     # seconds to wait before each retransmit


def append_metrics(total_bytes: int, elapsed: float, chunk_size: int, total_chunks: int):
    """Append one row of transfer metrics to transfer_metrics.csv."""
    if elapsed <= 0:
        elapsed = 0.001
    row = {
        "timestamp":    datetime.now().isoformat(timespec="seconds"),
        "chunk_size":   chunk_size,
        "total_bytes":  total_bytes,
        "elapsed_s":    round(elapsed, 4),
        "rate_kbps":    round((total_bytes * 8) / elapsed / 1_000, 2),
        "rate_KBps":    round(total_bytes / elapsed / 1_024, 3),
        "total_chunks": total_chunks,
        "file":         FILE_TO_SAVE,
    }
    file_exists = os.path.isfile(METRICS_CSV)
    with open(METRICS_CSV, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=METRICS_FIELDNAMES)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)
    Log.ok(f"Metrics saved → {C.BOLD}{METRICS_CSV}{C.RESET}  "
           f"(chunk_size={chunk_size}, {row['rate_kbps']} kbps)")


# ─────────────────────────────────────────────────────────────
#  ANSI color / style helpers
# ─────────────────────────────────────────────────────────────
class C:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"

    BLACK   = "\033[30m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN    = "\033[36m"
    WHITE   = "\033[37m"

    BG_RED    = "\033[41m"
    BG_GREEN  = "\033[42m"
    BG_YELLOW = "\033[43m"
    BG_BLUE   = "\033[44m"
    BG_CYAN   = "\033[46m"


# ─────────────────────────────────────────────────────────────
#  Structured logger
# ─────────────────────────────────────────────────────────────
class Log:
    _TAG_STYLES = {
        "INFO":    (C.CYAN,    C.BG_CYAN    + C.BLACK),
        "OK":      (C.GREEN,   C.BG_GREEN   + C.BLACK),
        "WARN":    (C.YELLOW,  C.BG_YELLOW  + C.BLACK),
        "ERROR":   (C.RED,     C.BG_RED     + C.WHITE),
        "SERIAL":  (C.BLUE,    C.BG_BLUE    + C.WHITE),
        "XFER":    (C.MAGENTA, C.MAGENTA),
        "CMD":     (C.WHITE,   C.WHITE      + C.BOLD),
        "ACK":     (C.GREEN,   C.BG_GREEN   + C.BLACK),
        "RETRY":   (C.YELLOW,  C.BG_YELLOW  + C.BLACK),
    }

    active_bar: "ProgressBar | None" = None

    @staticmethod
    def _ts() -> str:
        return C.DIM + time.strftime("%H:%M:%S") + C.RESET

    @classmethod
    def _print(cls, tag: str, msg: str):
        text_color, badge_style = cls._TAG_STYLES.get(tag, (C.WHITE, C.WHITE))
        badge     = f"{badge_style} {tag:<5} {C.RESET}"
        message   = f"{text_color}{msg}{C.RESET}"
        formatted = f"  {cls._ts()}  {badge}  {message}"

        bar = cls.active_bar
        if bar and bar._pinned:
            sys.stdout.write(
                bar._RESTORE
                + "\033[1L"
                + "\033[2K"
                + formatted + "\n"
            )
            sys.stdout.flush()
            sys.stdout.write(bar._SAVE)
            bar._render()
        else:
            print(formatted)

    @classmethod
    def info(cls, msg):   cls._print("INFO",  msg)
    @classmethod
    def ok(cls, msg):     cls._print("OK",    msg)
    @classmethod
    def warn(cls, msg):   cls._print("WARN",  msg)
    @classmethod
    def error(cls, msg):  cls._print("ERROR", msg)
    @classmethod
    def serial(cls, msg): cls._print("SERIAL", msg)
    @classmethod
    def xfer(cls, msg):   cls._print("XFER",  msg)
    @classmethod
    def cmd(cls, msg):    cls._print("CMD",   msg)
    @classmethod
    def ack(cls, msg):    cls._print("ACK",   msg)
    @classmethod
    def retry(cls, msg):  cls._print("RETRY", msg)


# ─────────────────────────────────────────────────────────────
#  Progress bar
# ─────────────────────────────────────────────────────────────
class ProgressBar:
    BAR_WIDTH = 38

    _SAVE    = "\033[s"
    _RESTORE = "\033[u"
    _CLEAR   = "\033[2K"
    _HIDE    = "\033[?25l"
    _SHOW    = "\033[?25h"

    def __init__(self, total_bytes: int, chunk_size: int):
        self.total        = total_bytes
        self.chunk_size   = chunk_size
        self.total_chunks = max(1, -(-total_bytes // chunk_size))
        self.current      = 0
        self.chunk_num    = 0
        self.start        = time.monotonic()
        self._pinned      = False

    def _pin(self):
        sys.stdout.write(self._HIDE)
        sys.stdout.write("\n")
        sys.stdout.write(self._SAVE)
        self._pinned = True
        self._render()

    def update(self, sent_so_far: int, chunk_num: int):
        self.current   = sent_so_far
        self.chunk_num = chunk_num
        if not self._pinned:
            self._pin()
        else:
            self._render()

    def _render(self):
        pct    = min(self.current / self.total, 1.0) if self.total else 1.0
        filled = int(self.BAR_WIDTH * pct)
        bar    = (C.GREEN + "█" * filled +
                  C.DIM   + "░" * (self.BAR_WIDTH - filled) +
                  C.RESET)

        elapsed  = time.monotonic() - self.start
        rate     = (self.current * 8 / elapsed / 1_000) if elapsed > 0 else 0.0
        eta      = ((self.total - self.current) * 8 / (rate * 1_000)) if rate > 0 else 0.0
        sent_kb  = self.current / 1_024
        total_kb = self.total   / 1_024
        chunk_info = f"{C.DIM}chunk {self.chunk_num+1}/{self.total_chunks}{C.RESET}"

        line = (
            f"  {C.MAGENTA}{C.BOLD}XFER{C.RESET} "
            f"[{bar}] "
            f"{C.BOLD}{pct*100:5.1f}%{C.RESET}  "
            f"{C.CYAN}{sent_kb:.1f}/{total_kb:.1f} KiB{C.RESET}  "
            f"{C.YELLOW}{rate:.0f} kbps{C.RESET}  "
            f"ETA {C.WHITE}{eta:.0f}s{C.RESET}  "
            f"{chunk_info}"
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
PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_IMAGE_REQUEST        = 0x02
VR_PID_IMAGE_DOWNLOAD       = 0x03
VR_PID_IMAGE_DOWNLOAD_DONE  = 0x05

PID_ACK                     = 0xAC

FILE_MODE_IDLE      = 0xDE
FILE_MODE_DUMP      = 0x00
FILE_MODE_WAIT_ACK  = 0xAB
FILE_MODE_CHUNK     = 0x01

FILE_MODE_LABELS = {
    FILE_MODE_IDLE:     "IDLE",
    FILE_MODE_DUMP:     "DUMP",
    FILE_MODE_WAIT_ACK: "WAIT_ACK",
    FILE_MODE_CHUNK:    "CHUNK",
}

# ─────────────────────────────────────────────────────────────
#  Config / globals
# ─────────────────────────────────────────────────────────────
current_dir = os.path.dirname(os.path.abspath(__file__))

try:
    with open("comport.txt", 'r') as f:
        SERIAL_PORT = f.read().strip()
except FileNotFoundError:
    SERIAL_PORT = '/dev/ttys005'

BAUD_RATE  = 115200
NUM_TEST   = 3
CHUNK_SIZES = [
    128, 256, 512, 1024, 2048, 4096, 8192,
    16384, 32768
]
IMAGES = [
    "testimg-1.jpg",
    "testimg-2.jpg",
    "testimg-3.jpg"
]
FILE_TO_SAVE = IMAGES[0]
CHUNK_SIZE   = CHUNK_SIZES[0]

ser: serial.Serial | None = None

transfer_start_time: float = 0.0
transfer_bytes_sent: int   = 0
_progress_bar: ProgressBar | None = None


# ─────────────────────────────────────────────────────────────
#  Serial helpers
# ─────────────────────────────────────────────────────────────
def _banner():
    print()
    print(C.BOLD + C.CYAN + "  ╔══════════════════════════════════════╗" + C.RESET)
    print(C.BOLD + C.CYAN + "  ║     VR Image Transfer Tool  v1.0     ║" + C.RESET)
    print(C.BOLD + C.CYAN + "  ╚══════════════════════════════════════╝" + C.RESET)
    print()
    Log.info(f"Port: {C.BOLD}{SERIAL_PORT}{C.RESET}  Baud: {C.BOLD}{BAUD_RATE}{C.RESET}  "
             f"File: {C.BOLD}{FILE_TO_SAVE}{C.RESET}  chunk_size: {C.BOLD}{CHUNK_SIZE}{C.RESET}")
    print()


def connect_serial():
    global ser
    while True:
        try:
            if ser and ser.is_open:
                ser.close()
            Log.serial(f"Connecting to {C.BOLD}{SERIAL_PORT}{C.RESET} @ {BAUD_RATE} baud …")
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
            Log.ok("Serial connection established.")
            return
        except (serial.SerialException, OSError) as e:
            Log.error(f"Connection failed: {e}  — retrying in 2 s …")
            time.sleep(2)


def send_data(data: bytes):
    global ser
    while True:
        try:
            ser.write(data)
            return
        except (serial.SerialException, OSError) as e:
            Log.error(f"Write failed: {e}  — reconnecting …")
            connect_serial()


def recv_whole_frame(timeout: float = INACTIVITY_TIMEOUT) -> bytes:
    """
    Read bytes until a complete KISS frame (two FEND bytes) is found.
    Raises TimeoutError if no complete frame arrives within `timeout` seconds
    of inactivity (no new bytes).
    """
    global ser
    recv_frame: bytes = b""
    last_activity     = time.monotonic()

    while True:
        try:
            if ser.in_waiting:
                recv_frame   += ser.read(ser.in_waiting)
                last_activity = time.monotonic()
                try:
                    start = recv_frame.index(KISS.FEND)
                    end   = recv_frame.index(KISS.FEND, start + 1)
                    return recv_frame
                except ValueError:
                    pass
            else:
                elapsed_idle = time.monotonic() - last_activity
                if elapsed_idle >= timeout:
                    raise TimeoutError(
                        f"No data received for {timeout:.1f}s "
                        f"(partial frame: {recv_frame.hex() if recv_frame else 'none'})"
                    )
                time.sleep(0.01)
        except (serial.SerialException, OSError) as e:
            Log.error(f"Read failed: {e}  — reconnecting …")
            connect_serial()
            recv_frame    = b""
            last_activity = time.monotonic()


# ─────────────────────────────────────────────────────────────
#  Trigger helper  (so retransmit can call it from anywhere)
# ─────────────────────────────────────────────────────────────
def send_trigger():
    """Open the trigger serial port, send TRIGGER_FRAME, then close."""
    TRIGGER_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10OMHTZ-if00-port0"
    try:
        req_ser = serial.Serial(TRIGGER_PORT, 9600, timeout=2)
        req_ser.write(TRIGGER_FRAME)
        req_ser.close()
        Log.retry(f"Trigger frame sent on {TRIGGER_PORT}")
    except (serial.SerialException, OSError) as e:
        Log.error(f"Could not send trigger: {e}")


# ─────────────────────────────────────────────────────────────
#  Transfer summary
# ─────────────────────────────────────────────────────────────
def print_transfer_summary(total_bytes: int, elapsed: float):
    if elapsed <= 0:
        elapsed = 0.001
    rate_kbps = (total_bytes * 8) / elapsed / 1_000
    rate_KBps = total_bytes / elapsed / 1_024

    print()
    print(C.BOLD + C.GREEN + "  ┌─────────────────────────────────────────┐" + C.RESET)
    print(C.BOLD + C.GREEN + "  │         FILE TRANSFER COMPLETE           │" + C.RESET)
    print(C.BOLD + C.GREEN + "  ├─────────────────────────────────────────┤" + C.RESET)
    print(f"  │  {C.CYAN}Total sent  {C.RESET}: {total_bytes:,} bytes ({total_bytes/1_024:.2f} KiB)")
    print(f"  │  {C.CYAN}Elapsed     {C.RESET}: {elapsed:.3f} s")
    print(f"  │  {C.CYAN}Data rate   {C.RESET}: {rate_kbps:.1f} kbps  ({rate_KBps:.2f} KiB/s)")
    print(C.BOLD + C.GREEN + "  └─────────────────────────────────────────┘" + C.RESET)
    print()


# ─────────────────────────────────────────────────────────────
#  Main state-machine loop  (with retransmission on timeout)
# ─────────────────────────────────────────────────────────────
def getCaptureRequest():
    """
    Runs the file-transfer state machine.  If a timeout or serial error
    occurs while waiting for a frame, the trigger is retransmitted and the
    state machine resumes from the last known chunk so no data is lost.
    After MAX_RETRIES consecutive failures the function raises RuntimeError.
    """
    global transfer_start_time, transfer_bytes_sent, _progress_bar

    file_mode        = FILE_MODE_IDLE
    current_chunk_id = 0
    current_file_id  = 0
    total_file_size  = 0
    retries          = 0          # consecutive timeout / error counter

    while True:

        # ── Active sending states ──────────────────────────────
        if file_mode == FILE_MODE_DUMP:
            with open(FILE_TO_SAVE, "rb") as image:
                data = image.read()

            total_file_size  = len(data)
            chunk_start      = current_chunk_id * CHUNK_SIZE
            chunk_end        = (current_chunk_id + 1) * CHUNK_SIZE
            transfer_content = data[chunk_start:chunk_end]

            if chunk_start <= total_file_size:
                if current_chunk_id == 0:
                    transfer_start_time = time.monotonic()
                    transfer_bytes_sent = 0
                    _progress_bar       = ProgressBar(total_file_size, CHUNK_SIZE)
                    Log.active_bar      = _progress_bar
                    Log.xfer(f"Starting transfer — {total_file_size:,} bytes  "
                             f"({total_file_size/1_024:.1f} KiB)  chunk_size={CHUNK_SIZE}")

                reply_frame = KISS.wrap_image_chunk(
                    current_file_id, current_chunk_id, transfer_content, VR_PID_IMAGE_DOWNLOAD
                )
                send_data(reply_frame)
                transfer_bytes_sent += len(transfer_content)

                if _progress_bar:
                    _progress_bar.update(transfer_bytes_sent, current_chunk_id)

                file_mode = FILE_MODE_WAIT_ACK
                retries   = 0   # successful send → reset retry counter

            else:
                # ── All chunks sent ───────────────────────────
                if _progress_bar:
                    _progress_bar.finish()
                    _progress_bar  = None
                    Log.active_bar = None

                Log.ok("All chunks sent — sending DOWNLOAD_DONE")
                reply_frame = KISS.wrap_frame(PAYLOAD_ID_VR, VR_PID_IMAGE_DOWNLOAD_DONE, b'', 0x00)
                send_data(reply_frame)

                elapsed = time.monotonic() - transfer_start_time
                print_transfer_summary(transfer_bytes_sent, elapsed)

                total_chunks_sent = -(-total_file_size // CHUNK_SIZE)
                append_metrics(transfer_bytes_sent, elapsed, CHUNK_SIZE, total_chunks_sent)

                file_mode        = FILE_MODE_IDLE
                current_chunk_id = 0
                current_file_id  = 0
                break

        # ── Single-chunk send ──────────────────────────────────
        elif file_mode == FILE_MODE_CHUNK:
            with open(FILE_TO_SAVE, "rb") as image:
                data = image.read()

            total_file_size  = len(data)
            chunk_start      = current_chunk_id * CHUNK_SIZE
            transfer_content = data[chunk_start: chunk_start + CHUNK_SIZE]

            reply_frame = KISS.wrap_image_chunk(
                current_file_id, current_chunk_id, transfer_content, VR_PID_IMAGE_DOWNLOAD
            )
            send_data(reply_frame)
            Log.xfer(f"Single-chunk #{current_chunk_id} sent")
            file_mode = FILE_MODE_WAIT_ACK
            retries   = 0

        # ── Receive & dispatch ─────────────────────────────────
        try:
            buf = recv_whole_frame(timeout=INACTIVITY_TIMEOUT)
            retries = 0   # got a frame → reset retry counter

        except TimeoutError as e:
            retries += 1
            mode_label = FILE_MODE_LABELS.get(file_mode, f"0x{file_mode:02X}")
            Log.warn(f"Timeout in mode={mode_label}: {e}")

            if retries > MAX_RETRIES:
                # Abort — caller (run()) will handle re-init if desired
                raise RuntimeError(
                    f"Transfer failed after {MAX_RETRIES} retries "
                    f"(chunk={current_chunk_id}, mode={mode_label})"
                )

            Log.retry(
                f"Attempt {retries}/{MAX_RETRIES} — waiting {RETRY_DELAY}s "
                f"then retransmitting trigger …"
            )
            time.sleep(RETRY_DELAY)

            # If we were mid-transfer, rewind so the chunk is re-sent
            if file_mode == FILE_MODE_WAIT_ACK:
                Log.retry(f"Rewinding to re-send chunk #{current_chunk_id}")
                file_mode = FILE_MODE_DUMP

            send_trigger()
            continue   # back to top of loop

        except Exception as e:
            retries += 1
            Log.error(f"Unexpected error receiving frame: {e}")
            if retries > MAX_RETRIES:
                raise RuntimeError(f"Too many errors: {e}")
            Log.retry(f"Attempt {retries}/{MAX_RETRIES} — retransmitting trigger …")
            time.sleep(RETRY_DELAY)
            if file_mode == FILE_MODE_WAIT_ACK:
                file_mode = FILE_MODE_DUMP
            send_trigger()
            continue

        frame = KISS.unwrap_frame(buf)

        if frame['type'] != 'generic':
            Log.warn(f"Received non-generic frame — ignoring (type={frame['type']})")
            continue

        if frame['payload_id'] != PAYLOAD_ID_VR:
            Log.warn(f"Unknown payload_id=0x{frame['payload_id']:02X} — ignoring")
            continue

        pid = frame['pid']

        # ── STATUS POLL ───────────────────────────────────────
        if pid == VR_PID_GET_STATUS:
            mode_label = FILE_MODE_LABELS.get(file_mode, f"0x{file_mode:02X}")
            Log.info(f"Status poll received  [mode={C.BOLD}{mode_label}{C.RESET}]")

        # ── ACK ───────────────────────────────────────────────
        elif pid == PID_ACK:
            if file_mode == FILE_MODE_WAIT_ACK:
                Log.ack(f"Chunk #{current_chunk_id} acknowledged — advancing")
                current_chunk_id += 1
                file_mode = FILE_MODE_DUMP
            else:
                Log.ack(f"Unexpected ACK in mode {FILE_MODE_LABELS.get(file_mode,'?')} — ignored")

        # ── CAPTURE COMMAND ───────────────────────────────────
        elif pid == VR_PID_GET_IMAGE_CAPTURE:
            Log.cmd("CAPTURE command received")
            reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
            send_data(reply)
            Log.ok("ACK sent for CAPTURE")

        # ── IMAGE REQUEST ─────────────────────────────────────
        elif pid == VR_PID_IMAGE_REQUEST:
            file_path = f"{current_dir}/{FILE_TO_SAVE}"
            file_size = os.path.getsize(file_path)
            Log.cmd(f"IMAGE REQUEST received  [{file_path}  {file_size:,} B]")

            reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
            send_data(reply)
            Log.ok("ACK sent for IMAGE REQUEST")

            if frame['data_len'] == 3:
                _file_id        = frame['data'][0]
                _chunk_id       = frame['data'][1:3]
                current_file_id = _file_id
                requested_chunk = int.from_bytes(_chunk_id, "big")

                if requested_chunk == 0xFFFF:
                    Log.xfer(f"Full-file transfer requested (file_id={_file_id})")
                    current_chunk_id = 0
                    file_mode        = FILE_MODE_DUMP
                else:
                    Log.xfer(f"Single-chunk request: chunk #{requested_chunk}  file_id={_file_id}")
                    current_chunk_id = requested_chunk
                    file_mode        = FILE_MODE_CHUNK
            else:
                Log.warn(f"IMAGE REQUEST had unexpected data_len={frame['data_len']} — ignoring")

        # ── UNKNOWN PID ───────────────────────────────────────
        else:
            Log.warn(f"Unknown PID 0x{pid:02X} received — ignoring")


# ─────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────
def run():
    global FILE_TO_SAVE, CHUNK_SIZE

    for i in range(len(IMAGES)):
        FILE_TO_SAVE = IMAGES[i]
        for chunk_index in range(len(CHUNK_SIZES)):
            CHUNK_SIZE = CHUNK_SIZES[chunk_index]
            for test_num in range(NUM_TEST):
                print("\033c", end="")
                _banner()
                connect_serial()
                time.sleep(3)
                send_trigger()

                # Outer retry loop: if getCaptureRequest() exhausts MAX_RETRIES,
                # we re-establish serial and fire the trigger again before giving up.
                attempt = 0
                while attempt < MAX_RETRIES:
                    try:
                        getCaptureRequest()
                        break   # success — move on to next test
                    except RuntimeError as e:
                        attempt += 1
                        Log.error(f"getCaptureRequest failed ({e})")
                        if attempt >= MAX_RETRIES:
                            Log.error(
                                f"Giving up on file={FILE_TO_SAVE} "
                                f"chunk_size={CHUNK_SIZE} test={test_num+1} "
                                f"after {MAX_RETRIES} outer retries."
                            )
                            break
                        Log.retry(
                            f"Outer retry {attempt}/{MAX_RETRIES} — "
                            f"reconnecting serial and re-sending trigger …"
                        )
                        connect_serial()
                        time.sleep(RETRY_DELAY)
                        send_trigger()

                print("Transfer attempt complete.")
                time.sleep(1)


if __name__ == "__main__":
    run()