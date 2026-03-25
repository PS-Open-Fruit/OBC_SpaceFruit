import serial
import time
import sys
import os
import subprocess
from kiss_protocol import KISSProtocol as KISS
from config import CHUNK_SIZE


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
    """Colour-coded, tagged logger. When a transfer is active, prints above the pinned bar."""

    _TAG_STYLES = {
        "INFO":     (C.CYAN,    C.BG_CYAN    + C.BLACK),
        "OK":       (C.GREEN,   C.BG_GREEN   + C.BLACK),
        "WARN":     (C.YELLOW,  C.BG_YELLOW  + C.BLACK),
        "ERROR":    (C.RED,     C.BG_RED     + C.WHITE),
        "SERIAL":   (C.BLUE,    C.BG_BLUE    + C.WHITE),
        "XFER":     (C.MAGENTA, C.MAGENTA),
        "CMD":      (C.WHITE,   C.WHITE     + C.BOLD),
        "ACK":      (C.GREEN,   C.BG_GREEN   + C.BLACK),
    }

    # Set by the main loop when a ProgressBar is active
    active_bar: "ProgressBar | None" = None

    @staticmethod
    def _ts() -> str:
        return C.DIM + time.strftime("%H:%M:%S") + C.RESET

    @classmethod
    def _print(cls, tag: str, msg: str):
        text_color, badge_style = cls._TAG_STYLES.get(tag, (C.WHITE, C.WHITE))
        badge   = f"{badge_style} {tag:<5} {C.RESET}"
        message = f"{text_color}{msg}{C.RESET}"
        formatted = f"  {cls._ts()}  {badge}  {message}"

        bar = cls.active_bar
        if bar and bar._pinned:
            # Move to saved position (bar line), scroll it down by inserting a line above,
            # print the log message there, then redraw the bar on the next line.
            sys.stdout.write(
                bar._RESTORE        # jump to bar's home position
                + "\033[1L"         # insert 1 blank line (pushes bar down)
                + "\033[2K"         # clear the line we're now on
                + formatted + "\n"  # print log line
            )
            sys.stdout.flush()
            # Re-save cursor at the bar's new home (one line lower) and redraw
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
    def serial(cls, msg): cls._print("SERIAL",msg)
    @classmethod
    def xfer(cls, msg):   cls._print("XFER",  msg)
    @classmethod
    def cmd(cls, msg):    cls._print("CMD",   msg)
    @classmethod
    def ack(cls, msg):    cls._print("ACK",   msg)


# ─────────────────────────────────────────────────────────────
#  Progress bar  (stays pinned to ONE terminal line)
# ─────────────────────────────────────────────────────────────
class ProgressBar:
    BAR_WIDTH = 38

    # ANSI escape sequences for cursor control
    _SAVE    = "\033[s"          # save cursor position
    _RESTORE = "\033[u"          # restore cursor to saved position
    _CLEAR   = "\033[2K"         # erase the entire current line
    _HIDE    = "\033[?25l"       # hide cursor while animating
    _SHOW    = "\033[?25h"       # show cursor again

    def __init__(self, total_bytes: int, chunk_size: int):
        self.total      = total_bytes
        self.chunk_size = chunk_size
        self.total_chunks = max(1, -(-total_bytes // chunk_size))  # ceiling division
        self.current    = 0
        self.chunk_num  = 0
        self.start      = time.monotonic()
        self._pinned    = False   # have we printed the bar's home line yet?

    def _pin(self):
        """Print the bar for the first time and save the cursor position."""
        sys.stdout.write(self._HIDE)
        sys.stdout.write("\n")          # blank line so the bar has breathing room
        sys.stdout.write(self._SAVE)    # remember exactly where we are
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
        pct     = min(self.current / self.total, 1.0) if self.total else 1.0
        filled  = int(self.BAR_WIDTH * pct)
        bar     = (C.GREEN + "█" * filled +
                   C.DIM   + "░" * (self.BAR_WIDTH - filled) +
                   C.RESET)

        elapsed = time.monotonic() - self.start
        rate    = (self.current * 8 / elapsed / 1_000) if elapsed > 0 else 0.0  # kbps
        eta     = ((self.total - self.current) * 8 / (rate * 1_000)) if rate > 0 else 0.0

        sent_kb  = self.current / 1_024
        total_kb = self.total   / 1_024

        chunk_info = (f"{C.DIM}chunk {self.chunk_num}/{self.total_chunks}{C.RESET}")

        line = (
            f"  {C.MAGENTA}{C.BOLD}XFER{C.RESET} "
            f"[{bar}] "
            f"{C.BOLD}{pct*100:5.1f}%{C.RESET}  "
            f"{C.CYAN}{sent_kb:.1f}/{total_kb:.1f} KiB{C.RESET}  "
            f"{C.YELLOW}{rate:.0f} kbps{C.RESET}  "
            f"ETA {C.WHITE}{eta:.0f}s{C.RESET}  "
            f"{chunk_info}"
        )

        # Jump back to the saved position, wipe the line, redraw
        sys.stdout.write(self._RESTORE + self._CLEAR + line)
        sys.stdout.flush()

    def finish(self):
        self.update(self.total, self.chunk_num)
        # Move past the bar line, restore cursor visibility
        sys.stdout.write("\n" + self._SHOW)
        sys.stdout.flush()


# ─────────────────────────────────────────────────────────────
#  Protocol constants
# ─────────────────────────────────────────────────────────────
PAYLOAD_ID_VR               = 0x01

VR_PID_GET_STATUS           = 0x00
VR_PID_PING                 = 0x00
VR_PID_GET_IMAGE_CAPTURE    = 0x01
VR_PID_IMAGE_REQUEST        = 0x02
VR_PID_IMAGE_DOWNLOAD       = 0x03
VR_PID_IMAGE_DOWNLOAD_DONE  = 0x88
VR_PID_SHUTDOWN             = 0x90  # 0x9X for dangerous command range

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

BAUD_RATE    = 115200
FILE_TO_SAVE = 'source-img/testimg-0.jpg'

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
    print(C.BOLD + C.CYAN + "  ║     VR Image Transfer Tool  v2.0     ║" + C.RESET)
    print(C.BOLD + C.CYAN + "  ╚══════════════════════════════════════╝" + C.RESET)
    print()
    Log.info(f"Port: {C.BOLD}{SERIAL_PORT}{C.RESET}  Baud: {C.BOLD}{BAUD_RATE}{C.RESET}  File: {C.BOLD}{FILE_TO_SAVE}{C.RESET}")
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


def recv_whole_frame() -> bytes:
    global ser
    recv_frame: bytes = b""
    while True:
        try:
            if ser.in_waiting:
                recv_frame += ser.read(ser.in_waiting)
                try:
                    start = recv_frame.index(KISS.FEND)
                    end   = recv_frame.index(KISS.FEND, start + 1)
                    return recv_frame
                except ValueError:
                    pass
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError) as e:
            Log.error(f"Read failed: {e}  — reconnecting …")
            connect_serial()
            recv_frame = b""


# ─────────────────────────────────────────────────────────────
#  Transfer summary
# ─────────────────────────────────────────────────────────────
def print_transfer_summary(total_bytes: int, elapsed: float):
    if elapsed <= 0:
        elapsed = 0.001
    rate_kbps = (total_bytes * 8) / elapsed / 1_000
    rate_KBps = total_bytes / elapsed / 1_024

    print()
    print(C.BOLD + C.GREEN + "\t┌───────────────────────────────────────────────┐" + C.RESET)
    print(C.BOLD + C.GREEN + "\t│            FILE TRANSFER COMPLETE             │" + C.RESET)
    print(C.BOLD + C.GREEN + "\t├───────────────────────────────────────────────┤" + C.RESET)
    line1 = f"{C.CYAN}Total sent  {C.RESET}: {total_bytes:,} bytes ({total_bytes/1_024:.2f} KiB)"
    line2 = f"{C.CYAN}Elapsed     {C.RESET}: {elapsed:.3f} s"
    line3 = f"{C.CYAN}Data rate   {C.RESET}: {rate_kbps:.1f} kbps  ({rate_KBps:.2f} KiB/s)"
    print(f"\t│   {line1.ljust(53)}{C.GREEN}│{C.RESET}")
    print(f"\t│   {line2.ljust(53)}{C.GREEN}│{C.RESET}")
    print(f"\t│   {line3.ljust(53)}{C.GREEN}│{C.RESET}")
    print(C.BOLD + C.GREEN + "\t└───────────────────────────────────────────────┘" + C.RESET)
    print()


# ─────────────────────────────────────────────────────────────
#  Main state-machine loop
# ─────────────────────────────────────────────────────────────
def getCaptureRequest():
    global transfer_start_time, transfer_bytes_sent, _progress_bar

    file_mode        = FILE_MODE_IDLE
    current_chunk_id = 0
    current_file_id  = 0
    total_file_size  = 0        # filled when we first open the file

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
                # Initialise timer + progress bar on first chunk
                if current_chunk_id == 0:
                    transfer_start_time = time.monotonic()
                    transfer_bytes_sent = 0
                    _progress_bar = ProgressBar(total_file_size, CHUNK_SIZE)
                    Log.active_bar = _progress_bar
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

            else:
                if _progress_bar:
                    _progress_bar.finish()
                    _progress_bar  = None
                    Log.active_bar = None

                Log.ok("All chunks sent — sending DOWNLOAD_DONE")
                reply_frame = KISS.wrap_frame(PAYLOAD_ID_VR, VR_PID_IMAGE_DOWNLOAD_DONE, b'', 0x00)
                send_data(reply_frame)

                elapsed = time.monotonic() - transfer_start_time
                print_transfer_summary(transfer_bytes_sent, elapsed)

                file_mode        = FILE_MODE_IDLE
                current_chunk_id = 0
                current_file_id  = 0

        # ── Receive & dispatch ────────────────────────────────
        buf   = recv_whole_frame()
        frame = KISS.unwrap_frame(buf)

        if frame['type'] != 'generic':
            Log.warn(f"Received non-generic frame — ignoring (type={frame['type']})")
            continue

        if frame['payload_id'] != PAYLOAD_ID_VR:
            Log.warn(f"Unknown payload_id=0x{frame['payload_id']:02X} — ignoring")
            continue

        pid = frame['pid']

        # ── STATUS POLL ───────────────────────────────────────
        # if pid == VR_PID_GET_STATUS:
        #     mode_label = FILE_MODE_LABELS.get(file_mode, f"0x{file_mode:02X}")
        #     Log.info(f"Status poll received  [mode={C.BOLD}{mode_label}{C.RESET}]")
        if pid == VR_PID_PING:
            Log.info(f"Ping Request")
            content = KISS.wrap_frame(PAYLOAD_ID_VR,VR_PID_PING,b'',command=KISS.CMD_DATA)
            send_data(content)
            Log.info("Responded")

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

            if frame['data_len'] == 3:
                _file_id   = frame['data'][0]
                _chunk_id  = frame['data'][1:3]
                current_file_id  = _file_id
                requested_chunk  = int.from_bytes(_chunk_id, "big")

                if requested_chunk == 0xFFFF:
                    Log.xfer(f"Full-file transfer requested (file_id={_file_id})")
                    current_chunk_id = 0
                    file_mode        = FILE_MODE_DUMP
                else:
                    Log.xfer(f"Single-chunk request: chunk #{requested_chunk}  file_id={_file_id}")
                    current_chunk_id = requested_chunk
                    file_mode        = FILE_MODE_CHUNK
                reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
                send_data(reply)
                Log.ok("ACK sent for IMAGE REQUEST")
            else:
                Log.warn(f"IMAGE REQUEST had unexpected data_len={frame['data_len']} — ignoring")

        # ── SHUTDOWN COMMAND ──────────────────────────────────
        elif pid == VR_PID_SHUTDOWN:
            Log.cmd("SHUTDOWN command received — sending ACK then shutting down")
            reply = KISS.wrap_frame(PAYLOAD_ID_VR, PID_ACK, b'')
            send_data(reply)
            Log.ok("ACK flushed — powering off now")
            time.sleep(2)
            subprocess.run(["sudo", "systemctl", "poweroff"])

        # ── UNKNOWN PID ───────────────────────────────────────
        else:
            Log.warn(f"Unknown PID 0x{pid:02X} received — ignoring")


# ─────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────
def run():
    print("\033c",end="")
    _banner()
    connect_serial()
    getCaptureRequest()


if __name__ == "__main__":
    run()
