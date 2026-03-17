import subprocess
import time
import sys
import threading
import argparse
import os

def read_output(process, name):
    """Utility to read and print subprocess output."""
    for line in iter(process.stdout.readline, ""):
        print(f"[{name}] {line.strip()}")
    process.stdout.close()

def main():
    parser = argparse.ArgumentParser(description="Automated GS CLI Test (GS Only)")
    parser.add_argument("--gs_port", type=str, default="COM4", help="Serial port for GS.py")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument("--filename", type=str, default="0.jpg", help="Filename for info/download tests")
    args = parser.parse_args()

    print(f"--- Starting Automated GS Test ---")
    print(f"GS Port: {args.gs_port}, Baud: {args.baud}")

    # 1. Start Ground Station as a background process (Unbuffered)
    gs_proc = subprocess.Popen(
        [sys.executable, "-u", "GS.py", "--port", args.gs_port, "--baud", str(args.baud)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    # Start thread to log GS output
    threading.Thread(target=read_output, args=(gs_proc, "GS"), daemon=True).start()

    # Allow some time for startup
    time.sleep(2)

    commands = [
        "ping obc",
        "ping vr",
        "list",
        "copy",
        f"info {args.filename}",
        # f"download {args.filename}",
        "status",
        "capture",
        "shutdown",
        "exit"
    ]

    try:
        for cmd in commands:
            print(f"\n[TEST] Sending '{cmd}' command to GS...")
            gs_proc.stdin.write(cmd + "\n")
            gs_proc.stdin.flush()
            
            # Wait for response based on command type
            if "download" in cmd:
                time.sleep(10) # Longer wait for downloads
            elif "exit" in cmd:
                # Special handling for exit
                try:
                    gs_proc.wait(timeout=5)
                    print("[TEST] GS exited normally.")
                except subprocess.TimeoutExpired:
                    print("[TEST] GS did not exit in time, will terminate.")
            else:
                time.sleep(3)

    except Exception as e:
        print(f"[TEST] Error during test: {e}")

    finally:
        # Cleanup if still running
        if gs_proc.poll() is None:
            print("\n[TEST] Cleaning up (terminating GS)...")
            gs_proc.terminate()
            gs_proc.wait()
        
        print("--- All Tests Completed ---")

if __name__ == "__main__":
    main()
