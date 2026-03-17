import subprocess
import time
import sys
import threading
import argparse
import os
import hashlib
import re
import csv
import random

def read_output(process, name, logger):
    """Utility to read and print subprocess output."""
    for line in iter(process.stdout.readline, ""):
        line = line.strip()
        if line:
            logger.append((time.time(), name, line))
    process.stdout.close()

def get_md5(fname):
    hash_md5 = hashlib.md5()
    with open(fname, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

# ANSI Color Codes
CLR_YELLOW = "\033[93m"
CLR_RED    = "\033[91m"
CLR_GREEN  = "\033[92m"
CLR_RESET  = "\033[0m"

def main():
    parser = argparse.ArgumentParser(description="Scenario 3B: Image Downlink Aggressive Resume")
    parser.add_argument("--gs_port", type=str, default="COM4", help="Serial port for GS.py")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument("--kill_interval", type=int, default=15, help="Seconds between GS termination")
    parser.add_argument("--csv", type=str, default="test_telemetry.csv", help="CSV file for telemetry data")
    parser.add_argument("--kills_csv", type=str, default="test_kills.csv", help="CSV file for kill events")
    args = parser.parse_args()

    source_file = os.path.join("sd_card", "0.jpg")
    dest_file = os.path.join("downloads", "0.jpg")

    if not os.path.exists(source_file):
        print(f"{CLR_YELLOW}[NOTE] Local reference file '{source_file}' not found. Test will run but MD5 verification will be skipped.{CLR_RESET}")

    if os.path.exists(dest_file):
        os.remove(dest_file)

    print(f"\n--- Starting Scenario 3B: Image Downlink Aggressive Resume ---")
    print(f"File: {source_file}, GS Port: {args.gs_port}")
    print(f"Aggressive Kill Interval: {args.kill_interval}s")

    # Initialize CSV files with headers
    try:
        with open(args.csv, 'w', newline='') as f:
            csv.writer(f).writerow(["Timestamp", "BytesDownloaded"])
        with open(args.kills_csv, 'w', newline='') as f:
            csv.writer(f).writerow(["OfflineStart", "OfflineEnd"])
        print(f"{CLR_GREEN}[SUCCESS]{CLR_RESET} CSV logs initialized.")
    except Exception as e:
        print(f"{CLR_RED}[ERROR]{CLR_RESET} Failed to initialize CSV logs: {e}")
        sys.exit(1)
    
    # Progress regex: "Offset: 12345"
    progress_regex = re.compile(r"Offset:\s*(\d+)")

    time.sleep(2)

    total_size = os.path.getsize(source_file)
    success = False
    iteration = 1
    
    try:
        while not success:
            print(f"\n[ITERATION {iteration}] Starting GS and initiating/resuming download...")
            gs_logs = []
            gs_proc = subprocess.Popen([sys.executable, "-u", "GS.py", "--port", args.gs_port, "--baud", str(args.baud)], 
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            threading.Thread(target=read_output, args=(gs_proc, f"GS_IT{iteration}", gs_logs), daemon=True).start()

            time.sleep(1)
            gs_proc.stdin.write("download 0.jpg\n")
            gs_proc.stdin.flush()

            it_start = time.time()
            it_completed = False
            
            while time.time() - it_start < args.kill_interval:
                # Process logs
                while gs_logs:
                    t, n, line = gs_logs.pop(0)
                    print(f"[{n}] {line}")
                    
                    # Track progress for visualization
                    match = progress_regex.search(line)
                    if match:
                        offset = int(match.group(1))
                        # Append to CSV in real-time
                        try:
                            with open(args.csv, 'a', newline='') as f:
                                csv.writer(f).writerow([t, offset])
                        except: pass

                    if "Download Complete!" in line:
                        it_completed = True
                        success = True
                        break
                if it_completed:
                    break
                time.sleep(0.5)

            if success:
                print(f"\n[ITERATION {iteration}] DOWNLOAD COMPLETED SUCCESSFULLY!")
                gs_proc.terminate()
                gs_proc.wait()
                break
            
            # Simulated connection drop
            offline_start = time.time()
            
            print(f"\n[ITERATION {iteration}] SIMULATING CONNECTION DROP (Terminating GS after {args.kill_interval}s)...")
            gs_proc.terminate()
            gs_proc.wait()

            # Random Offline Delay
            delay = random.uniform(5, 15)
            print(f"[TEST] OBC Offline for {delay:.1f} seconds...")
            time.sleep(delay)
            
            offline_end = time.time()
            
            # Log offline range
            try:
                with open(args.kills_csv, 'a', newline='') as f:
                    csv.writer(f).writerow([offline_start, offline_end])
            except: pass
            
            print(f"[TEST] Attempting to resume downlink...")
            
            if os.path.exists(dest_file):
                current_size = os.path.getsize(dest_file)
                print(f"Current progress: {current_size}/{total_size} bytes ({(current_size/total_size)*100:.1f}%)")
            
            iteration += 1
            time.sleep(3) # Small gap before next attempt

        print(f"\n--- Test Results ---")
        if success:
            print(f"{CLR_GREEN}[SUCCESS]{CLR_RESET} Download completed.")
            
            # Final data point
            if os.path.exists(dest_file):
                final_size = os.path.getsize(dest_file)
                try:
                    with open(args.csv, 'a', newline='') as f:
                        csv.writer(f).writerow([time.time(), final_size])
                except: pass

            print(f"{CLR_GREEN}[SUCCESS]{CLR_RESET} Real-time telemetry logging finished.")

            if os.path.exists(source_file):
                src_md5 = get_md5(source_file)
                dst_md5 = get_md5(dest_file)
                print(f"Source MD5 (Local Reference): {src_md5}")
                print(f"Dest   MD5 (Downloaded):      {dst_md5}")
                if src_md5 == dst_md5:
                    print(f"{CLR_GREEN}[SUCCESS]{CLR_RESET} MD5 verification PASSED after {iteration} resume cycles.")
                else:
                    print(f"{CLR_RED}[FAIL]{CLR_RESET} MD5 verification FAILED. Data corruption detected after multiple resumes.")
            else:
                print(f"{CLR_YELLOW}[NOTE]{CLR_RESET} Local reference file '{source_file}' not found. Skipping MD5 verification.")
                print(f"{CLR_GREEN}[SUCCESS]{CLR_RESET} Download finished, but data integrity was not verified against a local copy.")
        else:
            print(f"{CLR_RED}[FAIL]{CLR_RESET} Test failed to complete download.")

    except KeyboardInterrupt:
        print("\n[TEST] Interrupted by user.")
    finally:
        print("[TEST] Cleaning up...")
        if 'gs_proc' in locals() and gs_proc.poll() is None:
            gs_proc.terminate()
            gs_proc.wait()

if __name__ == "__main__":
    main()
