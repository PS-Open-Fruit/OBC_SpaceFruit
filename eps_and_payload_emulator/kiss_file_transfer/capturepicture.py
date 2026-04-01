# import cv2
# import datetime
# import os


# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()

# if ret:
#     filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
#     cv2.imwrite(filename, frame)
#     print(f"Captured: {filename}")
# else:
#     print("Error: Failed to capture image from camera.")

# cap.release()

import subprocess
import datetime
import os
import sys
import argparse

def main():
    # 1. Setup Argument Parser
    parser = argparse.ArgumentParser(description="Capture an image using fswebcam.")
    
    parser.add_argument("-o", "--output", type=str, default="source-img/testimg-1.jpg", help="Output filename (default: timestamped)")
    parser.add_argument("-r", "--resolution", type=str, default="640*480", help="Resolution (default: 1280x720)")
    parser.add_argument("-d", "--device", type=str, default="/dev/video0", help="Camera device (default: /dev/video0)")
    parser.add_argument("-s", "--skip", type=int, default=20, help="Frames to skip for exposure (default: 20)")

    args = parser.parse_args()

    # 2. Determine Filename
    if args.output:
        filename = args.output
    else:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"cap_{timestamp}.jpg"

    # 3. Build fswebcam Command
    # -S: Skip frames (crucial for USB webcams to adjust light/focus)
    command = [
        "fswebcam",
        "-d", args.device,
        "-r", args.resolution,
        "-S", str(args.skip),
        "--no-banner",
        "--quiet",
        filename
    ]

    try:
        print(f"DEBUG: Executing {' '.join(command)}")
        subprocess.run(command, check=True, capture_output=True, text=True)
        
        if os.path.exists(filename):
            print(f"SUCCESS: Saved to {filename}")
            sys.exit(0)
        else:
            print("ERROR: fswebcam completed but file is missing.")
            sys.exit(1)

    except subprocess.CalledProcessError as e:
        print(f"ERROR: fswebcam failed. {e.stderr}")
        sys.exit(1)
    except FileNotFoundError:
        print("ERROR: fswebcam not found. Install it with: sudo apt install fswebcam")
        sys.exit(1)

if __name__ == "__main__":
    main()