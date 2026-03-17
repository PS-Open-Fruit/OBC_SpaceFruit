import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Visualize Aggressive Resume Test Results")
    parser.add_argument("--csv", type=str, default="test_telemetry.csv", help="Input telemetry CSV file")
    parser.add_argument("--kills", type=str, default="test_kills.csv", help="Input kill events CSV file")
    parser.add_argument("--output", type=str, default="test_results.png", help="Output plot filename")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"[ERROR] Telemetry file '{args.csv}' not found.")
        sys.exit(1)

    # 1. Load Data
    df = pd.read_csv(args.csv)
    
    kills_df = None
    if os.path.exists(args.kills):
        kills_df = pd.read_csv(args.kills)
    else:
        print(f"[NOTE] Kill events file '{args.kills}' not found. No drop markers will be plotted.")

    if df.empty:
        print("[ERROR] Telemetry file is empty.")
        sys.exit(1)

    # 2. Process Data
    start_time = df['Timestamp'].iloc[0]
    df['RelativeTime'] = df['Timestamp'] - start_time
    
    # 3. Plotting
    plt.figure(figsize=(12, 6))
    
    # Font Setup
    plt.rcParams['font.family'] = 'Niramit'
    plt.rcParams['font.sans-serif'] = ['Niramit', 'Arial', 'sans-serif']
    
    plt.plot(df['RelativeTime'], df['BytesDownloaded'], label="Downlink Progress", color="#2ecc71", linewidth=2.5)
    
    # Plot Kill Events (Offline Ranges)
    if kills_df is not None and not kills_df.empty:
        # Subtract start_time to keep relative
        for i, row in kills_df.iterrows():
            k_start = row['OfflineStart'] - start_time
            k_end = row['OfflineEnd'] - start_time
            
            # Start of offline (Red dashed)
            plt.axvline(x=k_start, color="#e74c3c", linestyle="--", alpha=0.8, 
                        label="OBC Offline" if i == 0 else "")
            
            # End of offline (Blue dashed or subtle)
            plt.axvline(x=k_end, color="#3498db", linestyle=":", alpha=0.8,
                        label="OBC Online / Resume" if i == 0 else "")
            
            # Optional: Shaded region between them
            plt.axvspan(k_start, k_end, color='#e74c3c', alpha=0.1)

    # 4. Styling
    plt.title("Aggressive Resume Test: File Downlink Progress", fontsize=14, fontweight='bold', pad=20)
    plt.xlabel("Time (seconds)", fontsize=12)
    plt.ylabel("Bytes Downloaded", fontsize=12)
    
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()

    # 5. Save
    plt.savefig(args.output, dpi=300)
    print(f"[SUCCESS] Visualization saved to '{args.output}'")

if __name__ == "__main__":
    main()
