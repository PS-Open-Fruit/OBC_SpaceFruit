"""
plot_metrics.py  —  visualise chunk-size sweep results from transfer_metrics.csv

Usage:
    python plot_metrics.py                    # reads transfer_metrics.csv in CWD
    python plot_metrics.py my_metrics.csv     # custom file
"""

import sys
import csv
import collections
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

CSV_FILE = sys.argv[1] if len(sys.argv) > 1 else "transfer_metrics.csv"

# ── Load data ──────────────────────────────────────────────────────────────────
rows = []
with open(CSV_FILE, newline="") as f:
    for row in csv.DictReader(f):
        rows.append({
            "chunk_size":   int(row["chunk_size"]),
            "total_bytes":  int(row["total_bytes"]),
            "elapsed_s":    float(row["elapsed_s"]),
            "rate_kbps":    float(row["rate_kbps"]),
            "rate_KBps":    float(row["rate_KBps"]),
            "total_chunks": int(row["total_chunks"]),
            "timestamp":    row["timestamp"],
        })

if not rows:
    print("No data found in", CSV_FILE)
    sys.exit(1)

# ── Aggregate: mean + all individual runs per chunk_size ──────────────────────
by_chunk: dict[int, list[dict]] = collections.defaultdict(list)
for r in rows:
    by_chunk[r["chunk_size"]].append(r)

chunk_sizes  = sorted(by_chunk)
mean_kbps    = [sum(r["rate_kbps"]  for r in by_chunk[c]) / len(by_chunk[c]) for c in chunk_sizes]
mean_elapsed = [sum(r["elapsed_s"]  for r in by_chunk[c]) / len(by_chunk[c]) for c in chunk_sizes]
mean_chunks  = [sum(r["total_chunks"] for r in by_chunk[c]) / len(by_chunk[c]) for c in chunk_sizes]

# ── Plot ───────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
fig.suptitle("Chunk-size Sweep — Transfer Metrics", fontsize=14, fontweight="bold")

COLOR_MEAN   = "#2196F3"
COLOR_SCATTER = "#FF9800"
COLOR_ELAPSED = "#4CAF50"
COLOR_CHUNKS  = "#9C27B0"

# — Panel 1 : throughput (kbps) ────────────────────────────────────────────────
ax1 = axes[0]
# scatter all individual runs
for cs in chunk_sizes:
    ys = [r["rate_kbps"] for r in by_chunk[cs]]
    ax1.scatter([cs] * len(ys), ys, color=COLOR_SCATTER, alpha=0.55, s=30, zorder=3)
ax1.plot(chunk_sizes, mean_kbps, "o-", color=COLOR_MEAN, lw=2, ms=7, label="mean", zorder=4)
ax1.set_ylabel("Throughput (kbps)")
ax1.set_title("Throughput vs Chunk Size")
ax1.legend()
ax1.grid(True, linestyle="--", alpha=0.4)
ax1.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.0f"))

# annotate best point
best_idx = mean_kbps.index(max(mean_kbps))
ax1.annotate(
    f"best: {chunk_sizes[best_idx]} B\n{mean_kbps[best_idx]:.0f} kbps",
    xy=(chunk_sizes[best_idx], mean_kbps[best_idx]),
    xytext=(10, -20), textcoords="offset points",
    arrowprops=dict(arrowstyle="->", color="red"),
    color="red", fontsize=9,
)

# — Panel 2 : elapsed time ─────────────────────────────────────────────────────
ax2 = axes[1]
for cs in chunk_sizes:
    ys = [r["elapsed_s"] for r in by_chunk[cs]]
    ax2.scatter([cs] * len(ys), ys, color=COLOR_SCATTER, alpha=0.55, s=30, zorder=3)
ax2.plot(chunk_sizes, mean_elapsed, "s-", color=COLOR_ELAPSED, lw=2, ms=7, label="mean", zorder=4)
ax2.set_ylabel("Elapsed (s)")
ax2.set_title("Transfer Time vs Chunk Size")
ax2.legend()
ax2.grid(True, linestyle="--", alpha=0.4)

# — Panel 3 : number of chunks ─────────────────────────────────────────────────
ax3 = axes[2]
ax3.plot(chunk_sizes, mean_chunks, "^-", color=COLOR_CHUNKS, lw=2, ms=7)
ax3.set_ylabel("# Chunks (ACK round-trips)")
ax3.set_xlabel("Chunk size (bytes)")
ax3.set_title("Number of Chunks vs Chunk Size")
ax3.grid(True, linestyle="--", alpha=0.4)
ax3.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.0f"))

# x-axis ticks = every tested chunk size
ax3.set_xticks(chunk_sizes)
ax3.set_xticklabels([str(c) for c in chunk_sizes], rotation=45, ha="right")

plt.tight_layout()
out = "transfer_metrics_plot.png"
plt.savefig(out, dpi=150)
print(f"Saved → {out}")
plt.show()