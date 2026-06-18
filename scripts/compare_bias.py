#!/usr/bin/env python3
"""
Compare IMU bias estimates from ORB-SLAM3 bias_log.txt against
EuRoC ground-truth bias values from state_groundtruth_estimate0/data.csv.

Outputs per-timestamp comparison and summary statistics.
"""

import argparse
import csv
import os
import re
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def parse_groundtruth(csv_path: str) -> dict:
    """Parse EuRoC groundtruth CSV, return dict: timestamp_ns -> (bgx,bgy,bgz,bax,bay,baz)."""
    gt = {}
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        # Find column indices
        col_map = {h.strip().lstrip("#"): i for i, h in enumerate(header)}
        ts_idx = col_map["timestamp"]
        bgx_idx = col_map["b_w_RS_S_x [rad s^-1]"]
        bgy_idx = col_map["b_w_RS_S_y [rad s^-1]"]
        bgz_idx = col_map["b_w_RS_S_z [rad s^-1]"]
        bax_idx = col_map["b_a_RS_S_x [m s^-2]"]
        bay_idx = col_map["b_a_RS_S_y [m s^-2]"]
        baz_idx = col_map["b_a_RS_S_z [m s^-2]"]

        for row in reader:
            if not row:
                continue
            ts_ns = int(row[ts_idx])
            bg = np.array([float(row[bgx_idx]), float(row[bgy_idx]), float(row[bgz_idx])])
            ba = np.array([float(row[bax_idx]), float(row[bay_idx]), float(row[baz_idx])])
            gt[ts_ns] = (bg, ba)
    return gt


def parse_bias_log(log_path: str) -> list:
    """Parse bias_log.txt, return list of (timestamp_s, bg, ba).

    Expected log format:
        [INFO] [...] [Tracking]: Current ts: 1413393984.705760, gyro bias: -0.001597, 0.024367, 0.078578, acc bias: -0.044747, 0.128649, 0.046120
    """
    entries = []
    pattern = re.compile(
        r"Current ts:\s+([\d.]+),\s*"
        r"gyro bias:\s+([\-\d.eE+]+),\s*([\-\d.eE+]+),\s*([\-\d.eE+]+),\s*"
        r"acc bias:\s+([\-\d.eE+]+),\s*([\-\d.eE+]+),\s*([\-\d.eE+]+)"
    )
    with open(log_path, "r") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                ts_s = float(m.group(1))
                bg = np.array([float(m.group(2)), float(m.group(3)), float(m.group(4))])
                ba = np.array([float(m.group(5)), float(m.group(6)), float(m.group(7))])
                entries.append((ts_s, bg, ba))
    return entries


def align_and_compare(gt: dict, bias_entries: list, max_time_diff_s: float = 0.05):
    """Align bias estimates to groundtruth by nearest timestamp and compute errors."""
    gt_timestamps_ns = sorted(gt.keys())
    results = []

    for ts_s, bg_est, ba_est in bias_entries:
        # Convert estimate timestamp (seconds) to nanoseconds
        ts_ns_est = int(ts_s * 1e9)

        # Find nearest groundtruth timestamp
        best_ts = None
        best_diff = float("inf")
        for ts_ns in gt_timestamps_ns:
            diff = abs(ts_ns - ts_ns_est)
            if diff < best_diff:
                best_diff = diff
                best_ts = ts_ns
            else:
                # Since sorted, once diff starts increasing we can stop
                if diff > best_diff:
                    break

        if best_ts is None or best_diff > max_time_diff_s * 1e9:
            continue

        bg_gt, ba_gt = gt[best_ts]
        bg_err = bg_est - bg_gt
        ba_err = ba_est - ba_gt
        results.append({
            "ts_est_s": ts_s,
            "ts_gt_ns": best_ts,
            "ts_diff_ms": best_diff * 1e-6,
            "bg_est": bg_est,
            "bg_gt": bg_gt,
            "bg_err": bg_err,
            "ba_est": ba_est,
            "ba_gt": ba_gt,
            "ba_err": ba_err,
        })

    return results


def print_summary(results: list):
    """Print summary statistics."""
    if not results:
        print("[WARN] No matched bias entries found.")
        return

    n = len(results)
    bg_errs = np.array([r["bg_err"] for r in results])       # (N, 3)
    ba_errs = np.array([r["ba_err"] for r in results])       # (N, 3)
    bg_err_norm = np.linalg.norm(bg_errs, axis=1)
    ba_err_norm = np.linalg.norm(ba_errs, axis=1)

    print(f"Total matched entries: {n}")
    print()
    print("=" * 85)
    print("  Bias Error Summary (estimate - groundtruth)")
    print("=" * 85)
    print(f"{'Component':<12} {'Mean':>12} {'Std':>12} {'RMSE':>12} {'MaxAbs':>12}")
    print("-" * 85)

    labels = ["bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"]
    all_errs = np.hstack([bg_errs, ba_errs])  # (N, 6)
    for i, label in enumerate(labels):
        mean = np.mean(all_errs[:, i])
        std = np.std(all_errs[:, i])
        rmse = np.sqrt(np.mean(all_errs[:, i] ** 2))
        maxabs = np.max(np.abs(all_errs[:, i]))
        print(f"{label:<12} {mean:12.6f} {std:12.6f} {rmse:12.6f} {maxabs:12.6f}")

    print("-" * 85)
    print(f"{'||bg_err||':<12} {np.mean(bg_err_norm):12.6f} {np.std(bg_err_norm):12.6f} "
          f"{np.sqrt(np.mean(bg_err_norm**2)):12.6f} {np.max(bg_err_norm):12.6f}")
    print(f"{'||ba_err||':<12} {np.mean(ba_err_norm):12.6f} {np.std(ba_err_norm):12.6f} "
          f"{np.sqrt(np.mean(ba_err_norm**2)):12.6f} {np.max(ba_err_norm):12.6f}")
    print("=" * 85)


def save_per_entry(results: list, output_path: str):
    """Save per-timestamp comparison to CSV."""
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "ts_est_s", "ts_gt_ns", "ts_diff_ms",
            "bg_est_x", "bg_est_y", "bg_est_z",
            "bg_gt_x", "bg_gt_y", "bg_gt_z",
            "bg_err_x", "bg_err_y", "bg_err_z",
            "ba_est_x", "ba_est_y", "ba_est_z",
            "ba_gt_x", "ba_gt_y", "ba_gt_z",
            "ba_err_x", "ba_err_y", "ba_err_z",
        ])
        for r in results:
            writer.writerow([
                f"{r['ts_est_s']:.9f}", r["ts_gt_ns"], f"{r['ts_diff_ms']:.3f}",
                r["bg_est"][0], r["bg_est"][1], r["bg_est"][2],
                r["bg_gt"][0], r["bg_gt"][1], r["bg_gt"][2],
                r["bg_err"][0], r["bg_err"][1], r["bg_err"][2],
                r["ba_est"][0], r["ba_est"][1], r["ba_est"][2],
                r["ba_gt"][0], r["ba_gt"][1], r["ba_gt"][2],
                r["ba_err"][0], r["ba_err"][1], r["ba_err"][2],
            ])


def plot_comparison_pdf(results: list, output_pdf: str):
    """Plot groundtruth vs estimate for all 6 bias axes in a single PDF file.

    Each axis gets its own page with both groundtruth and estimated curves
    plotted together over time.
    """
    if not results:
        print("[WARN] No data to plot.")
        return

    # Extract data arrays
    ts_est = np.array([r["ts_est_s"] for r in results])  # estimate timestamps (seconds)
    ts_gt = np.array([r["ts_gt_ns"] for r in results]) * 1e-9  # groundtruth timestamps -> seconds

    # Align to a common time origin for better readability
    t0 = min(ts_est[0], ts_gt[0])
    ts_est_rel = ts_est - t0
    ts_gt_rel = ts_gt - t0

    bg_est = np.array([r["bg_est"] for r in results])  # (N, 3)
    bg_gt = np.array([r["bg_gt"] for r in results])    # (N, 3)
    ba_est = np.array([r["ba_est"] for r in results])  # (N, 3)
    ba_gt = np.array([r["ba_gt"] for r in results])    # (N, 3)

    axes_info = [
        ("gyro", "bg_x", "Gyro Bias X (rad/s)", 0, bg_est, bg_gt),
        ("gyro", "bg_y", "Gyro Bias Y (rad/s)", 1, bg_est, bg_gt),
        ("gyro", "bg_z", "Gyro Bias Z (rad/s)", 2, bg_est, bg_gt),
        ("acc",  "ba_x", "Acc Bias X (m/s^2)",  0, ba_est, ba_gt),
        ("acc",  "ba_y", "Acc Bias Y (m/s^2)",  1, ba_est, ba_gt),
        ("acc",  "ba_z", "Acc Bias Z (m/s^2)",  2, ba_est, ba_gt),
    ]

    with PdfPages(output_pdf) as pdf:
        for group, label, ylabel, idx, est_all, gt_all in axes_info:
            fig, ax = plt.subplots(figsize=(12, 5))

            ax.plot(ts_gt_rel, gt_all[:, idx], "b-",  linewidth=1.5, label="Groundtruth")
            ax.plot(ts_est_rel, est_all[:, idx], "r--", linewidth=1.5, label="Estimated")

            ax.set_xlabel("Time (s, relative to start)")
            ax.set_ylabel(ylabel)
            ax.set_title(f"{label} — Groundtruth vs Estimate")
            ax.legend(loc="best")
            ax.grid(True, alpha=0.3)

            # Print summary stats on the plot
            err = est_all[:, idx] - gt_all[:, idx]
            rmse = np.sqrt(np.mean(err ** 2))
            mean_err = np.mean(err)
            ax.text(0.02, 0.97, f"RMSE: {rmse:.6f}  |  Mean Err: {mean_err:.6f}",
                    transform=ax.transAxes, fontsize=9, verticalalignment="top",
                    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

    print(f"\nComparison plots saved to: {output_pdf}")


def main():
    parser = argparse.ArgumentParser(
        description="Compare ORB-SLAM3 IMU bias estimates with EuRoC groundtruth."
    )
    parser.add_argument("groundtruth", help="Path to groundtruth data.csv")
    parser.add_argument("bias_log", help="Path to bias_log.txt")
    parser.add_argument("-o", "--output", default=None,
                        help="Output CSV for per-entry comparison")
    parser.add_argument("-p", "--pdf", default=None,
                        help="Output PDF for comparison plots (6 axes in one file)")
    parser.add_argument("--max-time-diff", type=float, default=0.05,
                        help="Max time difference (seconds) for matching (default: 0.05)")
    args = parser.parse_args()

    if not os.path.isfile(args.groundtruth):
        print(f"[ERROR] Groundtruth file not found: {args.groundtruth}")
        sys.exit(1)
    if not os.path.isfile(args.bias_log):
        print(f"[ERROR] Bias log file not found: {args.bias_log}")
        sys.exit(1)

    print(f"Loading groundtruth: {args.groundtruth}")
    gt = parse_groundtruth(args.groundtruth)
    print(f"  Loaded {len(gt)} groundtruth entries")

    print(f"Loading bias log: {args.bias_log}")
    bias_entries = parse_bias_log(args.bias_log)
    print(f"  Loaded {len(bias_entries)} bias estimate entries")

    results = align_and_compare(gt, bias_entries, args.max_time_diff)

    print_summary(results)

    if args.output and results:
        save_per_entry(results, args.output)
        print(f"\nPer-entry comparison saved to: {args.output}")

    if args.pdf and results:
        plot_comparison_pdf(results, args.pdf)


if __name__ == "__main__":
    main()
