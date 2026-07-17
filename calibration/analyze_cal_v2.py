#!/usr/bin/env python3
# analyze_cal.py — runs on your LAPTOP (Python 3; pip install matplotlib)
#
# Recommends ENERGY_GATE and RHYTHM_TH for the two-state IDLE / MOVING sketch.
#
# DATA COLLECTION (Arduino sketch, CAL_MODE = 1):
#   The sketch prints CSV lines "variance,rhythm" over serial, 2 per second.
#   For each activity: open the Serial Monitor, do the activity 60-90 s,
#   copy the output, paste into a file named cal_1.csv, cal_2.csv, ...
#   (one activity per file, in the same folder as this script).
#   Junk lines / partial lines are ignored automatically. Chop off the first
#   ~10 s of each capture if it includes you getting set up.
#
# LABELING: fill in LABELS below. Categories:
#   "idle_quiet"  -> should read IDLE, low energy   (sitting, standing)
#   "idle_active" -> should read IDLE, high energy possible (fidgeting, car/bus)
#   "moving"      -> should read MOVING (slow walk, walk, run)
#
# The two categories of idle matter because they constrain different knobs:
#   ENERGY_GATE separates idle_quiet from moving (energy axis)
#   RHYTHM_TH   separates idle_active from moving (rhythm axis)

import csv
import glob
import os
import sys

# ---- EDIT THIS to match your sessions --------------------------------------
LABELS = {
    # file number: ("name for the plot", "category")
    1: ("sitting",     "idle_quiet"),
    2: ("standing",    "idle_quiet"),
    3: ("fidgeting",   "idle_active"),
    4: ("car/bus",     "idle_active"),
    5: ("slow walk",   "moving"),
    6: ("normal walk", "moving"),
    7: ("running",     "moving"),
}
# -----------------------------------------------------------------------------

COLORS = {"idle_quiet": "tab:blue", "idle_active": "tab:orange", "moving": "tab:green"}

def load():
    data = {}
    for path in sorted(glob.glob("cal_*.csv")):
        try:
            num = int(os.path.splitext(os.path.basename(path))[0].split("_")[1])
        except (IndexError, ValueError):
            continue
        rows = []
        with open(path) as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) != 2:
                    continue
                try:
                    rows.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    continue  # header or junk line
        if rows:
            data[num] = rows
    return data

def percentile(vals, p):
    vals = sorted(vals)
    if not vals:
        return float("nan")
    k = (len(vals) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(vals) - 1)
    return vals[lo] + (vals[hi] - vals[lo]) * (k - lo)

def main():
    data = load()
    if not data:
        sys.exit("No cal_*.csv files found in this folder.")

    unlabeled = [n for n in data if n not in LABELS]
    if unlabeled:
        print("WARNING: sessions not in LABELS (skipped):", unlabeled)

    cats = {"idle_quiet": [], "idle_active": [], "moving": []}
    for num, rows in data.items():
        if num in LABELS:
            cats[LABELS[num][1]].extend(rows)

    if not cats["idle_quiet"] or not cats["moving"]:
        sys.exit("Need at least one 'idle_quiet' and one 'moving' session labeled.")

    # ---- recommend thresholds ----
    quiet_var_hi = percentile([v for v, _ in cats["idle_quiet"]], 95)
    mov_var_lo   = percentile([v for v, _ in cats["moving"]], 5)
    mov_rhy_lo   = percentile([r for _, r in cats["moving"]], 5)

    energy_gate = (max(quiet_var_hi, 1e-6) * max(mov_var_lo, 1e-6)) ** 0.5

    if cats["idle_active"]:
        act_rhy_hi = percentile([r for _, r in cats["idle_active"]], 95)
        rhythm_th  = (act_rhy_hi + mov_rhy_lo) / 2.0
    else:
        act_rhy_hi = None
        rhythm_th  = max(0.5, mov_rhy_lo - 0.1)

    # ---- score: what fraction of each session classifies correctly ----
    def predict(v, r):
        return "MOVING" if (v >= energy_gate and r > rhythm_th) else "IDLE"

    print("=" * 64)
    print("idle_quiet variance   95th pct : %12.0f" % quiet_var_hi)
    print("moving variance        5th pct : %12.0f" % mov_var_lo)
    if act_rhy_hi is not None:
        print("idle_active rhythm    95th pct : %12.2f" % act_rhy_hi)
    print("moving rhythm          5th pct : %12.2f" % mov_rhy_lo)
    print("-" * 64)
    print("RECOMMENDED:  ENERGY_GATE = %.0f" % energy_gate)
    print("RECOMMENDED:  RHYTHM_TH   = %.2f" % rhythm_th)
    print("-" * 64)
    print("Per-session accuracy with these thresholds:")
    worst = None
    for num in sorted(data):
        if num not in LABELS:
            continue
        name, cat = LABELS[num]
        want = "MOVING" if cat == "moving" else "IDLE"
        rows = data[num]
        ok = sum(1 for v, r in rows if predict(v, r) == want)
        acc = 100.0 * ok / len(rows)
        print("  %2d %-12s (%-11s want %-6s): %5.1f%%  (%d/%d windows)"
              % (num, name, cat, want, acc, ok, len(rows)))
        if worst is None or acc < worst[0]:
            worst = (acc, name)
    if worst and worst[0] < 90:
        print("NOTE: '%s' is the weak spot (%.0f%%). Check its cluster in the" % (worst[1], worst[0]))
        print("      scatter plot; nudge the nearest threshold toward it, or")
        print("      recollect that session if it looks contaminated.")
    print("=" * 64)

    # ---- plots ----
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    for num in sorted(data):
        if num not in LABELS:
            continue
        name, cat = LABELS[num]
        vs = [max(v, 1e-3) for v, _ in data[num]]
        ax1.hist(vs, bins=30, alpha=0.55, label="%d: %s" % (num, name), color=COLORS[cat])
    ax1.axvline(energy_gate, color="k", linestyle="--", label="ENERGY_GATE")
    ax1.set_xscale("log")
    ax1.set_xlabel("variance (log scale)")
    ax1.set_ylabel("windows")
    ax1.set_title("Energy per condition")
    ax1.legend(fontsize=8)

    for num in sorted(data):
        if num not in LABELS:
            continue
        name, cat = LABELS[num]
        xs = [max(v, 1e-3) for v, _ in data[num]]
        ys = [r for _, r in data[num]]
        ax2.scatter(xs, ys, s=12, alpha=0.6, label="%d: %s" % (num, name), color=COLORS[cat])
    ax2.axvline(energy_gate, color="k", linestyle="--")
    ax2.axhline(rhythm_th, color="k", linestyle=":")
    ax2.set_xscale("log")
    ax2.set_xlabel("variance (log scale)")
    ax2.set_ylabel("rhythm (autocorr peak)")
    ax2.set_title("MOVING = right of dashed AND above dotted; all else IDLE")
    ax2.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig("calibration_plots.png", dpi=140)
    print("Saved calibration_plots.png")
    plt.show()

if __name__ == "__main__":
    main()
