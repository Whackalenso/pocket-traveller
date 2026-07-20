#!/usr/bin/env python3
# analyze_cal.py — runs on your LAPTOP (normal Python 3, needs matplotlib).
#
#   pip install matplotlib
#   python3 analyze_cal.py
#
# 1. Pull the cal_*.csv files off the Pico (Thonny: View -> Files, right-click
#    each file on the Pico side -> Download) into the same folder as this script.
# 2. Fill in LABELS below: which session number was which activity, and which
#    CATEGORY each belongs to:
#      "idle"       -> should read still, low energy (sitting, standing)
#      "jostle"     -> should read still, HIGH energy possible (bus, fidgeting)
#      "locomotion" -> should read moving (walking, running)
# 3. Run it. You get two plots + suggested ENERGY_GATE and RHYTHM_TH values.

import csv
import glob
import os
import sys

# ---- EDIT THIS to match your sessions --------------------------------------
LABELS = {
    # file number: ("name for the plot", "category")
    1: ("sitting",        "idle"),
    2: ("standing",       "idle"),
    3: ("slow walk",      "locomotion"),
    4: ("normal walk",    "locomotion"),
    5: ("running",        "locomotion"),
    6: ("bus ride",       "jostle"),
    7: ("fidgeting",      "jostle"),
}
# -----------------------------------------------------------------------------

CATEGORY_COLORS = {"idle": "tab:blue", "jostle": "tab:orange", "locomotion": "tab:green"}

def load():
    data = {}   # num -> list of (var, rhythm)
    for path in sorted(glob.glob("cal_*.csv")):
        num = int(os.path.splitext(os.path.basename(path))[0].split("_")[1])
        rows = []
        with open(path) as f:
            for row in csv.DictReader(f):
                try:
                    rows.append((float(row["variance"]), float(row["rhythm"])))
                except (KeyError, ValueError):
                    pass
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
        print("WARNING: sessions with no entry in LABELS (skipped):", unlabeled)

    by_cat = {"idle": [], "jostle": [], "locomotion": []}
    for num, rows in data.items():
        if num in LABELS:
            by_cat[LABELS[num][1]].extend(rows)

    if not by_cat["idle"] or not by_cat["locomotion"]:
        sys.exit("Need at least one 'idle' and one 'locomotion' session labeled.")

    # ---- threshold suggestions ----
    idle_var_hi = percentile([v for v, _ in by_cat["idle"]], 95)
    loco_var_lo = percentile([v for v, _ in by_cat["locomotion"]], 5)
    loco_rhy_lo = percentile([r for _, r in by_cat["locomotion"]], 5)

    energy_gate = (idle_var_hi * loco_var_lo) ** 0.5   # geometric midpoint of the gap
    var_overlap = idle_var_hi >= loco_var_lo

    if by_cat["jostle"]:
        jostle_rhy_hi = percentile([r for _, r in by_cat["jostle"]], 95)
        rhythm_th = (jostle_rhy_hi + loco_rhy_lo) / 2.0
        rhy_overlap = jostle_rhy_hi >= loco_rhy_lo
    else:
        jostle_rhy_hi = None
        rhythm_th = max(0.5, loco_rhy_lo - 0.1)
        rhy_overlap = False

    print("=" * 60)
    print("idle variance        95th pct : %12.0f" % idle_var_hi)
    print("locomotion variance   5th pct : %12.0f" % loco_var_lo)
    if jostle_rhy_hi is not None:
        print("jostle rhythm        95th pct : %12.2f" % jostle_rhy_hi)
    print("locomotion rhythm     5th pct : %12.2f" % loco_rhy_lo)
    print("-" * 60)
    print("SUGGESTED:  ENERGY_GATE = %.0f" % energy_gate)
    print("SUGGESTED:  RHYTHM_TH   = %.2f" % rhythm_th)
    if var_overlap:
        print("NOTE: idle and locomotion variance OVERLAP — the energy gate")
        print("      alone can't separate them cleanly; rhythm will carry more")
        print("      weight. Consider a lower gate + trusting RHYTHM_TH.")
    if rhy_overlap:
        print("NOTE: jostle and locomotion rhythm OVERLAP — check which jostle")
        print("      session causes it in the scatter plot; that condition may")
        print("      genuinely be rhythmic (e.g. finger drumming).")
    print("=" * 60)

    # ---- plots ----
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # histogram of variance per session (log x — variance spans decades)
    for num in sorted(data):
        if num not in LABELS:
            continue
        name, cat = LABELS[num]
        vars_ = [max(v, 1e-3) for v, _ in data[num]]
        ax1.hist(vars_, bins=30, alpha=0.55, label="%d: %s" % (num, name),
                 color=CATEGORY_COLORS[cat])
    ax1.axvline(energy_gate, color="k", linestyle="--", label="ENERGY_GATE")
    ax1.set_xscale("log")
    ax1.set_xlabel("variance (raw LSB^2, log scale)")
    ax1.set_ylabel("windows")
    ax1.set_title("Energy per condition")
    ax1.legend(fontsize=8)

    # scatter: variance vs rhythm
    for num in sorted(data):
        if num not in LABELS:
            continue
        name, cat = LABELS[num]
        xs = [max(v, 1e-3) for v, _ in data[num]]
        ys = [r for _, r in data[num]]
        ax2.scatter(xs, ys, s=12, alpha=0.6, label="%d: %s" % (num, name),
                    color=CATEGORY_COLORS[cat])
    ax2.axvline(energy_gate, color="k", linestyle="--")
    ax2.axhline(rhythm_th, color="k", linestyle=":")
    ax2.set_xscale("log")
    ax2.set_xlabel("variance (log scale)")
    ax2.set_ylabel("rhythm (autocorr peak)")
    ax2.set_title("Decision plane — 'moving' = right of dashed AND above dotted")
    ax2.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig("calibration_plots.png", dpi=140)
    print("Saved calibration_plots.png")
    plt.show()

if __name__ == "__main__":
    main()
