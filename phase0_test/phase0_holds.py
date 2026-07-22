#!/usr/bin/env python3
"""
phase0_holds.py

From a Phase0test CSV, extract the SETTLED length of each sensor at the two
extreme positions (the A-hold and B-hold dwells) for every cycle, and check
whether those settled positions are consistent as the ramp duration is swept
across the run.

Rationale
---------
Phase0test now sweeps the ramp duration linearly from --min-ramp to --max-ramp
over the cycles. If the muscle/sensor settles to the same place regardless of
how fast it got there, the settled A-hold and B-hold lengths should be flat
versus ramp duration (a horizontal line). A slope or large spread means the
settled extreme depends on ramp rate (rate-dependence / creep) — i.e. NOT
consistent.

For each cycle and each extreme, the settled value is the mean length over the
LATTER part of that hold dwell (default last 50 %), so the ramp/settling
transient is excluded.

Two subplots (one per sensor): settled length at the A and B extremes vs the
cycle's ramp duration, with per-series mean±σ, peak-to-peak, and slope
(mm per second of ramp) reported. A PNG is saved next to the CSV.

Usage (runnable from any working directory):
    python phase0_test/phase0_holds.py               # newest CSV in phase0_data/
    python phase0_test/phase0_holds.py path/to/run.csv
    python phase0_test/phase0_holds.py --settle-frac 0.5 --no-show
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


DEFAULT_DIR = os.path.join(os.path.dirname(__file__), "phase0_data")

# (sensor id, label, position column)
SENSORS = [
    (57, "R57", "pos_R57_mm"),
    (58, "R58", "pos_R58_mm"),
]
# (hold segment, short label, marker, colour)
EXTREMES = [
    ("A_hold", "A extreme", "o", "tab:red"),
    ("B_hold", "B extreme", "s", "tab:blue"),
]


def latest_csv(directory):
    files = sorted(glob.glob(os.path.join(directory, "phase0test_*.csv")))
    return files[-1] if files else None


def _to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _to_int(v):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return None


def load_rows(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def settled_value(rows, cycle, segment, pos_col, settle_frac):
    """Mean/std length over the last `settle_frac` of one hold dwell."""
    seq = [(_to_float(r.get("elapsed_s")), _to_float(r.get(pos_col)))
           for r in rows
           if _to_int(r.get("cycle")) == cycle and r.get("segment") == segment]
    seq = [(t, y) for t, y in seq if t is not None and y is not None]
    if len(seq) < 2:
        return None, None
    seq.sort()
    start = int(len(seq) * (1.0 - settle_frac))
    ys = np.array([y for _, y in seq[start:]])
    if ys.size == 0:
        return None, None
    return float(ys.mean()), float(ys.std(ddof=1) if ys.size > 1 else 0.0)


def cycle_ramp(rows, cycle):
    for r in rows:
        if _to_int(r.get("cycle")) == cycle:
            rv = _to_float(r.get("ramp_s"))
            if rv is not None:
                return rv
    return None


def plot_sensor(ax, rows, sid, label, pos_col, cycles, settle_frac, x_is_ramp):
    lines = []
    for segment, seg_label, marker, colour in EXTREMES:
        xs, ys, es, cs = [], [], [], []
        for c in cycles:
            mean, std = settled_value(rows, c, segment, pos_col, settle_frac)
            if mean is None:
                continue
            x = cycle_ramp(rows, c) if x_is_ramp else c
            if x is None:
                continue
            xs.append(x); ys.append(mean); es.append(std or 0.0); cs.append(c)
        if not xs:
            continue
        order = np.argsort(xs)
        xs = np.array(xs)[order]; ys = np.array(ys)[order]
        es = np.array(es)[order]; cs = np.array(cs)[order]
        ax.errorbar(xs, ys, yerr=es, fmt=marker + "-", color=colour,
                    capsize=3, lw=1.2, ms=5, label=seg_label)

        mean_y = float(ys.mean())
        sd = float(ys.std(ddof=1)) if ys.size > 1 else 0.0
        p2p = float(ys.max() - ys.min())
        slope = float(np.polyfit(xs, ys, 1)[0]) if ys.size > 1 else 0.0
        lines.append({"segment": segment, "label": seg_label, "mean": mean_y,
                      "sd": sd, "p2p": p2p, "slope": slope, "n": ys.size})

    ax.set_xlabel("ramp duration (s)" if x_is_ramp else "cycle")
    ax.set_ylabel("settled length (mm)")
    subtitle = "\n".join(
        f"{ln['segment'][0]}: {ln['mean']:.3f}±{ln['sd']:.3f} mm  "
        f"p2p {ln['p2p']:.3f}  slope {ln['slope']:+.3f} mm/s"
        for ln in lines
    )
    ax.set_title(f"Sensor {sid} ({label}) settled extremes\n{subtitle}", fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description="Phase0test settled-extreme consistency vs ramp")
    parser.add_argument("csv", nargs="?", default=None,
                        help="CSV path (default: newest in phase0_data/)")
    parser.add_argument("--dir", default=DEFAULT_DIR, help="dir to search for the newest CSV")
    parser.add_argument("--settle-frac", type=float, default=0.5,
                        help="fraction of each hold (from the end) averaged as settled value")
    parser.add_argument("--vs-cycle", action="store_true",
                        help="plot vs cycle index instead of ramp duration")
    parser.add_argument("--no-show", action="store_true", help="save the PNG without displaying")
    args = parser.parse_args(argv)

    path = args.csv or latest_csv(args.dir)
    if not path or not os.path.exists(path):
        print(f"[ERR] no CSV found (looked in {args.dir}). "
              "Run Phase0test.py first or pass a path.")
        return 2
    print(f"Reading {os.path.abspath(path)}")

    rows = load_rows(path)
    if not rows:
        print("[ERR] CSV has no data rows.")
        return 1

    # Include every real cycle (>=1) so the fastest ramp (cycle 1) is kept;
    # only the cycle-0 zero-hold pre-samples are skipped.
    cycles = sorted({_to_int(r.get("cycle")) for r in rows
                     if _to_int(r.get("cycle")) is not None
                     and _to_int(r.get("cycle")) >= 1})
    have_ramp = any(_to_float(r.get("ramp_s")) is not None for r in rows)
    x_is_ramp = have_ramp and not args.vs_cycle
    if not have_ramp and not args.vs_cycle:
        print("[WARN] no ramp_s column in CSV; plotting vs cycle index instead.")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), constrained_layout=True)
    for ax, (sid, label, pos_col) in zip(axes, SENSORS):
        lines = plot_sensor(ax, rows, sid, label, pos_col, cycles,
                            args.settle_frac, x_is_ramp)
        print(f"  sensor {sid} ({label}):")
        for ln in lines:
            verdict = ("consistent" if ln["p2p"] < 0.10
                       else "check — varies with ramp")
            print(f"      {ln['label']:<9} settled = {ln['mean']:.3f} mm  "
                  f"σ={ln['sd']:.3f}  p2p={ln['p2p']:.3f} mm  "
                  f"slope={ln['slope']:+.3f} mm/s  [{ln['n']} pts]  → {verdict}")

    xdesc = "ramp duration" if x_is_ramp else "cycle"
    fig.suptitle(f"Phase 0 settled extremes vs {xdesc} — {os.path.basename(path)}",
                 fontsize=12)

    out_png = os.path.splitext(path)[0] + "_holds.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print(f"Saved plot to {os.path.abspath(out_png)}")

    if not args.no_show:
        try:
            plt.show()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
