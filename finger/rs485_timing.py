#!/usr/bin/env python3
"""
finger/rs485_timing.py

Read-only RS-485 acquisition check for the finger's five draw-wire sensors.
Stage 1 of the physical-control handoff: measure what the bus can actually
deliver before any control loop is designed around it.

It opens the encoder port, polls only the sensors named in sensor_mapping.json
back to back (no inter-cycle sleep), and reports per-sensor read latency,
timeouts, protocol errors, resting count jitter, and the achieved sweep rate.
No valve command is ever sent. Stop app.py first: it holds the same port.

    .venv/bin/python finger/rs485_timing.py --seconds 10
    .venv/bin/python finger/rs485_timing.py --port /dev/ttyCH9344USB0 --log runs/timing.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM  # noqa: E402
from finger.rs485 import Bus, open_port  # noqa: E402
from sensor_mapping import DEFAULT_MAPPING_PATH, load_mapping  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--port", default="/dev/ttyCH9344USB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.06, help="per-read serial timeout (s)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--mapping", default=DEFAULT_MAPPING_PATH)
    ap.add_argument("--log", help="write every sample to this CSV")
    args = ap.parse_args()

    mapping = load_mapping(args.mapping)
    sensors = [(m["sensor"], m["muscle"], reg) for reg, m in sorted(mapping.items())
               if m["sensor"] is not None]
    if not sensors:
        print("no sensors in mapping", file=sys.stderr)
        return 1
    print(f"port {args.port} @ {args.baud} 8N1, timeout {args.timeout}s")
    print("sensors:", ", ".join(f"{name}=id{sid} (V{reg})" for sid, name, reg in sensors))

    bus = Bus(open_port(args.port, args.baud, args.timeout), DEFAULT_COUNTS_PER_TURN)
    mm_per_count = math.pi * DEFAULT_DRUM_DIAMETER_MM / DEFAULT_COUNTS_PER_TURN

    lat = {sid: [] for sid, _, _ in sensors}
    outcome = {sid: {"ok": 0, "timeout": 0, "error": 0} for sid, _, _ in sensors}
    counts = {sid: [] for sid, _, _ in sensors}
    status = {sid: set() for sid, _, _ in sensors}
    sweeps = []
    rows = []

    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        t_sweep = time.monotonic()
        for sid, name, _ in sensors:
            t0 = time.monotonic()
            rd = bus.read(sid)
            outcome[sid][rd.outcome] += 1
            lat[sid].append(rd.t - t0)
            if rd.ok:
                counts[sid].append(rd.counts)
                status[sid].add(rd.status)
            rows.append((rd.t, sid, name, rd.outcome, rd.counts, rd.status, rd.error_count))
        sweeps.append(time.monotonic() - t_sweep)
    bus.close()

    if args.log:
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        with open(args.log, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_monotonic", "sensor", "muscle", "outcome", "abs_counts",
                        "status", "error_count"])
            w.writerows(rows)
        print(f"logged {len(rows)} samples to {args.log}")

    ms = lambda s: f"{s * 1e3:6.1f}"
    print(f"\n{'muscle':6} {'id':>3} {'reads':>5} {'ok':>5} {'t/o':>4} {'err':>4} "
          f"{'lat min':>8} {'med':>7} {'max':>7}  {'jitter counts':>13} {'= mm':>7}  status")
    for sid, name, _ in sensors:
        o = outcome[sid]
        n = sum(o.values())
        l = lat[sid]
        c = counts[sid]
        jitter = (max(c) - min(c)) if c else None
        print(f"{name:6} {sid:3d} {n:5d} {o['ok']:5d} {o['timeout']:4d} {o['error']:4d} "
              f"{ms(min(l)):>8} {ms(statistics.median(l)):>7} {ms(max(l)):>7}  "
              f"{(jitter if jitter is not None else '-'):>13} "
              f"{(f'{jitter * mm_per_count:.4f}' if jitter is not None else '-'):>7}  "
              f"{sorted(status[sid]) or '-'}")
    print(f"\nsweeps: {len(sweeps)} in {args.seconds:g}s -> "
          f"{len(sweeps) / args.seconds:.1f} Hz; "
          f"period med {ms(statistics.median(sweeps))} ms, max {ms(max(sweeps))} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
