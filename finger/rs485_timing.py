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

import serial  # noqa: E402

from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM  # noqa: E402
from modbus_rtu import (  # noqa: E402
    GJW_STATE_COUNT,
    GJW_STATE_REGISTER,
    READ_FUNCTIONS,
    build_read_request,
    compute_absolute_position,
    decode_gjw_state_registers,
    parse_read_registers_response,
)
from sensor_mapping import DEFAULT_MAPPING_PATH, load_mapping  # noqa: E402


def read_once(ser: serial.Serial, slave: int) -> tuple[dict | None, str]:
    """One Modbus read. Returns (state, outcome) with outcome ok/timeout/error."""
    request = build_read_request(slave, READ_FUNCTIONS["holding"],
                                 GJW_STATE_REGISTER, GJW_STATE_COUNT)
    ser.reset_input_buffer()
    ser.write(request)
    header = ser.read(3)
    if len(header) != 3:
        return None, "timeout"
    body = ser.read(2 if header[1] & 0x80 else header[2] + 2)
    try:
        regs = parse_read_registers_response(header + body, slave,
                                             READ_FUNCTIONS["holding"], GJW_STATE_COUNT)
    except Exception:
        return None, "error"
    return decode_gjw_state_registers(regs), "ok"


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

    ser = serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1,
                        timeout=args.timeout)
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
            state, res = read_once(ser, sid)
            t1 = time.monotonic()
            outcome[sid][res] += 1
            lat[sid].append(t1 - t0)
            abs_pos = None
            if state:
                abs_pos = compute_absolute_position(state["single_turn_position"],
                                                    state["turns"], DEFAULT_COUNTS_PER_TURN)
                counts[sid].append(abs_pos)
                status[sid].add(state["status_code"])
            rows.append((t1, sid, name, res, abs_pos,
                         state["status_code"] if state else None,
                         state["error_count"] if state else None))
        sweeps.append(time.monotonic() - t_sweep)
    ser.close()

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
