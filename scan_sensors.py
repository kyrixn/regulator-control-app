#!/usr/bin/env python3
"""
scan_sensors.py

Read-only RS-485 check of the whole station: which sensor ids answer on which
EKU081 channel, and what rate each bus delivers. It never opens the Giga and
never sends a valve command. Stop app.py first: on Windows a COM port can be
open in only one program at a time.

Phase 1 scans the id range on every channel in parallel (each silent id costs
one timeout). Phase 2 then reads the found sensors back to back for --seconds
and reports reads, timeouts, errors, latency, resting jitter, encoder status
and sweep rate. The run fails (exit 1) on a duplicate or missing sensor, or on
any sensor whose phase-2 reads fail more often than --max-error-rate (0).

    python scan_sensors.py --expect 32
    python scan_sensors.py --ids 50-86,173-175 --seconds 60     # accept a bus
    python scan_sensors.py --ports COM20,COM16 --log runs/scan.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    list_ch9344_ports,
    parse_slave_ids,
)
from finger.rs485 import Reading, open_port, read_sensor
from sensor_mapping import DEFAULT_MAPPING_PATH, load_all


@dataclass
class PortResult:
    port: str
    error: Optional[str] = None
    found: List[int] = field(default_factory=list)
    scan_s: float = 0.0
    samples: List[Reading] = field(default_factory=list)          # phase 2
    latencies: Dict[int, List[float]] = field(default_factory=dict)
    sweeps: List[float] = field(default_factory=list)             # seconds per sweep


def run_port(port: str, ids: Sequence[int], baud: int, timeout: float,
             seconds: float, counts_per_turn: int) -> PortResult:
    """Phase 1 and 2 on one channel. Never raises on bus problems."""
    res = PortResult(port)
    try:
        ser = open_port(port, baud, timeout)
    except Exception as exc:
        res.error = str(exc)
        return res
    try:
        t0 = time.monotonic()
        for sid in ids:
            # Anything but a timeout means a device answered, even if garbled.
            if read_sensor(ser, sid, counts_per_turn).outcome != "timeout":
                res.found.append(sid)
        res.scan_s = time.monotonic() - t0

        t_end = time.monotonic() + seconds
        while res.found and time.monotonic() < t_end:
            t_sweep = time.monotonic()
            for sid in res.found:
                t1 = time.monotonic()
                rd = read_sensor(ser, sid, counts_per_turn)
                res.latencies.setdefault(sid, []).append(rd.t - t1)
                res.samples.append(rd)
            res.sweeps.append(time.monotonic() - t_sweep)
    finally:
        ser.close()
    return res


def id_owners(results: Sequence[PortResult]) -> Dict[int, List[str]]:
    """Sensor id -> the channels it answered on."""
    out: Dict[int, List[str]] = {}
    for r in results:
        for sid in r.found:
            out.setdefault(sid, []).append(r.port)
    return out


def check_ids(owners: Dict[int, List[str]], expected_ids: Sequence[int],
              expect_n: Optional[int]) -> List[str]:
    """Problems that make the exit status non-zero."""
    problems = [f"sensor {sid} answers on {' and '.join(ports)}"
                for sid, ports in sorted(owners.items()) if len(ports) > 1]
    if expect_n is not None and len(owners) != expect_n:
        problems.append(f"expected {expect_n} sensors, found {len(owners)}")
    missing = sorted(set(expected_ids) - set(owners))
    if missing:
        problems.append("mapped but silent: " + ", ".join(map(str, missing)))
    return problems


def sensor_stats(samples: Sequence[Reading], mm_per_count: float) -> Dict[int, dict]:
    """Per-sensor outcome counts, resting jitter (max - min counts) and the
    encoder status codes seen in good reads."""
    per: Dict[int, dict] = {}
    for rd in samples:
        s = per.setdefault(rd.slave, {"ok": 0, "timeout": 0, "error": 0, "counts": [],
                                      "status": set()})
        s[rd.outcome] += 1
        if rd.ok:
            s["counts"].append(rd.counts)
            s["status"].add(rd.status)
    for s in per.values():
        c = s.pop("counts")
        s["jitter_counts"] = (max(c) - min(c)) if c else None
        s["jitter_mm"] = None if s["jitter_counts"] is None else s["jitter_counts"] * mm_per_count
        s["status"] = sorted(s["status"])
    return per


def quality_problems(stats: Dict[int, dict], max_error_rate: float) -> List[str]:
    """Sensors whose phase-2 reads failed more often than allowed. Being found
    in phase 1 is not enough: a marginal bus answers most reads, not all."""
    out = []
    for sid, s in sorted(stats.items()):
        n = s["ok"] + s["timeout"] + s["error"]
        bad = s["timeout"] + s["error"]
        if n and bad / n > max_error_rate:
            out.append(f"sensor {sid}: {bad}/{n} reads failed "
                       f"({s['timeout']} timeout, {s['error']} CRC/protocol)")
    return out


def _port_list(value: str) -> List[str]:
    return [p.strip() for p in value.split(",") if p.strip()]


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Piped output on Windows is cp1252; replace what it cannot print.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    ap = argparse.ArgumentParser(
        description="Read-only scan of the station's RS-485 sensors (no valve I/O).")
    ap.add_argument("--ports", type=_port_list, default=None,
                    help="comma list of channels (default: every EKU081 channel)")
    ap.add_argument("--ids", type=parse_slave_ids, default=parse_slave_ids("1-247"),
                    help="ids to scan (default 1-247)")
    ap.add_argument("--expect", type=int, default=None,
                    help="number of sensors that should answer")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="phase-2 back-to-back reading time per channel (s); "
                         "use 60+ to accept a bus")
    ap.add_argument("--max-error-rate", type=float, default=0.0,
                    help="fraction of phase-2 reads per sensor allowed to time out "
                         "or fail CRC before the run fails (default 0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.06, help="per-read timeout (s)")
    ap.add_argument("--mapping", default=DEFAULT_MAPPING_PATH)
    ap.add_argument("--log", help="write every phase-2 sample to this CSV")
    args = ap.parse_args(argv)

    ports = args.ports or list_ch9344_ports()
    if not ports:
        print("[ERR] no EKU081 channels found; pass --ports", file=sys.stderr)
        return 2
    _, index = load_all(args.mapping)
    mm_per_count = math.pi * DEFAULT_DRUM_DIAMETER_MM / DEFAULT_COUNTS_PER_TURN

    print(f"channels {', '.join(ports)} @ {args.baud} 8N1, timeout {args.timeout}s")
    print(f"phase 1: {len(args.ids)} ids ({args.ids[0]}..{args.ids[-1]}), up to "
          f"{len(args.ids) * args.timeout:.0f} s; phase 2: {args.seconds:g} s")

    results: List[Optional[PortResult]] = [None] * len(ports)

    def work(i: int, port: str) -> None:
        try:
            results[i] = run_port(port, args.ids, args.baud, args.timeout, args.seconds,
                                  DEFAULT_COUNTS_PER_TURN)
        except Exception as exc:          # e.g. the adapter unplugged mid-scan
            results[i] = PortResult(port, error=f"{type(exc).__name__}: {exc}")

    threads = [threading.Thread(target=work, args=(i, p)) for i, p in enumerate(ports)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    def name(sid: int) -> str:
        return index[sid]["muscle"] if sid in index else "unmapped"

    print("\n== ids per channel ==")
    for r in results:
        if r.error:
            print(f"{r.port}: open failed: {r.error}")
            continue
        listing = ", ".join(f"{sid} {name(sid)}" for sid in r.found) or "-"
        print(f"{r.port}: {len(r.found)} in {r.scan_s:.1f} s -> {listing}")

    ms = lambda s: f"{s * 1e3:6.1f}"  # noqa: E731
    print("\n== bus timing (phase 2) ==")
    print(f"{'chan':6} {'id':>3} {'muscle':14} {'reads':>5} {'ok':>5} {'t/o':>4} {'err':>4} "
          f"{'lat med':>8} {'max':>7}  {'jitter':>6} {'= mm':>7}  status")
    all_stats: Dict[int, dict] = {}
    for r in results:
        if r.error or not r.found:
            continue
        stats = sensor_stats(r.samples, mm_per_count)
        all_stats.update(stats)
        for sid in r.found:
            s = stats.get(sid, {"ok": 0, "timeout": 0, "error": 0,
                                "jitter_counts": None, "jitter_mm": None, "status": []})
            lat = r.latencies.get(sid) or [0.0]
            n = s["ok"] + s["timeout"] + s["error"]
            jc = "-" if s["jitter_counts"] is None else s["jitter_counts"]
            jm = "-" if s["jitter_mm"] is None else f"{s['jitter_mm']:.4f}"
            print(f"{r.port:6} {sid:3d} {name(sid)[:14]:14} {n:5d} {s['ok']:5d} "
                  f"{s['timeout']:4d} {s['error']:4d} {ms(statistics.median(lat)):>8} "
                  f"{ms(max(lat)):>7}  {jc:>6} {jm:>7}  {s['status'] or '-'}")
        if r.sweeps:
            fails = sum(1 for rd in r.samples if not rd.ok)
            print(f"{r.port}: {len(r.sweeps)} sweeps of {len(r.found)} -> "
                  f"{len(r.sweeps) / sum(r.sweeps):.1f} Hz per sensor; sweep med "
                  f"{ms(statistics.median(r.sweeps))} ms, max {ms(max(r.sweeps))} ms; "
                  f"{fails} failed read(s)")

    if args.log:
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        with open(args.log, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["channel", "t_monotonic", "sensor", "muscle", "outcome",
                        "abs_counts", "status", "error_count"])
            for r in results:
                for rd in r.samples:
                    w.writerow([r.port, rd.t, rd.slave, name(rd.slave), rd.outcome,
                                rd.counts, rd.status, rd.error_count])
        print(f"\nlogged phase-2 samples to {args.log}")

    owners = id_owners(results)
    problems = (check_ids(owners, [s for s in index if s in args.ids], args.expect)
                + quality_problems(all_stats, args.max_error_rate))
    print(f"\n== result: {len(owners)} sensor(s) on {len(ports)} channel(s) ==")
    for p in problems:
        print(f"  [!] {p}")
    if not problems:
        print("  OK")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
