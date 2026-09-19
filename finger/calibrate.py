#!/usr/bin/env python3
"""
finger/calibrate.py

Read-only calibration of the finger's five draw-wire sensors. Fills in the
encoder sign and the reference count for each muscle in finger.local.json so
the config becomes armable. It never sends a valve command: you move the
muscles by hand. Stop app.py first; it holds the RS-485 port.

Modes

  watch                 Live table of all five channels: counts, change since
                        the tool started (counts and mm), status, read health.
                        Pull each muscle in turn to confirm which role moves.
                        Ctrl-C to leave. Nothing is saved.

  reference [ROLE ...]  Put the finger at the maximum-extension pose
                        (MCP -9, PIP -8, DIP 0, deviation 0 deg), hold it still,
                        then run this. Samples every channel for --settle
                        seconds, refuses if a channel is offline or moved more
                        than --max-drift-mm, and stores the mean count as that
                        muscle's reference_counts. Roles default to all five.

  sign ROLE [ROLE ...]  For each role: capture a baseline, then you SHORTEN that
                        muscle by hand (pull the tendon toward the muscle) and
                        hold; press Enter; the tool captures again and records
                        sign = +1 if counts rose while shortening, -1 if they
                        fell. Needs at least --min-move-mm of travel.

  show                  Print the config and whether it is armable.

Examples

  .venv/bin/python finger/calibrate.py watch
  .venv/bin/python finger/calibrate.py reference
  .venv/bin/python finger/calibrate.py sign DI PI
  .venv/bin/python finger/calibrate.py sign ED FDS FDP --min-move-mm 1

Every save rewrites finger.local.json atomically; the other fields are kept.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import config as C  # noqa: E402
from finger.rs485 import Bus, Reading, open_port  # noqa: E402


# ----------------------------------------------------------------------------
# Pure helpers (unit-tested without hardware)
# ----------------------------------------------------------------------------

def decide_sign(delta_counts: int, min_counts: int) -> Optional[int]:
    """Sign such that contraction = sign * (counts - ref) is positive when the
    muscle shortens. `delta_counts` is counts_after - counts_before for a
    shortening move. None when the move is too small to trust."""
    if abs(delta_counts) < min_counts:
        return None
    return 1 if delta_counts > 0 else -1


def summarize(samples: Sequence[int]) -> Dict[str, float]:
    """mean / span / n of a list of counts."""
    return {"mean": statistics.fmean(samples), "span": max(samples) - min(samples),
            "n": len(samples)}


# ----------------------------------------------------------------------------
# Sampling
# ----------------------------------------------------------------------------

def sample(bus: Bus, cfg: C.FingerConfig, roles: Sequence[str], seconds: float
           ) -> Dict[str, List[int]]:
    """Poll the given roles back to back for `seconds`; ok readings only."""
    out: Dict[str, List[int]] = {r: [] for r in roles}
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        for r in roles:
            rd = bus.read(cfg.muscles[r].sensor)
            if rd.ok:
                out[r].append(rd.counts)
    return out


def capture(bus: Bus, cfg: C.FingerConfig, roles: Sequence[str], seconds: float,
            max_span_counts: int, label: str) -> Dict[str, Dict[str, float]]:
    """Sample and validate: every role online and quiet. Raises on failure."""
    data = sample(bus, cfg, roles, seconds)
    result, problems = {}, []
    for r in roles:
        s = data[r]
        if len(s) < 5:
            problems.append(f"{r} (id {cfg.muscles[r].sensor}): "
                            f"only {len(s)} good reads in {seconds:g}s")
            continue
        st = summarize(s)
        if st["span"] > max_span_counts:
            problems.append(f"{r}: moved {st['span'] * cfg.mm_per_count:.3f} mm during "
                            f"{label} (limit {max_span_counts * cfg.mm_per_count:.3f} mm); "
                            "hold it still")
        result[r] = st
    if problems:
        raise RuntimeError("\n  ".join([f"{label} capture rejected:"] + problems))
    return result


# ----------------------------------------------------------------------------
# Modes
# ----------------------------------------------------------------------------

def mode_watch(bus: Bus, cfg: C.FingerConfig, hz: float) -> int:
    start: Dict[str, Optional[int]] = {r: None for r in C.ROLES}
    fails: Dict[str, int] = {r: 0 for r in C.ROLES}
    total = 0
    period = 1.0 / hz
    print("Pull each muscle in turn; the role whose 'change' moves is the one on that "
          "sensor. Ctrl-C to leave.\n")
    try:
        while True:
            t0 = time.monotonic()
            rows = []
            total += 1
            for r in C.ROLES:
                m = cfg.muscles[r]
                rd = bus.read(m.sensor)
                if not rd.ok:
                    fails[r] += 1
                    rows.append(f"  {r:3} V{m.regulator:<2} id{m.sensor:<3}  {'-- ' + rd.outcome:>14}"
                                f"{'':>26}  fails {fails[r]}/{total}")
                    continue
                if start[r] is None:
                    start[r] = rd.counts
                d = rd.counts - start[r]
                mm = d * cfg.mm_per_count
                bar = "#" * min(30, int(abs(mm) * 10))
                rows.append(f"  {r:3} V{m.regulator:<2} id{m.sensor:<3}  {rd.counts:>12}  "
                            f"change {d:>+9} = {mm:>+8.3f} mm  st {rd.status:<2} "
                            f"fails {fails[r]}/{total}  {bar}")
            sys.stdout.write("\x1b[H\x1b[J")
            print(f"{cfg.rs485_port}  {cfg.mm_per_count * 1e3:.4f} um/count   "
                  f"(change is raw counts, no sign applied)\n")
            print("\n".join(rows))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nleft watch; nothing saved")
    return 0


def mode_reference(bus: Bus, cfg: C.FingerConfig, roles: Sequence[str], settle: float,
                   max_drift_mm: float, yes: bool) -> int:
    print("Reference capture at the maximum-extension pose "
          "(MCP -9, PIP -8, DIP 0, deviation 0 deg).")
    print(f"Roles: {', '.join(roles)}. Hold the finger still for {settle:g} s.")
    if not yes:
        input("Press Enter when the finger is in the reference pose... ")
    max_span = int(max_drift_mm / cfg.mm_per_count)
    try:
        st = capture(bus, cfg, roles, settle, max_span, "reference")
    except RuntimeError as exc:
        print(f"[ERR] {exc}")
        return 1
    for r in roles:
        ref = int(round(st[r]["mean"]))
        old = cfg.muscles[r].reference_counts
        cfg.muscles[r].reference_counts = ref
        print(f"  {r:3} reference {ref:>12}  (span {st[r]['span'] * cfg.mm_per_count:.4f} mm, "
              f"{int(st[r]['n'])} reads){'' if old is None else f'  was {old}'}")
    C.save(cfg)
    print(f"saved {cfg.path}")
    return 0


def mode_sign(bus: Bus, cfg: C.FingerConfig, roles: Sequence[str], settle: float,
              max_drift_mm: float, min_move_mm: float) -> int:
    max_span = int(max_drift_mm / cfg.mm_per_count)
    min_counts = int(min_move_mm / cfg.mm_per_count)
    for r in roles:
        m = cfg.muscles[r]
        print(f"\n=== {r} (regulator {m.regulator}, sensor {m.sensor}) ===")
        input("Leave the muscle at rest and press Enter to capture the baseline... ")
        try:
            before = capture(bus, cfg, [r], settle, max_span, "baseline")[r]
        except RuntimeError as exc:
            print(f"[ERR] {exc}\nskipping {r}")
            continue
        input(f"Now SHORTEN {r} by hand (pull the tendon toward the muscle, at least "
              f"{min_move_mm:g} mm), hold it there, and press Enter... ")
        try:
            after = capture(bus, cfg, [r], settle, max_span, "shortened")[r]
        except RuntimeError as exc:
            print(f"[ERR] {exc}\nskipping {r}")
            continue
        delta = int(round(after["mean"] - before["mean"]))
        sign = decide_sign(delta, min_counts)
        moved_mm = delta * cfg.mm_per_count
        if sign is None:
            print(f"[ERR] {r}: counts changed by {delta} ({moved_mm:+.3f} mm), less than "
                  f"{min_move_mm:g} mm; not recorded. Move it further and retry.")
            continue
        old = m.sign
        m.sign = sign
        print(f"  {r}: shortening moved counts by {delta:+d} ({moved_mm:+.3f} mm) -> sign {sign:+d}"
              f"{'' if old is None else f'  (was {old:+d})'}")
        C.save(cfg)
        print(f"  saved {cfg.path}")
    problems = cfg.arm_problems()
    print("\n" + ("config is now ARMABLE" if not problems
                  else "still not armable:\n  " + "\n  ".join(problems)))
    return 0


def mode_show(cfg: C.FingerConfig) -> int:
    print(f"{cfg.path} + {cfg.mapping_path}")
    print(f"valve {cfg.valve_port}, rs485 {cfg.rs485_port} @ {cfg.baud}")
    print(f"{cfg.mm_per_count * 1e3:.5f} um/count ({cfg.drum_diameter_mm} mm drum)")
    for r in C.ROLES:
        m = cfg.muscles[r]
        print(f"  {r:3} V{m.regulator:<2} id{m.sensor:<3} idle {m.idle_kpa:g} cap {m.ceiling_kpa:g} kPa "
              f"sign {m.sign} ref {m.reference_counts}")
    problems = cfg.arm_problems()
    print("ARMABLE" if not problems else "NOT ARMABLE:\n  " + "\n  ".join(problems))
    return 0


# ----------------------------------------------------------------------------

def parse_roles(values: Sequence[str]) -> List[str]:
    roles = [v.upper() for v in values]
    bad = [r for r in roles if r not in C.ROLES]
    if bad:
        raise SystemExit(f"unknown role(s) {bad}; choose from {list(C.ROLES)}")
    return list(dict.fromkeys(roles))  # dedupe, keep order


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Read-only sensor calibration for the finger.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Modes", 1)[1])
    ap.add_argument("mode", choices=["watch", "reference", "sign", "show"])
    ap.add_argument("roles", nargs="*", help="muscle roles (ED FDS FDP DI PI)")
    ap.add_argument("--config", default=C.DEFAULT_CONFIG_PATH)
    ap.add_argument("--mapping", default=C.DEFAULT_MAPPING_PATH,
                    help="sensor_mapping.json supplying regulator/sensor per role")
    ap.add_argument("--port", help="override the RS-485 port from the config")
    ap.add_argument("--settle", type=float, default=2.0, help="capture window (s)")
    ap.add_argument("--max-drift-mm", type=float, default=0.05,
                    help="reject a capture if a channel moves more than this")
    ap.add_argument("--min-move-mm", type=float, default=0.5,
                    help="sign: minimum travel to trust the direction")
    ap.add_argument("--hz", type=float, default=5.0, help="watch refresh rate")
    ap.add_argument("-y", "--yes", action="store_true", help="reference: skip the prompt")
    args = ap.parse_args(argv)

    try:
        cfg = C.load(args.config, args.mapping)
    except C.ConfigError as exc:
        print(f"[ERR] {exc}")
        return 1
    if args.mode == "show":
        return mode_show(cfg)

    roles = parse_roles(args.roles) if args.roles else list(C.ROLES)
    if args.mode == "sign" and not args.roles:
        ap.error("sign needs at least one role, e.g. sign DI PI")

    port = args.port or cfg.rs485_port
    try:
        bus = Bus(open_port(port, cfg.baud), cfg.counts_per_turn)
    except Exception as exc:
        print(f"[ERR] cannot open {port}: {exc}\n(is app.py still running?)")
        return 1
    try:
        if args.mode == "watch":
            return mode_watch(bus, cfg, args.hz)
        if args.mode == "reference":
            return mode_reference(bus, cfg, roles, args.settle, args.max_drift_mm, args.yes)
        return mode_sign(bus, cfg, roles, args.settle, args.max_drift_mm, args.min_move_mm)
    except KeyboardInterrupt:
        print("\naborted; last saved state is kept")
        return 130
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
