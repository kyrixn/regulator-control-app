#!/usr/bin/env python3
"""
experiments/pair_sweep.py

Friction dead band and settling of one antagonist pair (spec:
docs/superpowers/specs/2026-10-01-pair-sweep-design.md). With the pair's sum
fixed at 2P, the differential d sweeps 0 -> +A -> 0 -> -A -> 0 each cycle,
holding at every turn, while every length and command (and optionally the
Giga's pressure readback) is logged. Other muscles are held at fixed pressures.

Dry by default: prints the plan, checks the sensors for a few seconds, sends
nothing. --arm runs it. Stop app.py first (it holds the ports).

    python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30
    python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30 --arm --adc-hz 10

Then: python experiments/analyze_sweep.py runs/<run>.csv
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensor_mapping import DEFAULT_MAPPING_PATH, muscles_by_name  # noqa: E402
from station import Abort, Recorder, Station  # noqa: E402


@dataclass
class Segment:
    kind: str                     # "ramp" | "hold"
    targets: Dict[str, float]     # muscle -> kPa at the end of the segment
    rate: float = 0.0             # kPa/s for ramps
    seconds: float = 0.0          # for holds
    label: str = ""


def pair_sweep_segments(a: str, b: str, pretension: float, amplitude: float, rate: float,
                        cycles: int, holds: Dict[str, float], approach_rate: float = 5.0,
                        settle_s: float = 5.0, turn_hold_s: float = 2.0,
                        ref_hold_s: float = 3.0, pre_hold_s: float = 0.0,
                        turn_plus: Tuple[float, float] = None,
                        turn_minus: Tuple[float, float] = None) -> List[Segment]:
    """The whole run as ramps and holds around the base (a = b = P). By default
    the turns are a = P + A, b = P - A and the reverse, so the pair's sum stays
    2P; turn_plus / turn_minus = (a kPa, b kPa) set a turn explicitly instead.
    A pre-hold keeps the base pressures for pre_hold_s before the first cycle
    (creep at constant pressure, before any cycling)."""
    if rate <= 0 or cycles < 1:
        raise ValueError("rate must be > 0 and cycles >= 1")
    P, A = pretension, amplitude
    turn_plus = turn_plus or (P + A, P - A)
    turn_minus = turn_minus or (P - A, P + A)
    for name, (pa, pb) in (("+ turn", turn_plus), ("- turn", turn_minus)):
        if min(pa, pb) < 0:
            raise ValueError(f"{name} {pa:g}/{pb:g} kPa: pressures must be >= 0 "
                             "(amplitude above the pretension?)")
    if not turn_plus[0] - turn_plus[1] > 0 > turn_minus[0] - turn_minus[1]:
        raise ValueError("the + turn must pull a harder than b, the - turn b harder than a")
    base = {a: P, b: P, **holds}
    plus = {**base, a: turn_plus[0], b: turn_plus[1]}
    minus = {**base, a: turn_minus[0], b: turn_minus[1]}
    segs = [Segment("ramp", base, approach_rate, label="inflate"),
            Segment("hold", base, seconds=settle_s, label="settle")]
    if pre_hold_s > 0:
        segs.append(Segment("hold", base, seconds=pre_hold_s, label="pre-hold"))
    for k in range(1, cycles + 1):
        segs += [
            Segment("ramp", plus, rate, label=f"c{k} to +"),
            Segment("hold", plus, seconds=turn_hold_s, label=f"c{k} hold +"),
            Segment("ramp", base, rate, label=f"c{k} + to 0"),
            Segment("hold", base, seconds=turn_hold_s, label=f"c{k} zero from +"),
            Segment("ramp", minus, rate, label=f"c{k} to -"),
            Segment("hold", minus, seconds=turn_hold_s, label=f"c{k} hold -"),
            Segment("ramp", base, rate, label=f"c{k} - to 0"),
            Segment("hold", base, seconds=ref_hold_s, label=f"c{k} zero from -"),
        ]
    segs.append(Segment("ramp", {m: 0.0 for m in base}, approach_rate, label="vent"))
    return segs


def plan_summary(segments: List[Segment], start: Dict[str, float], dither=None
                 ) -> Tuple[float, Dict[str, Tuple[float, float]]]:
    """(total seconds, {muscle: (min kPa, max kPa)}) of a segment list. With
    dither = (amp, names), the cycle segments' range of those muscles widens by
    amp (the dither only runs during the cycles)."""
    current = dict(start)
    names = {n for s in segments for n in s.targets}
    ranges = {n: (current.get(n, 0.0), current.get(n, 0.0)) for n in names}
    total = 0.0
    for s in segments:
        if s.kind == "ramp":
            total += max(abs(v - current.get(n, 0.0)) for n, v in s.targets.items()) / s.rate
        else:
            total += s.seconds
        for n, v in s.targets.items():
            widen = dither[0] if dither and n in dither[1] and s.label.startswith("c") else 0.0
            lo, hi = ranges[n]
            ranges[n] = (min(lo, v - widen), max(hi, v + widen))
        current.update(s.targets)
    return total, ranges


def cycle_min(segments: List[Segment], names) -> float:
    """Lowest setpoint of the named muscles during the cycles (dither excluded)."""
    return min(v for s in segments if s.label.startswith("c")
               for n, v in s.targets.items() if n in names)


def parse_turn(value: str) -> Tuple[float, float]:
    """'60,10' -> (60.0, 10.0): the pair's (a, b) pressures at a turn."""
    parts = [float(v) for v in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected two pressures, a,b")
    return parts[0], parts[1]


def parse_holds(value: str) -> Dict[str, float]:
    """'FCU=30,ECU=30' -> {'FCU': 30.0, 'ECU': 30.0}."""
    out = {}
    for part in filter(None, (p.strip() for p in value.split(","))):
        name, _, kpa = part.partition("=")
        out[name.strip()] = float(kpa)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Friction and settling sweep of one antagonist pair.")
    ap.add_argument("--pair", required=True, help="two muscles, e.g. FCR,ECR (a goes up first)")
    ap.add_argument("--pretension", type=float, default=30.0, help="P, kPa (default 30)")
    ap.add_argument("--amplitude", type=float, default=15.0, help="A, kPa, at most P (default 15)")
    ap.add_argument("--turn-plus", type=parse_turn, default=None,
                    help="a,b kPa at the + turn instead of P+A,P-A (e.g. 60,10)")
    ap.add_argument("--turn-minus", type=parse_turn, default=None,
                    help="a,b kPa at the - turn instead of P-A,P+A (e.g. 10,50)")
    ap.add_argument("--rate", type=float, default=1.0, help="sweep rate, kPa/s (default 1)")
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--pre-hold", type=float, default=0.0,
                    help="seconds at the base pressures before the first cycle (creep test)")
    ap.add_argument("--dither", type=float, default=0.0,
                    help="dither amplitude, kPa, on the pair's differential during the cycles")
    ap.add_argument("--dither-hz", type=float, default=2.0, help="dither frequency (default 2)")
    ap.add_argument("--hold", type=parse_holds, default={},
                    help="other muscles held during the run, e.g. FCU=30,ECU=30")
    ap.add_argument("--max-kpa", type=float, default=50.0, help="cap for this run (default 50)")
    ap.add_argument("--adc-hz", type=float, default=0.0,
                    help="poll the Giga's pressure readback at this rate (0 = off)")
    ap.add_argument("--arm", action="store_true", help="actually send pressures")
    ap.add_argument("--out", default="runs", help="output folder (default runs/)")
    ap.add_argument("--mapping", default=DEFAULT_MAPPING_PATH)
    ap.add_argument("--valve-port", default=None)
    ap.add_argument("--rs485-ports", default=None, help="comma list (default: every EKU081 channel)")
    args = ap.parse_args(argv)

    names = [n.strip() for n in args.pair.split(",")]
    if len(names) != 2:
        ap.error("--pair needs exactly two muscles")
    a, b = names
    by_name = muscles_by_name(args.mapping)
    for n in (a, b):
        m = by_name.get(n)
        if m is None or m["regulator"] is None or m["sensor"] is None:
            ap.error(f"{n}: needs a regulator and a sensor in the mapping")
    for n in args.hold:
        if n not in by_name or by_name[n]["regulator"] is None:
            ap.error(f"{n}: no regulator in the mapping")

    segs = pair_sweep_segments(a, b, args.pretension, args.amplitude, args.rate, args.cycles,
                               args.hold, pre_hold_s=args.pre_hold,
                               turn_plus=args.turn_plus, turn_minus=args.turn_minus)
    dither = (args.dither, (a, b)) if args.dither > 0 else None
    total, ranges = plan_summary(segs, {}, dither=dither)
    top = max(hi for _, hi in ranges.values())
    if top > args.max_kpa:
        ap.error(f"the plan reaches {top:g} kPa, above --max-kpa {args.max_kpa:g}")
    if dither and cycle_min(segs, (a, b)) - args.dither < 0:
        ap.error("the dither would take a pressure below 0 kPa; lower --dither or --amplitude")

    involved = [a, b, *args.hold]
    muscles = {n: by_name[n] for n in involved}
    tp = next(s.targets for s in segs if s.label == "c1 hold +")
    tm = next(s.targets for s in segs if s.label == "c1 hold -")
    print(f"pair {a} (V{muscles[a]['regulator']}, s{muscles[a]['sensor']}) vs "
          f"{b} (V{muscles[b]['regulator']}, s{muscles[b]['sensor']}): base {args.pretension:g}/"
          f"{args.pretension:g}, turns {tp[a]:g}/{tp[b]:g} and {tm[a]:g}/{tm[b]:g} kPa, "
          f"{args.rate:g} kPa/s, "
          f"{args.cycles} cycles, {total / 60:.1f} min"
          + (f", pre-hold {args.pre_hold:g} s" if args.pre_hold else "")
          + (f", dither +/-{args.dither:g} kPa at {args.dither_hz:g} Hz" if dither else ""))
    for n in involved:
        lo, hi = ranges[n]
        print(f"  {n:4} V{muscles[n]['regulator']:<2} {lo:5.1f} .. {hi:5.1f} kPa")

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(args.out, f"{stamp}_{a}-{b}_P{args.pretension:g}"
                                  f"{'' if args.arm else '_dry'}")
    with open(base + ".json", "w") as fh:
        json.dump({"pair": [a, b], "pretension": args.pretension, "amplitude": args.amplitude,
                   "turn_plus": [tp[a], tp[b]], "turn_minus": [tm[a], tm[b]],
                   "rate": args.rate, "cycles": args.cycles, "hold": args.hold,
                   "pre_hold": args.pre_hold,
                   "dither": ({"amp": args.dither, "hz": args.dither_hz} if dither else None),
                   "muscles": muscles, "armed": args.arm, "adc_hz": args.adc_hz,
                   "segments": [asdict(s) for s in segs]}, fh, indent=1)

    rec = Recorder(base + ".csv")
    ports = [p.strip() for p in args.rs485_ports.split(",")] if args.rs485_ports else None
    try:
        with Station(rec, muscles, armed=args.arm, max_kpa=args.max_kpa,
                     valve_port=args.valve_port, rs485_ports=ports,
                     adc_hz=args.adc_hz) as st:
            missing = st.wait_for_sensors(timeout=10.0)
            if missing:
                print(f"[ERR] no fresh reading from sensor(s) {missing}; nothing sent")
                return 1
            print("sensors:", ", ".join(f"{n} {st.counts()[muscles[n]['sensor']]}"
                                        for n in involved if muscles[n]["sensor"]))
            if not args.arm:
                time.sleep(3.0)
                print(f"DRY RUN: nothing sent. Add --arm to run it ({total / 60:.1f} min).")
                return 0
            if not st.ping():
                print("[ERR] the Giga does not answer a ping (touchscreen in standalone mode, "
                      "or not responding); nothing sent")
                return 1
            others = st.sync_commanded()
            if others:
                print(f"[WARN] regulators outside this run are on and stay as they are: {others}")
            t0 = time.monotonic()
            for i, seg in enumerate(segs, 1):
                if dither and seg.label == "c1 to +":
                    st.set_dither(args.dither, args.dither_hz, {a: +1, b: -1})
                if dither and seg.label == "vent":
                    st.clear_dither()
                st.mark(seg.label)
                print(f"[{time.monotonic() - t0:6.1f} s] {i}/{len(segs)} {seg.label}", flush=True)
                if seg.kind == "ramp":
                    st.ramp(seg.targets, seg.rate)
                else:
                    st.hold(seg.seconds)
            st.off()
            st.mark("done")
    except Abort as exc:
        print(f"[ABORT] {exc}: stop sent, every valve vented")
        return 1
    except KeyboardInterrupt:
        print("\n[STOP] interrupted: stop sent, every valve vented")
        return 1
    finally:
        rec.close()
    print(f"logged to {base}.csv\nanalyse: python experiments/analyze_sweep.py {base}.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
