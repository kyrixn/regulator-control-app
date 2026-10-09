#!/usr/bin/env python3
"""
experiments/pair_sweep.py

Friction dead band and settling of one antagonist pair (spec:
docs/superpowers/specs/2026-10-01-pair-sweep-design.md). With the pair's sum
fixed at 2P, the differential d sweeps 0 -> +A -> 0 -> -A -> 0 each cycle,
holding at every turn, while every length and command (and optionally the
Giga's pressure readback) is logged. Other muscles are held at fixed pressures.

Line mode sweeps a group of muscles together instead: each muscle's pressure
is linear in one coordinate s, given as its pressures at s = -1 and s = +1
(e.g. measured end states of the wrist), and s sweeps
centre -> centre+a -> centre -> centre-a -> centre, with a stepped through
--cycles-at. The largest change runs at --rate kPa/s and every muscle arrives
at its turn together, so the pressures stay on the line.

Dry by default: prints the plan, checks the sensors for a few seconds, sends
nothing. --arm runs it. --plan only prints the plan and opens no port. Stop
app.py first (it holds the ports).

    python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30
    python experiments/pair_sweep.py --pair FCR,ECR --hold FCU=30,ECU=30 --arm --adc-hz 10
    python experiments/pair_sweep.py --line FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10 \
        --cycles-at 0.6:6,0.4:3,0.6:3,0.8:3 --hold Sup=60 --max-kpa 70 --mass-g 0

Then: python experiments/analyze_sweep.py runs/<run>.csv
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensor_mapping import DEFAULT_MAPPING_PATH, MAX_REGULATOR, muscles_by_name  # noqa: E402
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
    return _sweep(base, [(plus, minus)] * cycles, rate, approach_rate, settle_s,
                  turn_hold_s, ref_hold_s, pre_hold_s)


def _sweep(base: Dict[str, float], turns: List[Tuple[Dict[str, float], Dict[str, float]]],
           rate: float, approach_rate: float, settle_s: float, turn_hold_s: float,
           ref_hold_s: float, pre_hold_s: float) -> List[Segment]:
    """Inflate to the base, settle, optional pre-hold, one cycle per (plus,
    minus) target pair (base -> plus -> base -> minus -> base, holding at each),
    then vent."""
    segs = [Segment("ramp", base, approach_rate, label="inflate"),
            Segment("hold", base, seconds=settle_s, label="settle")]
    if pre_hold_s > 0:
        segs.append(Segment("hold", base, seconds=pre_hold_s, label="pre-hold"))
    for k, (plus, minus) in enumerate(turns, 1):
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


def line_pressures(line: Dict[str, Tuple[float, float]], s: float) -> Dict[str, float]:
    """Each muscle's kPa at sweep coordinate s, linear between its pressures at
    s = -1 and s = +1."""
    return {n: round(lo + (hi - lo) * (s + 1.0) / 2.0, 3) for n, (lo, hi) in line.items()}


def schedule_turns(schedule: List[Tuple[float, int]], centre: float = 0.0
                   ) -> List[Tuple[float, float]]:
    """[(a, n), ...] -> one (s at the - turn, s at the + turn) per cycle."""
    return [(centre - a, centre + a) for a, n in schedule for _ in range(n)]


def line_sweep_segments(line: Dict[str, Tuple[float, float]],
                        schedule: List[Tuple[float, int]], rate: float,
                        holds: Dict[str, float], centre: float = 0.0,
                        approach_rate: float = 5.0, settle_s: float = 5.0,
                        turn_hold_s: float = 2.0, ref_hold_s: float = 3.0,
                        pre_hold_s: float = 0.0) -> List[Segment]:
    """The whole line-mode run: s sweeps centre -> centre + a -> centre ->
    centre - a -> centre for every (a, cycles) block of the schedule, the held
    muscles staying at their pressures throughout."""
    if rate <= 0:
        raise ValueError("rate must be > 0")
    if not schedule or any(a <= 0 or n < 1 for a, n in schedule):
        raise ValueError("every block needs an amplitude > 0 and at least 1 cycle")
    both = set(line) & set(holds)
    if both:
        raise ValueError(f"{', '.join(sorted(both))}: both swept and held")
    base = {**line_pressures(line, centre), **holds}
    turns = []
    for s_minus, s_plus in schedule_turns(schedule, centre):
        plus = {**base, **line_pressures(line, s_plus)}
        minus = {**base, **line_pressures(line, s_minus)}
        for s, targets in ((s_plus, plus), (s_minus, minus)):
            low = {n: v for n, v in targets.items() if v < 0}
            if low:
                raise ValueError(f"at s = {s:g}: {low} below 0 kPa")
        turns.append((plus, minus))
    return _sweep(base, turns, rate, approach_rate, settle_s, turn_hold_s, ref_hold_s,
                  pre_hold_s)


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


def parse_line(value: str) -> Dict[str, Tuple[float, float]]:
    """'FCR=20:60,ECR=30:10' -> {'FCR': (20.0, 60.0), 'ECR': (30.0, 10.0)}: each
    muscle's kPa at s = -1 and at s = +1."""
    out = {}
    for part in filter(None, (p.strip() for p in value.split(","))):
        name, _, ends = part.partition("=")
        lo, sep, hi = ends.partition(":")
        if not sep or not name.strip():
            raise argparse.ArgumentTypeError(f"'{part}': expected NAME=kPa_at_-1:kPa_at_+1")
        try:
            out[name.strip()] = (float(lo), float(hi))
        except ValueError:
            raise argparse.ArgumentTypeError(
                f"'{part}': expected NAME=kPa_at_-1:kPa_at_+1") from None
    if len(out) < 2:
        raise argparse.ArgumentTypeError("a line needs at least two muscles")
    return out


def parse_schedule(value: str) -> List[Tuple[float, int]]:
    """'0.6:6,0.4:3' -> [(0.6, 6), (0.4, 3)]: amplitude in s, then cycles."""
    out = []
    for part in filter(None, (p.strip() for p in value.split(","))):
        a, _, n = part.partition(":")
        try:
            out.append((float(a), int(n)))
        except ValueError:
            raise argparse.ArgumentTypeError(f"'{part}': expected amplitude:cycles") from None
    if not out:
        raise argparse.ArgumentTypeError("expected amplitude:cycles[,...]")
    return out


def run_tag(condition: Dict[str, object]) -> str:
    """'_0g_palm_d1_m1' from the condition fields that are set (filename-safe)."""
    parts = []
    if condition.get("mass_g") is not None:
        parts.append(f"{condition['mass_g']:g}g")
    for key in ("station", "session", "mount"):
        if condition.get(key):
            parts.append(re.sub(r"[^A-Za-z0-9.-]+", "-", str(condition[key])))
    return "".join("_" + p for p in parts)


def parse_holds(value: str) -> Dict[str, float]:
    """'FCU=30,ECU=30' -> {'FCU': 30.0, 'ECU': 30.0}."""
    out = {}
    for part in filter(None, (p.strip() for p in value.split(","))):
        name, _, kpa = part.partition("=")
        out[name.strip()] = float(kpa)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Friction and settling sweep of one antagonist pair, or of a muscle "
                    "group along a line.")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--pair", help="two muscles, e.g. FCR,ECR (a goes up first)")
    mode.add_argument("--line", type=parse_line, default=None,
                      help="muscles swept together, each as NAME=kPa at s=-1:kPa at s=+1, "
                           "e.g. FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10")
    ap.add_argument("--pretension", type=float, default=30.0, help="pair: P, kPa (default 30)")
    ap.add_argument("--amplitude", type=float, default=15.0,
                    help="pair: A, kPa, at most P (default 15)")
    ap.add_argument("--turn-plus", type=parse_turn, default=None,
                    help="pair: a,b kPa at the + turn instead of P+A,P-A (e.g. 60,10)")
    ap.add_argument("--turn-minus", type=parse_turn, default=None,
                    help="pair: a,b kPa at the - turn instead of P-A,P+A (e.g. 10,50)")
    ap.add_argument("--cycles-at", type=parse_schedule, default=None,
                    help="line: blocks of amplitude:cycles in s, e.g. 0.6:6,0.4:3,0.8:3")
    ap.add_argument("--centre", type=float, default=0.0,
                    help="line: s at the zero holds; the turns are centre +/- a (default 0)")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="sweep rate of the largest change, kPa/s (default 1)")
    ap.add_argument("--cycles", type=int, default=6, help="pair: cycles (default 6)")
    ap.add_argument("--pre-hold", type=float, default=0.0,
                    help="seconds at the base pressures before the first cycle (creep test)")
    ap.add_argument("--dither", type=float, default=0.0,
                    help="pair: dither amplitude, kPa, on the differential during the cycles")
    ap.add_argument("--dither-hz", type=float, default=2.0, help="dither frequency (default 2)")
    ap.add_argument("--hold", type=parse_holds, default={},
                    help="other muscles held during the run, by name or as V<regulator> "
                         "for one outside the mapping, e.g. FCU=30,ECU=30,V28=60")
    ap.add_argument("--max-kpa", type=float, default=50.0, help="cap for this run (default 50)")
    ap.add_argument("--adc-hz", type=float, default=0.0,
                    help="poll the Giga's pressure readback at this rate (0 = off)")
    # The run's condition: stored in the .json and in the file name.
    ap.add_argument("--mass-g", type=float, default=None, help="load on the hand, g")
    ap.add_argument("--station", default=None, help="load station id, e.g. palm1")
    ap.add_argument("--session", default=None, help="session id, e.g. d1")
    ap.add_argument("--mount", default=None, help="mount id, e.g. m1")
    ap.add_argument("--plan", action="store_true",
                    help="only print the plan: no port opened, nothing written")
    ap.add_argument("--arm", action="store_true", help="actually send pressures")
    ap.add_argument("--out", default="runs", help="output folder (default runs/)")
    ap.add_argument("--mapping", default=DEFAULT_MAPPING_PATH)
    ap.add_argument("--valve-port", default=None)
    ap.add_argument("--rs485-ports", default=None, help="comma list (default: every EKU081 channel)")
    args = ap.parse_args(argv)
    if args.plan and args.arm:
        ap.error("--plan and --arm together: --plan never sends")

    by_name = muscles_by_name(args.mapping)
    for n in args.hold:
        if n in by_name and by_name[n]["regulator"] is not None:
            continue
        reg = re.fullmatch(r"[Vv](\d+)", n)
        if reg and int(reg.group(1)) <= MAX_REGULATOR:
            by_name[n] = {"regulator": int(reg.group(1)), "sensor": None, "region": None}
        else:
            ap.error(f"{n}: no regulator in the mapping (use V<id> for an unmapped regulator)")
    condition = {"mass_g": args.mass_g, "station": args.station, "session": args.session,
                 "mount": args.mount}
    dither = None

    if args.pair:
        names = [n.strip() for n in args.pair.split(",")]
        if len(names) != 2:
            ap.error("--pair needs exactly two muscles")
        if args.cycles_at or args.centre:
            ap.error("--cycles-at and --centre belong to --line")
        a, b = names
        for n in (a, b):
            m = by_name.get(n)
            if m is None or m["regulator"] is None or m["sensor"] is None:
                ap.error(f"{n}: needs a regulator and a sensor in the mapping")
        segs = pair_sweep_segments(a, b, args.pretension, args.amplitude, args.rate,
                                   args.cycles, args.hold, pre_hold_s=args.pre_hold,
                                   turn_plus=args.turn_plus, turn_minus=args.turn_minus)
        dither = (args.dither, (a, b)) if args.dither > 0 else None
        tp = next(s.targets for s in segs if s.label == "c1 hold +")
        tm = next(s.targets for s in segs if s.label == "c1 hold -")
        involved = [a, b, *args.hold]
        muscles = {n: by_name[n] for n in involved}
        cycles = args.cycles
        name = f"{a}-{b}_P{args.pretension:g}"
        meta = {"mode": "pair", "pair": [a, b], "pretension": args.pretension,
                "amplitude": args.amplitude,
                "turn_plus": [tp[a], tp[b]], "turn_minus": [tm[a], tm[b]]}
    else:
        line = args.line
        if not args.cycles_at:
            ap.error("--line needs --cycles-at, e.g. 0.6:6")
        if args.dither or args.turn_plus or args.turn_minus:
            ap.error("--dither, --turn-plus and --turn-minus belong to --pair")
        for n in line:
            m = by_name.get(n)
            if m is None or m["regulator"] is None:
                ap.error(f"{n}: no regulator in the mapping")
        try:
            segs = line_sweep_segments(line, args.cycles_at, args.rate, args.hold,
                                       centre=args.centre, pre_hold_s=args.pre_hold)
        except ValueError as exc:
            ap.error(str(exc))
        involved = [*line, *args.hold]
        muscles = {n: by_name[n] for n in involved}
        cycles = sum(n for _, n in args.cycles_at)
        name = "-".join(line) + "_line"
        turns = schedule_turns(args.cycles_at, args.centre)
        meta = {"mode": "line", "line": {n: list(v) for n, v in line.items()},
                "centre": args.centre, "schedule": [list(b) for b in args.cycles_at],
                "turns": [list(t) for t in turns]}

    regs = [m["regulator"] for m in muscles.values()]
    twice = sorted({r for r in regs if regs.count(r) > 1})
    if twice:
        ap.error(f"regulator(s) {twice} driven twice in this run")
    total, ranges = plan_summary(segs, {}, dither=dither)
    top = max(hi for _, hi in ranges.values())
    if top > args.max_kpa:
        ap.error(f"the plan reaches {top:g} kPa, above --max-kpa {args.max_kpa:g}")
    if dither and cycle_min(segs, dither[1]) - args.dither < 0:
        ap.error("the dither would take a pressure below 0 kPa; lower --dither or --amplitude")

    if args.pair:
        print(f"pair {a} (V{muscles[a]['regulator']}, s{muscles[a]['sensor']}) vs "
              f"{b} (V{muscles[b]['regulator']}, s{muscles[b]['sensor']}): base "
              f"{args.pretension:g}/{args.pretension:g}, turns {tp[a]:g}/{tp[b]:g} and "
              f"{tm[a]:g}/{tm[b]:g} kPa, {args.rate:g} kPa/s, "
              f"{args.cycles} cycles, {total / 60:.1f} min"
              + (f", pre-hold {args.pre_hold:g} s" if args.pre_hold else "")
              + (f", dither +/-{args.dither:g} kPa at {args.dither_hz:g} Hz" if dither else ""))
    else:
        who = ", ".join(f"{n} (V{muscles[n]['regulator']}, s{muscles[n]['sensor']})"
                        for n in line)
        print(f"line {who}: "
              f"centre s = {args.centre:g}, {cycles} cycles, largest change {args.rate:g} kPa/s, "
              f"{total / 60:.1f} min" + (f", pre-hold {args.pre_hold:g} s" if args.pre_hold else ""))
        print(f"  {'s':<18}" + "".join(f"{n:>7}" for n in line))
        shown = [("-1", -1.0), ("+1", 1.0), (f"{args.centre:g} (zero)", args.centre)]
        for a_, n_ in args.cycles_at:
            shown += [(f"{args.centre - a_:+g} ({n_}x a {a_:g})", args.centre - a_),
                      (f"{args.centre + a_:+g} ({n_}x a {a_:g})", args.centre + a_)]
        for label, s_ in shown:
            p = line_pressures(line, s_)
            print(f"  {label:<18}" + "".join(f"{p[n]:7.1f}" for n in line))
    for n in involved:
        lo, hi = ranges[n]
        print(f"  {n:4} V{muscles[n]['regulator']:<2} {lo:5.1f} .. {hi:5.1f} kPa")
    if any(v is not None for v in condition.values()):
        print("condition: " + ", ".join(f"{k} {v:g}" if isinstance(v, float) else f"{k} {v}"
                                        for k, v in condition.items() if v is not None))
    if args.plan:
        print("PLAN ONLY: no port opened, nothing written.")
        return 0

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(args.out, f"{stamp}_{name}{run_tag(condition)}"
                                  f"{'' if args.arm else '_dry'}")
    with open(base + ".json", "w") as fh:
        json.dump({**meta, "rate": args.rate, "cycles": cycles, "hold": args.hold,
                   "pre_hold": args.pre_hold,
                   "dither": ({"amp": args.dither, "hz": args.dither_hz} if dither else None),
                   "condition": condition,
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
