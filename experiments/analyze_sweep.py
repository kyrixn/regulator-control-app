#!/usr/bin/env python3
"""
experiments/analyze_sweep.py

Numbers from a pair_sweep.py run (spec:
docs/superpowers/specs/2026-10-01-pair-sweep-design.md):

  dead band     how far the differential d travels after each turn (end of the
                hold at +A or -A) before a length moves more than --slip mm. It
                is measured up to the next turn, because on this wrist the band
                runs past d = 0 (friction, kPa)
  creep         length change during each 2 s turn hold (the wrist still moving)
  gap at zero   length at "zero from +" minus at "zero from -" (dead band, mm)
  slope         length per kPa of d on the moving branches (median, cycles 2+)
  settling      each cycle's reference lengths (end of "zero from -"); settled
                at the first cycle that changes less than --tol mm from the last
  readback      a straight-line fit of the Giga's ADC volts to the commanded kPa
                during holds, per regulator, if the run logged it

Line-mode runs (pair_sweep.py --line) are analysed the same way along the
sweep coordinate s instead of d, for every swept muscle, plus:

  clipping      whether a length flattens near the end of a ramp to a turn
                while s still moves (a joint stop), per turn and muscle
  by amplitude  dead band and slope per block of --cycles-at; settling is
                judged over the first block only

The first version measured breakaway from each ramp's start with a 0.05 mm
threshold; on the real wrist that caught creep after the turn, not slip.

Writes <run>_report.txt, <run>_loops.png and <run>_settling.png next to the CSV.

    python experiments/analyze_sweep.py runs/20261001-101500_FCR-ECR_P30.csv
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM  # noqa: E402

MM_PER_COUNT = math.pi * DEFAULT_DRUM_DIAMETER_MM / DEFAULT_COUNTS_PER_TURN
FIRMWARE_V_PER_KPA = 10.0 / 600.0          # the sketch's assumed 0-10 V <-> -100..500 kPa
FIRMWARE_V_AT_0KPA = 100.0 / 60.0


# ----------------------------------------------------------------------------
# Pure helpers (unit-tested)
# ----------------------------------------------------------------------------

def interp(tq: float, t: Sequence[float], y: Sequence[float]) -> float:
    """Linear interpolation, clamped at the ends."""
    i = bisect.bisect_left(t, tq)
    if i <= 0:
        return y[0]
    if i >= len(t):
        return y[-1]
    t0, t1 = t[i - 1], t[i]
    return y[i - 1] if t1 == t0 else y[i - 1] + (y[i] - y[i - 1]) * (tq - t0) / (t1 - t0)


def step_at(tq: float, t: Sequence[float], y: Sequence[float]) -> Optional[float]:
    """Last value at or before tq (a command holds until the next one)."""
    i = bisect.bisect_right(t, tq) - 1
    return y[i] if i >= 0 else None


def breakaway(t_len: Sequence[float], length: Sequence[float], t_cmd: Sequence[float],
              d_cmd: Sequence[float], t0: float, t1: float, thr_mm: float) -> Optional[float]:
    """|d(t_move) - d(t0)|: how far the differential travelled in [t0, t1] before
    the length first moved more than thr_mm from its value at t0. None if never."""
    idx = [i for i, t in enumerate(t_len) if t0 <= t <= t1]
    if not idx:
        return None
    start = length[idx[0]]
    for i in idx:
        if abs(length[i] - start) > thr_mm:
            return abs(interp(t_len[i], t_cmd, d_cmd) - interp(t0, t_cmd, d_cmd))
    return None


def clipped(t_len: Sequence[float], length: Sequence[float], t_cmd: Sequence[float],
            x_cmd: Sequence[float], t0: float, t1: float, moving_mm: float) -> Optional[bool]:
    """Did the length flatten at the end of the sweep ramp [t0, t1] (a joint
    stop)? Compares its slope against the sweep coordinate over the last 20 % of
    the ramp with the 20 % before. None when it was not moving there (less than
    moving_mm over that window), e.g. still inside the dead band."""
    ta, tb = t0 + 0.6 * (t1 - t0), t0 + 0.8 * (t1 - t0)
    dy1 = interp(tb, t_len, length) - interp(ta, t_len, length)
    dy2 = interp(t1, t_len, length) - interp(tb, t_len, length)
    dx1 = interp(tb, t_cmd, x_cmd) - interp(ta, t_cmd, x_cmd)
    dx2 = interp(t1, t_cmd, x_cmd) - interp(tb, t_cmd, x_cmd)
    if abs(dy1) < moving_mm or not dx1 or not dx2:
        return None
    return abs(dy2 / dx2) < 0.3 * abs(dy1 / dx1)


def line_coordinate(line: Dict[str, Sequence[float]], muscle: str,
                    kpa: Sequence[float]) -> List[float]:
    """A line-mode muscle's commanded kPa -> the sweep coordinate s (its kPa at
    s = -1 and s = +1 are line[muscle])."""
    lo, hi = line[muscle]
    return [2.0 * (p - lo) / (hi - lo) - 1.0 for p in kpa]


def hold_mean(t: Sequence[float], y: Sequence[float], t0: float, t1: float,
              last_s: float = 1.0) -> Optional[float]:
    """Mean of y over the last `last_s` seconds of [t0, t1]."""
    vals = [v for tt, v in zip(t, y) if max(t0, t1 - last_s) <= tt <= t1]
    return sum(vals) / len(vals) if vals else None


def settled_cycle(refs: Sequence[Dict[int, float]], tol_mm: float) -> Optional[int]:
    """First cycle (1-based) whose reference differs from the previous cycle's
    by less than tol_mm on every sensor."""
    for k in range(1, len(refs)):
        prev, cur = refs[k - 1], refs[k]
        common = set(prev) & set(cur)
        if common and all(abs(cur[s] - prev[s]) < tol_mm for s in common):
            return k + 1
    return None


def differential(t_a: Sequence[float], p_a: Sequence[float], t_b: Sequence[float],
                 p_b: Sequence[float], base: Tuple[float, float]) -> List[float]:
    """The sweep coordinate at a's command times: half the pair's pressure
    difference, relative to the base, ((p_a - p_b) - (base_a - base_b)) / 2. With
    a constant sum it equals p_a - P; it also stays meaningful when the two turns
    have different sums (e.g. 60/10 and 10/50)."""
    return [((pa - interp(t, t_b, p_b)) - (base[0] - base[1])) / 2.0
            for t, pa in zip(t_a, p_a)]


def smooth(t: Sequence[float], y: Sequence[float], window_s: float) -> List[float]:
    """Centred moving average over window_s seconds (removes a dither of that period)."""
    cum = [0.0]
    for v in y:
        cum.append(cum[-1] + v)
    out = []
    for tt in t:
        i = bisect.bisect_left(t, tt - window_s / 2)
        j = bisect.bisect_right(t, tt + window_s / 2)
        out.append((cum[j] - cum[i]) / (j - i))
    return out


def drift(t: Sequence[float], y: Sequence[float], t0: float, t1: float,
          last_s: float = 60.0) -> Tuple[float, float]:
    """(total change over [t0, t1], rate in units/min over its last last_s seconds)."""
    total = interp(t1, t, y) - interp(t0, t, y)
    ta = max(t0, t1 - last_s)
    rate = (interp(t1, t, y) - interp(ta, t, y)) / ((t1 - ta) / 60.0) if t1 > ta else 0.0
    return total, rate


def line_fit(x: Sequence[float], y: Sequence[float]) -> Tuple[float, float, float]:
    """Least squares y = m x + c; returns (m, c, r^2)."""
    n = len(x)
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    m = sxy / sxx
    c = my - m * mx
    ss_res = sum((b - (m * a + c)) ** 2 for a, b in zip(x, y))
    ss_tot = sum((b - my) ** 2 for b in y)
    return m, c, 1.0 - ss_res / ss_tot if ss_tot else 1.0


# ----------------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------------

def load(csv_path: str):
    lens: Dict[int, Tuple[List[float], List[float]]] = {}
    cmds: Dict[int, Tuple[List[float], List[float]]] = {}
    adc: Dict[int, Tuple[List[float], List[float]]] = {}
    bases: Dict[int, Tuple[List[float], List[float]]] = {}
    marks: List[Tuple[float, str]] = []
    with open(csv_path, newline="") as fh:
        for r in csv.DictReader(fh):
            t = float(r["t"])
            if r["kind"] == "len" and r["value"] != "":
                ts, vs = lens.setdefault(int(r["id"]), ([], []))
                ts.append(t)
                vs.append(int(r["value"]))
            elif r["kind"] == "cmd" and r["value"] not in ("", "off"):
                ts, vs = cmds.setdefault(int(r["id"]), ([], []))
                ts.append(t)
                vs.append(float(r["value"]))
            elif r["kind"] == "base" and r["value"] != "":
                ts, vs = bases.setdefault(int(r["id"]), ([], []))
                ts.append(t)
                vs.append(float(r["value"]))
            elif r["kind"] == "adc":
                ts, vs = adc.setdefault(int(r["id"]), ([], []))
                ts.append(t)
                vs.append(float(r["value"]))
            elif r["kind"] == "mark":
                marks.append((t, r["note"]))
    for series in (lens, cmds, adc, bases):
        for s, (ts, vs) in series.items():
            order = sorted(range(len(ts)), key=ts.__getitem__)
            series[s] = ([ts[i] for i in order], [vs[i] for i in order])
    return lens, cmds, adc, sorted(marks), bases


def segment_times(marks: List[Tuple[float, str]]) -> Dict[str, Tuple[float, float]]:
    """label -> (start, end), each segment ending at the next mark."""
    out = {}
    for (t0, label), (t1, _) in zip(marks, marks[1:]):
        out[label] = (t0, t1)
    return out


# ----------------------------------------------------------------------------
# Numbers and report
# ----------------------------------------------------------------------------

def compute(csv_path: str, slip_mm: float = 0.5, tol_mm: float = 0.1,
            exclude: Sequence[str] = ("Sup",)) -> dict:
    """All numbers of one run, per cycle and overall (see the module docstring)."""
    with open(os.path.splitext(csv_path)[0] + ".json") as fh:
        meta = json.load(fh)
    line = meta.get("line") if meta.get("mode") == "line" else None
    names = list(line) if line else list(meta["pair"])
    ncyc = meta["cycles"]
    muscles = meta["muscles"]
    sensors = tuple(muscles[n]["sensor"] for n in names if muscles[n]["sensor"] is not None)
    lens, cmds, adc, marks, bases = load(csv_path)
    seg = segment_times(marks)
    res = {"meta": meta, "pair": tuple(names), "sensors": sensors, "line": line,
           "cycles": [], "slope": {}, "slope_by_amp": {},
           "settled": None, "readback": {}, "seg": seg, "no_cycles": "c1 to +" not in seg}
    if res["no_cycles"]:
        return res

    # Lengths in mm relative to each sensor's first sample; d from a's command.
    mm = {s: (ts, [(v - vs[0]) * MM_PER_COUNT for v in vs]) for s, (ts, vs) in lens.items()}
    dither = meta.get("dither")
    if dither:
        # The dither moves the lengths at its own frequency; average it out so
        # the dead band and gaps describe the slow sweep.
        mm = {s: (ts, smooth(ts, ys, 1.0 / dither["hz"])) for s, (ts, ys) in mm.items()}
    if line:
        # s from the muscle with the largest pressure span (the finest steps).
        lead = max(line, key=lambda n: abs(line[n][1] - line[n][0]))
        t_cmd, kpa_lead = cmds[muscles[lead]["regulator"]]
        d_cmd = line_coordinate(line, lead, kpa_lead)
        res["lead"] = lead
    else:
        a, b = names
        P = meta["pretension"]
        ra, rb = muscles[a]["regulator"], muscles[b]["regulator"]
        # The sweep's own coordinate, from the base setpoints when dithering, else
        # from the commands: d = half the pair's pressure difference from the base.
        src = bases if ra in bases and rb in bases else cmds
        t_cmd, kpa_a = src[ra]
        d_cmd = differential(t_cmd, kpa_a, *src[rb], base=(P, P))
    res.update(mm=mm, t_cmd=t_cmd, d_cmd=d_cmd)
    name_of = {m["sensor"]: n for n, m in muscles.items() if m["sensor"] is not None}
    ref_sensors = [s for s, n in name_of.items() if n not in exclude and s in mm]
    res.update(name_of=name_of, ref_sensors=ref_sensors)
    pair = [s for s in sensors if s in mm]

    def end_of(label):
        return seg[label][1] if label in seg else None

    def change_over(s, label):
        if label not in seg:
            return None
        t0, t1 = seg[label]
        return interp(t1, *mm[s]) - interp(t0, *mm[s])

    refs = []
    for k in range(1, ncyc + 1):
        t_plus, t_minus = end_of(f"c{k} hold +"), end_of(f"c{k} hold -")
        t_next = end_of(f"c{k + 1} hold +") or (seg["vent"][0] if "vent" in seg else None)
        c = {"cycle": k, "dead_plus": {}, "dead_minus": {}, "creep_plus": {}, "creep_minus": {},
             "gap": {}, "clip_plus": {}, "clip_minus": {}}
        if line and k <= len(meta.get("turns", [])):
            s_minus, s_plus = meta["turns"][k - 1]
            c["amp"] = round((s_plus - s_minus) / 2.0, 6)
        for s in pair:
            for sign in ("+", "-"):
                r = seg.get(f"c{k} to {sign}")
                c["clip_plus" if sign == "+" else "clip_minus"][s] = (
                    clipped(*mm[s], t_cmd, d_cmd, *r, slip_mm / 2) if line and r else None)
            c["dead_plus"][s] = (breakaway(*mm[s], t_cmd, d_cmd, t_plus, t_minus, slip_mm)
                                 if t_plus and t_minus else None)
            c["dead_minus"][s] = (breakaway(*mm[s], t_cmd, d_cmd, t_minus, t_next, slip_mm)
                                  if t_minus and t_next else None)
            c["creep_plus"][s] = change_over(s, f"c{k} hold +")
            c["creep_minus"][s] = change_over(s, f"c{k} hold -")
            zp, zm = seg.get(f"c{k} zero from +"), seg.get(f"c{k} zero from -")
            vp = hold_mean(*mm[s], *zp) if zp else None
            vm = hold_mean(*mm[s], *zm) if zm else None
            c["gap"][s] = None if vp is None or vm is None else vp - vm
        zm = seg.get(f"c{k} zero from -")
        ref = {s: hold_mean(*mm[s], *zm) for s in ref_sensors} if zm else {}
        c["ref"] = {s: v for s, v in ref.items() if v is not None}
        refs.append(c["ref"])
        c["change"] = (max(abs(c["ref"][s] - refs[-2][s]) for s in c["ref"] if s in refs[-2])
                       if k > 1 and c["ref"] and refs[-2] else None)
        res["cycles"].append(c)
    first_block = meta["schedule"][0][1] if line else len(refs)
    res["settled"] = settled_cycle(refs[:first_block], tol_mm)

    if "pre-hold" in seg:
        t0, t1 = seg["pre-hold"]
        res["pre_hold"] = {s: drift(*mm[s], t0, t1) for s in ref_sensors}

    # Moving-branch slope: the last 40 % of each sweep ramp, past the dead band
    # (line mode: ramps that ran into a stop left out, and also per amplitude).
    for s in pair:
        slopes, by_amp = [], {}
        for k in range(2, ncyc + 1):
            c = res["cycles"][k - 1]
            for lab in ("to +", "to -"):
                if f"c{k} {lab}" not in seg:
                    continue
                if c[{"to +": "clip_plus", "to -": "clip_minus"}[lab]].get(s):
                    continue
                t0, t1 = seg[f"c{k} {lab}"]
                ta = t0 + 0.6 * (t1 - t0)
                dd = interp(t1, t_cmd, d_cmd) - interp(ta, t_cmd, d_cmd)
                if dd:
                    v = (interp(t1, *mm[s]) - interp(ta, *mm[s])) / dd
                    slopes.append(v)
                    if "amp" in c:
                        by_amp.setdefault(c["amp"], []).append(v)
        res["slope"][s] = sorted(slopes)[len(slopes) // 2] if slopes else None
        for amp, vals in by_amp.items():
            res["slope_by_amp"].setdefault(amp, {})[s] = sorted(vals)[len(vals) // 2]

    # Readback: volts against commanded kPa during holds (first 1 s skipped).
    holds = [(t0 + 1.0, t1) for label, (t0, t1) in seg.items()
             if "hold" in label or "zero" in label or label == "settle"]
    for reg, (ta, va) in sorted(adc.items()):
        if reg not in cmds:
            continue
        xs, ys = [], []
        for t, v in zip(ta, va):
            if any(h0 <= t <= h1 for h0, h1 in holds):
                kp = step_at(t, *cmds[reg])
                if kp is not None:
                    xs.append(kp)
                    ys.append(v)
        if xs:
            means = {}
            for x, y in zip(xs, ys):
                means.setdefault(x, []).append(y)
            res["readback"][reg] = {
                "fit": line_fit(xs, ys) if len(means) >= 2 else None,
                "points": sorted((x, sum(v) / len(v)) for x, v in means.items()),
            }
    return res


def render(res: dict, csv_path: str, slip_mm: float, tol_mm: float) -> str:
    if res["no_cycles"]:
        return f"{csv_path}: no sweep cycles in this run (dry run?)"
    meta = res["meta"]
    a, b = res["pair"]
    sa, sb = res["sensors"]
    name_of = res["name_of"]

    def fmt(v, p=1):
        return "-" if v is None else f"{v:.{p}f}"

    lines = [f"run {os.path.basename(csv_path)}",
             f"pair {a} (s{sa}) vs {b} (s{sb}), base {meta['pretension']:g}/{meta['pretension']:g}"
             + (f", turns {meta['turn_plus'][0]:g}/{meta['turn_plus'][1]:g} and "
                f"{meta['turn_minus'][0]:g}/{meta['turn_minus'][1]:g} kPa" if meta.get("turn_plus")
                else f", A {meta['amplitude']:g} kPa")
             + f", {meta['rate']:g} kPa/s, held {meta['hold']}",
             f"d = half the pair's pressure difference from the base, (p({a}) - p({b}))/2; "
             "dead bands and slopes are per kPa of d",
             f"slip threshold {slip_mm:g} mm; settling tolerance {tol_mm:g} mm "
             f"(sensors: {', '.join(name_of[s] for s in res['ref_sensors'])})"]
    if meta.get("dither"):
        dz = meta["dither"]
        lines.append(f"dither +/-{dz['amp']:g} kPa at {dz['hz']:g} Hz on the differential; "
                     f"lengths averaged over {1 / dz['hz']:.2f} s")
    if res.get("pre_hold"):
        t0, t1 = res["seg"]["pre-hold"]
        lines.append(f"pre-hold {t1 - t0:.0f} s at the base pressures, before any cycling: "
                     + "; ".join(f"{name_of[s]} {tot:+.3f} mm (last min {rate:+.3f} mm/min)"
                                 for s, (tot, rate) in res["pre_hold"].items()))
    lines += ["",
             "      | dead band after turn, kPa      | creep in turn hold, mm         "
             "| gap at 0, mm  | reference, mm             | change",
             f"cycle | +A: {a:>5} {b:>5}  -A: {a:>5} {b:>5} | +A: {a:>5} {b:>5}  -A: {a:>5} {b:>5} "
             f"| {a:>6} {b:>6} | {a:>7} {b:>7} {'sum':>7} | mm"]
    for c in res["cycles"]:
        def g(key, s, p=1):
            return fmt(c[key].get(s), p)
        ref = c["ref"]
        ref_sum = ref[sa] + ref[sb] if sa in ref and sb in ref else None
        lines.append(
            f"{c['cycle']:5} |     {g('dead_plus', sa):>5} {g('dead_plus', sb):>5}      "
            f"{g('dead_minus', sa):>5} {g('dead_minus', sb):>5} |     "
            f"{g('creep_plus', sa, 2):>5} {g('creep_plus', sb, 2):>5}      "
            f"{g('creep_minus', sa, 2):>5} {g('creep_minus', sb, 2):>5} | "
            f"{g('gap', sa, 2):>6} {g('gap', sb, 2):>6} | {fmt(ref.get(sa), 3):>7} "
            f"{fmt(ref.get(sb), 3):>7} {fmt(ref_sum, 3):>7} | {fmt(c['change'], 3)}")

    def median(vals):
        vals = sorted(v for v in vals if v is not None)
        return vals[len(vals) // 2] if vals else None

    later = res["cycles"][1:] or res["cycles"]
    lines += ["", "summary (cycles 2+):"]
    for s, n in ((sa, a), (sb, b)):
        lines.append(f"  {n}: dead band {fmt(median(c['dead_plus'].get(s) for c in later))} kPa "
                     f"after +A, {fmt(median(c['dead_minus'].get(s) for c in later))} kPa after -A; "
                     f"moving branch {fmt(res['slope'].get(s), 3)} mm/kPa; gap at 0 "
                     f"{fmt(median(c['gap'].get(s) for c in later), 2)} mm")
    last = res["cycles"][-1]["change"]
    lines.append(f"  settled at cycle {res['settled']}" if res["settled"] else
                 f"  not settled within {len(res['cycles'])} cycles (tolerance {tol_mm:g} mm; "
                 f"last change {fmt(last, 3)} mm)")

    return "\n".join(lines + readback_lines(res))


def readback_lines(res: dict) -> List[str]:
    lines = []
    if res["readback"]:
        fits = [r["fit"] for r in res["readback"].values() if r["fit"]]
        lines += ["", "pressure readback (holds, first 1 s skipped); the sketch assumes "
                  f"{FIRMWARE_V_PER_KPA:.5f} V/kPa and {FIRMWARE_V_AT_0KPA:.3f} V at 0 kPa:"]
        m_avg = sum(f[0] for f in fits) / len(fits) if fits else None
        c_avg = sum(f[1] for f in fits) / len(fits) if fits else None
        for reg, r in sorted(res["readback"].items()):
            if r["fit"]:
                m, c0, r2 = r["fit"]
                lines.append(f"  V{reg:<3} fit {m:.5f} V/kPa, {c0:.3f} V at 0 kPa, r^2 {r2:.4f}")
            else:
                kp, v = r["points"][0]
                pred = "" if m_avg is None else f" (fit line predicts {m_avg * kp + c_avg:.3f} V)"
                lines.append(f"  V{reg:<3} single pressure {kp:g} kPa reads {v:.3f} V{pred}")
    return lines


def _fmt(v, p=1):
    return "-" if v is None else f"{v:.{p}f}"


def _median(vals):
    vals = sorted(v for v in vals if v is not None)
    return vals[len(vals) // 2] if vals else None


def render_line(res: dict, csv_path: str, slip_mm: float, tol_mm: float) -> str:
    """The report of a line-mode run: everything along s, one column per muscle."""
    meta, line, name_of = res["meta"], res["line"], res["name_of"]
    sensors = [sn for sn in res["sensors"] if sn in res["mm"]]
    names = [name_of[sn] for sn in sensors]
    lead = res["lead"]
    lo, hi = line[lead]
    cond = {k: v for k, v in (meta.get("condition") or {}).items() if v is not None}
    first_block = meta["schedule"][0][1]
    lines = [f"run {os.path.basename(csv_path)}",
             "line " + ", ".join(f"{n} {line[n][0]:g}:{line[n][1]:g}" for n in line)
             + " kPa at s = -1:+1; centre s = " + f"{meta.get('centre', 0):g}"
             + f", held {meta['hold']}, {meta['rate']:g} kPa/s (largest change)",
             f"s from the {lead} command: 1 unit of s = {abs(hi - lo) / 2:g} kPa of {lead}; "
             "dead bands are in units of s, slopes in mm per unit of s",
             f"slip threshold {slip_mm:g} mm; settling over the first block "
             f"({first_block} cycles), tolerance {tol_mm:g} mm"]
    if cond:
        lines.append("condition: " + ", ".join(f"{k} {v:g}" if isinstance(v, float)
                                               else f"{k} {v}" for k, v in cond.items()))
    if res.get("pre_hold"):
        t0, t1 = res["seg"]["pre-hold"]
        lines.append(f"pre-hold {t1 - t0:.0f} s at the centre pressures, before any cycling: "
                     + "; ".join(f"{name_of[sn]} {tot:+.3f} mm (last min {rate:+.3f} mm/min)"
                                 for sn, (tot, rate) in res["pre_hold"].items()))

    def clips(c, key):
        hit = [name_of[sn] for sn in sensors if c[key].get(sn)]
        return ",".join(hit) if hit else "-"

    w = 11
    lines += ["", "dead band after each turn, s (after +turn / after -turn); clipped = length "
              "flattened before the turn (a stop)",
              "cycle    a | " + "".join(f"{n:>{w}}" for n in names) + " | clipped +  clipped -"]
    for c in res["cycles"]:
        cells = "".join(f"{_fmt(c['dead_plus'].get(sn), 2) + '/' + _fmt(c['dead_minus'].get(sn), 2):>{w}}"
                        for sn in sensors)
        lines.append(f"{c['cycle']:5} {_fmt(c.get('amp'), 2):>4} | {cells} | "
                     f"{clips(c, 'clip_plus'):<10} {clips(c, 'clip_minus')}")

    lines += ["", "gap at the centre (zero from + minus zero from -) and reference "
              "(end of zero from -), mm",
              "cycle    a | gap " + "".join(f"{n:>7}" for n in names)
              + " | ref " + "".join(f"{n:>8}" for n in names) + " | change"]
    for c in res["cycles"]:
        lines.append(f"{c['cycle']:5} {_fmt(c.get('amp'), 2):>4} |     "
                     + "".join(f"{_fmt(c['gap'].get(sn), 2):>7}" for sn in sensors) + " |     "
                     + "".join(f"{_fmt(c['ref'].get(sn), 3):>8}" for sn in sensors)
                     + f" | {_fmt(c['change'], 3)}")

    lines += ["", "by amplitude (medians; cycle 1 left out):"]
    amps = sorted({c["amp"] for c in res["cycles"] if "amp" in c})
    for amp in amps:
        cs = [c for c in res["cycles"] if c.get("amp") == amp and c["cycle"] > 1]
        if not cs:
            continue
        ks = ",".join(str(c["cycle"]) for c in cs)
        lines.append(f"  a {amp:g} (cycles {ks}):")
        for sn, n in zip(sensors, names):
            nclip = sum(bool(c["clip_plus"].get(sn)) + bool(c["clip_minus"].get(sn)) for c in cs)
            slope = res["slope_by_amp"].get(amp, {}).get(sn)
            lines.append(
                f"    {n:4} dead band {_fmt(_median(c['dead_plus'].get(sn) for c in cs), 2)} "
                f"after +, {_fmt(_median(c['dead_minus'].get(sn) for c in cs), 2)} after -; "
                f"slope {_fmt(slope, 2)} mm/s-unit; gap {_fmt(_median(c['gap'].get(sn) for c in cs), 2)} mm; "
                f"clipped {nclip}/{2 * len(cs)} turns")
    last = res["cycles"][first_block - 1]["change"] if len(res["cycles"]) >= first_block else None
    lines.append(f"  settled at cycle {res['settled']} (first block)" if res["settled"] else
                 f"  not settled within the first block of {first_block} cycles (tolerance "
                 f"{tol_mm:g} mm; last change {_fmt(last, 3)} mm)")
    return "\n".join(lines + readback_lines(res))


def analyze(csv_path: str, slip_mm: float = 0.5, tol_mm: float = 0.1,
            exclude: Sequence[str] = ("Sup",), plots: bool = True) -> str:
    res = compute(csv_path, slip_mm, tol_mm, exclude)
    if res["line"] and not res["no_cycles"]:
        report = render_line(res, csv_path, slip_mm, tol_mm)
    else:
        report = render(res, csv_path, slip_mm, tol_mm)
    base = os.path.splitext(csv_path)[0]
    with open(base + "_report.txt", "w") as fh:
        fh.write(report + "\n")
    if plots and not res["no_cycles"]:
        refs = [c["ref"] for c in res["cycles"]]
        swept = [sn for sn in res["sensors"] if sn in res["mm"]]
        if res["line"]:
            xlabel = f"s (1 unit = {abs(res['line'][res['lead']][1] - res['line'][res['lead']][0]) / 2:g} kPa of {res['lead']})"
            total = None
        else:
            a, b = res["pair"]
            xlabel = f"d = (p({a}) − p({b}))/2 from the base  (kPa)"
            total = (res["sensors"], f"{a}+{b}")
        _plots(base, res["mm"], res["t_cmd"], res["d_cmd"], res["seg"], len(res["cycles"]),
               swept, [res["name_of"][sn] for sn in swept], xlabel, refs, res["ref_sensors"],
               res["name_of"], total)
    return report


def _plots(base, mm, t_cmd, d_cmd, seg, cycles, swept, names, xlabel, refs, sensors,
           name_of, total=None):
    """<run>_loops.png (length against the sweep coordinate, one panel per swept
    muscle, one colour per cycle) and <run>_settling.png (each cycle's reference
    lengths; with total = ((sa, sb), label) also their sum)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ncol = min(len(swept), 2)
    nrow = (len(swept) + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.5 * ncol, 4.5 * nrow), sharex=True,
                             squeeze=False)
    axes = [ax for row in axes for ax in row]
    cmap = plt.get_cmap("viridis")
    for k in range(1, cycles + 1):
        t0 = seg.get(f"c{k} to +", (None,))[0]
        t1 = seg.get(f"c{k} zero from -", (None, None))[1]
        if t0 is None or t1 is None:
            continue
        for ax, sn in zip(axes, swept):
            ts, ys = mm[sn]
            pts = [(interp(t, t_cmd, d_cmd), y) for t, y in zip(ts, ys) if t0 <= t <= t1]
            if pts:
                ax.plot(*zip(*pts), color=cmap((k - 1) / max(1, cycles - 1)), lw=1,
                        label=f"cycle {k}")
    for ax, n in zip(axes, names):
        ax.set_title(n)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("length change (mm)")
        ax.grid(alpha=0.3)
    for ax in axes[len(swept):]:
        ax.set_visible(False)
    axes[min(1, len(swept) - 1)].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(base + "_loops.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ks = list(range(1, len(refs) + 1))
    for sn in sensors:
        ax.plot(ks, [r.get(sn, float("nan")) for r in refs], marker="o", label=name_of[sn])
    if total:
        (sa, sb), label = total
        ax.plot(ks, [r.get(sa, float("nan")) + r.get(sb, float("nan")) for r in refs],
                marker="s", ls="--", color="k", label=label)
    ax.set_xlabel("cycle")
    ax.set_ylabel("reference length (mm, end of 'zero from −')")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_settling.png", dpi=130)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Analyse a pair_sweep.py run (pair or line mode).")
    ap.add_argument("csv")
    ap.add_argument("--slip", type=float, default=0.5,
                    help="length change that counts as slip after a turn, mm (default 0.5)")
    ap.add_argument("--tol", type=float, default=0.1, help="settling tolerance, mm")
    ap.add_argument("--exclude", default="Sup", help="muscles left out of settling")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)
    print(analyze(args.csv, args.slip, args.tol,
                  [n.strip() for n in args.exclude.split(",") if n.strip()],
                  plots=not args.no_plots))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
