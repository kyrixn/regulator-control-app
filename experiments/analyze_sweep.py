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
    a, b = meta["pair"]
    P, ncyc = meta["pretension"], meta["cycles"]
    muscles = meta["muscles"]
    sa, sb = muscles[a]["sensor"], muscles[b]["sensor"]
    lens, cmds, adc, marks, bases = load(csv_path)
    seg = segment_times(marks)
    res = {"meta": meta, "pair": (a, b), "sensors": (sa, sb), "cycles": [], "slope": {},
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
    pair = [s for s in (sa, sb) if s in mm]

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
             "gap": {}}
        for s in pair:
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
    res["settled"] = settled_cycle(refs, tol_mm)

    if "pre-hold" in seg:
        t0, t1 = seg["pre-hold"]
        res["pre_hold"] = {s: drift(*mm[s], t0, t1) for s in ref_sensors}

    # Moving-branch slope: the last 40 % of each sweep ramp, past the dead band.
    for s in pair:
        slopes = []
        for k in range(2, ncyc + 1):
            for lab in ("to +", "to -"):
                if f"c{k} {lab}" not in seg:
                    continue
                t0, t1 = seg[f"c{k} {lab}"]
                ta = t0 + 0.6 * (t1 - t0)
                dd = interp(t1, t_cmd, d_cmd) - interp(ta, t_cmd, d_cmd)
                if dd:
                    slopes.append((interp(t1, *mm[s]) - interp(ta, *mm[s])) / dd)
        res["slope"][s] = sorted(slopes)[len(slopes) // 2] if slopes else None

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
    return "\n".join(lines)


def analyze(csv_path: str, slip_mm: float = 0.5, tol_mm: float = 0.1,
            exclude: Sequence[str] = ("Sup",), plots: bool = True) -> str:
    res = compute(csv_path, slip_mm, tol_mm, exclude)
    report = render(res, csv_path, slip_mm, tol_mm)
    base = os.path.splitext(csv_path)[0]
    with open(base + "_report.txt", "w") as fh:
        fh.write(report + "\n")
    if plots and not res["no_cycles"]:
        refs = [c["ref"] for c in res["cycles"]]
        _plots(base, res["mm"], res["t_cmd"], res["d_cmd"], res["seg"], len(res["cycles"]),
               *res["sensors"], *res["pair"], refs, res["ref_sensors"], res["name_of"])
    return report


def _plots(base, mm, t_cmd, d_cmd, seg, cycles, sa, sb, a, b, refs, sensors, name_of):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharex=True)
    cmap = plt.get_cmap("viridis")
    for k in range(1, cycles + 1):
        t0 = seg.get(f"c{k} to +", (None,))[0]
        t1 = seg.get(f"c{k} zero from -", (None, None))[1]
        if t0 is None or t1 is None:
            continue
        for ax, s in zip(axes, (sa, sb)):
            ts, ys = mm[s]
            pts = [(interp(t, t_cmd, d_cmd), y) for t, y in zip(ts, ys) if t0 <= t <= t1]
            if pts:
                ax.plot(*zip(*pts), color=cmap((k - 1) / max(1, cycles - 1)), lw=1,
                        label=f"cycle {k}")
    for ax, n in zip(axes, (a, b)):
        ax.set_title(n)
        ax.set_xlabel(f"d = (p({a}) − p({b}))/2 from the base  (kPa)")
        ax.set_ylabel("length change (mm)")
        ax.grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_loops.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ks = list(range(1, len(refs) + 1))
    for s in sensors:
        ax.plot(ks, [r.get(s, float("nan")) for r in refs], marker="o", label=name_of[s])
    ax.plot(ks, [r.get(sa, float("nan")) + r.get(sb, float("nan")) for r in refs],
            marker="s", ls="--", color="k", label=f"{a}+{b}")
    ax.set_xlabel("cycle")
    ax.set_ylabel("reference length (mm, end of 'zero from −')")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_settling.png", dpi=130)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Analyse a pair_sweep.py run.")
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
