#!/usr/bin/env python3
"""
finger/step_response.py

Open-loop pressure step response of ONE finger muscle. Stage 2 groundwork:
measure the plant before designing the loop around it.

Sequence (the tested muscle starts from its own idle pretension, idle_kpa in
the config; the other four sit at a low --others pressure, 10 kPa by default,
so the antagonists keep their tendons taut without stiffening the joints):

    idle ... settle
    for each level:  step ROLE -> level, hold      (rise, static gain)
                     step ROLE -> idle,  rest      (unloading, hysteresis)
    vent everything

The other four muscles stay at --others throughout; their length traces show
coupling. Lengths are logged for all five at the bus rate (~39 Hz) together
with the commanded pressure echoed by the Giga. At the end the script prints,
per level: settled contraction, static gain in mm/kPa, 63 % rise time, and
the residual after unloading. A CSV (and a PNG if --plot) go under runs/.

Safety: every level is checked against the role's ceiling and the idle
pressure against every ceiling before any port is opened; the config must
be ARMABLE for all five roles; everything is vented on exit, error or
Ctrl-C; the run aborts if the tested sensor stops answering for a second.
The Giga has no command-loss watchdog yet: stay at the bench while this runs.

    .venv/bin/python finger/step_response.py FDP --dry-run
    .venv/bin/python finger/step_response.py FDP
    .venv/bin/python finger/step_response.py DI --levels 30,60,90,120 --hold 6 --plot
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import config as C  # noqa: E402

RUNS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")
SENSOR_STALE_S = 1.0
ANALYSIS_WINDOW_S = 1.0


# ----------------------------------------------------------------------------
# Plan (pure, unit-tested)
# ----------------------------------------------------------------------------

@dataclass
class Plan:
    role: str
    levels: List[float]
    idle: Dict[str, float]          # tested role: its idle_kpa (or --idle); others: --others
    hold: float
    rest: float
    settle: float
    ramp: float
    cycles: int

    def duration(self) -> float:
        return self.settle + self.cycles * len(self.levels) * (self.hold + self.rest)


def default_levels(idle: float, ceiling: float) -> List[float]:
    """Four equal steps from idle up to the ceiling."""
    return [round(idle + (ceiling - idle) * k / 4, 1) for k in (1, 2, 3, 4)]


DEFAULT_OTHERS_KPA = 10.0


def make_plan(cfg: C.FingerConfig, role: str, levels: Optional[Sequence[float]],
              idle_override: Optional[float], hold: float, rest: float, settle: float,
              ramp: float, cycles: int, others: float = DEFAULT_OTHERS_KPA) -> Plan:
    if role not in C.ROLES:
        raise ValueError(f"unknown role {role!r}; choose from {list(C.ROLES)}")
    ceiling = cfg.muscles[role].ceiling_kpa
    if others < 0:
        raise ValueError("others pressure must be >= 0")
    for r in C.ROLES:
        if r != role and others > cfg.muscles[r].ceiling_kpa:
            raise ValueError(f"others {others:g} kPa exceeds {r} ceiling {cfg.muscles[r].ceiling_kpa:g}")
    idle = {r: (cfg.muscles[r].idle_kpa if r == role else others) for r in C.ROLES}
    if idle_override is not None:
        if not 0 <= idle_override <= ceiling:
            raise ValueError(f"{role}: idle {idle_override} kPa outside 0..{ceiling} (ceiling)")
        idle[role] = idle_override
    levels = list(levels) if levels else default_levels(idle[role], ceiling)
    if not levels:
        raise ValueError("no levels")
    for lv in levels:
        if not 0 <= lv <= ceiling:
            raise ValueError(f"{role}: level {lv} kPa outside 0..{ceiling} (ceiling)")
        if lv <= idle[role]:
            raise ValueError(f"{role}: level {lv} kPa is not above idle {idle[role]:g}")
    for name, v, lo in (("hold", hold, 0.5), ("rest", rest, 0.5), ("settle", settle, 0.5)):
        if v < lo:
            raise ValueError(f"{name} must be >= {lo} s")
    if ramp < 0 or cycles < 1:
        raise ValueError("ramp must be >= 0 and cycles >= 1")
    return Plan(role, levels, idle, hold, rest, settle, ramp, cycles)


# ----------------------------------------------------------------------------
# Recording
# ----------------------------------------------------------------------------

@dataclass
class Row:
    t: float                       # seconds since run start
    phase: str
    level: Optional[float]
    cmd_kpa: Dict[str, Optional[float]]
    mm: Dict[str, Optional[float]]
    counts: Dict[str, Optional[int]]
    ok: Dict[str, bool]


class Recorder:
    """Reads all five sensors as fast as the bus allows and tags each row with
    the current phase and the pressure the Giga last echoed for each muscle."""

    def __init__(self, bus, cfg: C.FingerConfig, valve, mv_to_kpa, test_role: str):
        self.bus, self.cfg, self.valve, self.mv_to_kpa = bus, cfg, valve, mv_to_kpa
        self.test_role = test_role
        self.rows: List[Row] = []
        self.phase, self.level = "init", None
        self.t0 = time.monotonic()
        self.stop = threading.Event()
        self.fault: Optional[str] = None
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def set_phase(self, phase: str, level: Optional[float]) -> None:
        with self.lock:
            self.phase, self.level = phase, level

    def _commands(self) -> Dict[str, Optional[float]]:
        with self.valve.display_lock:
            data = dict(self.valve.valve_data)
        out = {}
        for r in C.ROLES:
            mv = data.get(self.cfg.muscles[r].regulator)
            out[r] = None if mv is None else round(self.mv_to_kpa(mv), 2)
        return out

    def _loop(self) -> None:
        last_ok = time.monotonic()
        while not self.stop.is_set():
            mm, counts, ok = {}, {}, {}
            for r in C.ROLES:
                rd = self.bus.read(self.cfg.muscles[r].sensor)
                ok[r] = rd.ok
                counts[r] = rd.counts if rd.ok else None
                mm[r] = self.cfg.contraction_mm(r, rd.counts) if rd.ok else None
            now = time.monotonic()
            if ok[self.test_role]:
                last_ok = now
            elif now - last_ok > SENSOR_STALE_S:
                self.fault = f"{self.test_role} sensor silent for {now - last_ok:.1f} s"
                return
            with self.lock:
                phase, level = self.phase, self.level
            self.rows.append(Row(now - self.t0, phase, level, self._commands(), mm, counts, ok))

    def start(self) -> None:
        self.thread.start()

    def finish(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2)


def write_csv(rows: Sequence[Row], path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "phase", "level_kpa"]
                   + [f"cmd_{r}_kpa" for r in C.ROLES]
                   + [f"len_{r}_mm" for r in C.ROLES]
                   + [f"counts_{r}" for r in C.ROLES]
                   + [f"ok_{r}" for r in C.ROLES])
        for row in rows:
            w.writerow([f"{row.t:.4f}", row.phase, "" if row.level is None else row.level]
                       + ["" if row.cmd_kpa[r] is None else row.cmd_kpa[r] for r in C.ROLES]
                       + ["" if row.mm[r] is None else f"{row.mm[r]:.4f}" for r in C.ROLES]
                       + ["" if row.counts[r] is None else row.counts[r] for r in C.ROLES]
                       + [int(row.ok[r]) for r in C.ROLES])


# ----------------------------------------------------------------------------
# Analysis (pure, unit-tested)
# ----------------------------------------------------------------------------

@dataclass
class LevelResult:
    level: float
    baseline_mm: float
    settled_mm: float
    delta_mm: float
    gain_mm_per_kpa: Optional[float]
    t63_s: Optional[float]
    residual_mm: Optional[float]      # after unloading, relative to baseline
    coupling_mm: Dict[str, float] = field(default_factory=dict)  # max |move| of others


def _mean_last(rows: Sequence[Row], role: str, window: float) -> Optional[float]:
    if not rows:
        return None
    t_end = rows[-1].t
    vals = [r.mm[role] for r in rows if r.t >= t_end - window and r.mm[role] is not None]
    return statistics.fmean(vals) if vals else None


def analyze(rows: Sequence[Row], role: str, idle: float,
            window: float = ANALYSIS_WINDOW_S) -> List[LevelResult]:
    """`idle` is the tested role's own idle pressure (the step starts there)."""
    """Per (cycle, level): baseline from the tail of the preceding phase,
    settled value from the tail of the hold, rise time to 63 % of the change,
    residual from the tail of the rest phase."""
    # Group rows into consecutive phases in order.
    phases: List[List[Row]] = []
    for r in rows:
        if phases and phases[-1][0].phase == r.phase:
            phases[-1].append(r)
        else:
            phases.append([r])
    results = []
    for i, ph in enumerate(phases):
        if not ph[0].phase.startswith("step"):
            continue
        level = ph[0].level
        prev = phases[i - 1] if i > 0 else []
        nxt = phases[i + 1] if i + 1 < len(phases) and phases[i + 1][0].phase.startswith("rest") else []
        base = _mean_last(prev, role, window)
        settled = _mean_last(ph, role, window)
        if base is None or settled is None or level is None:
            continue
        delta = settled - base
        dp = level - idle
        gain = delta / dp if abs(dp) > 1e-9 else None
        t63 = None
        if abs(delta) > 1e-6:
            target = base + 0.632 * delta
            t_step = ph[0].t
            for r in ph:
                v = r.mm[role]
                if v is None:
                    continue
                if (delta > 0 and v >= target) or (delta < 0 and v <= target):
                    t63 = r.t - t_step
                    break
        after = _mean_last(nxt, role, window)
        residual = None if after is None else after - base
        coupling = {}
        for other in C.ROLES:
            if other == role:
                continue
            ob = _mean_last(prev, other, window)
            vals = [r.mm[other] for r in ph if r.mm[other] is not None]
            if ob is not None and vals:
                coupling[other] = max(abs(v - ob) for v in vals)
        results.append(LevelResult(level, base, settled, delta, gain, t63, residual, coupling))
    return results


def print_results(results: Sequence[LevelResult], role: str, idle: float) -> None:
    if not results:
        print("no complete step phases to analyze")
        return
    print(f"\n{role}: step from idle {idle:g} kPa   (contraction +ve = shortening)")
    print(f"{'level':>7} {'settled':>9} {'delta':>8} {'gain':>10} {'t63':>7} {'residual':>9}  coupling (max |mm| of others)")
    print(f"{'kPa':>7} {'mm':>9} {'mm':>8} {'mm/kPa':>10} {'s':>7} {'mm':>9}")
    for r in results:
        g = "-" if r.gain_mm_per_kpa is None else f"{r.gain_mm_per_kpa:.4f}"
        t = "-" if r.t63_s is None else f"{r.t63_s:.2f}"
        res = "-" if r.residual_mm is None else f"{r.residual_mm:+.3f}"
        cp = " ".join(f"{k}={v:.2f}" for k, v in r.coupling_mm.items())
        print(f"{r.level:>7g} {r.settled_mm:>9.3f} {r.delta_mm:>+8.3f} {g:>10} {t:>7} {res:>9}  {cp}")
    gains = [r.gain_mm_per_kpa for r in results if r.gain_mm_per_kpa is not None]
    if gains:
        print(f"\nmean static gain {statistics.fmean(gains):.4f} mm/kPa "
              f"(range {min(gains):.4f}..{max(gains):.4f})")


def plot(rows: Sequence[Row], role: str, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r.t for r in rows]
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6),
                                 gridspec_kw={"height_ratios": [2, 1]})
    for r in C.ROLES:
        y = [row.mm[r] if row.mm[r] is not None else float("nan") for row in rows]
        a1.plot(t, y, lw=1.6 if r == role else 0.8, label=r, alpha=1 if r == role else 0.6)
    a1.set_ylabel("contraction (mm)")
    a1.legend(loc="upper right", fontsize=8)
    a1.grid(alpha=0.3)
    a2.step(t, [row.cmd_kpa[role] if row.cmd_kpa[role] is not None else float("nan") for row in rows],
            where="post", color="k", lw=1)
    a2.set_ylabel(f"{role} cmd (kPa)")
    a2.set_xlabel("time (s)")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"plot: {path}")


# ----------------------------------------------------------------------------
# Run
# ----------------------------------------------------------------------------

def describe(plan: Plan, cfg: C.FingerConfig) -> str:
    m = cfg.muscles[plan.role]
    others = ", ".join(f"{r}=V{cfg.muscles[r].regulator}@{plan.idle[r]:g}" for r in C.ROLES if r != plan.role)
    return "\n".join([
        f"role      : {plan.role} = regulator {m.regulator}, sensor {m.sensor}, "
        f"ceiling {m.ceiling_kpa:g} kPa",
        f"levels    : {plan.levels} kPa, {'step' if plan.ramp == 0 else f'{plan.ramp:g} s ramp'}",
        f"idle      : {plan.role} {plan.idle[plan.role]:g} kPa; others {others} kPa (low, to avoid co-contraction)",
        f"timing    : settle {plan.settle:g} s, hold {plan.hold:g} s, rest {plan.rest:g} s, "
        f"{plan.cycles} cycle(s) -> about {plan.duration():.0f} s",
    ])


def run(plan: Plan, cfg: C.FingerConfig, valve_port: str, rs485_port: str,
        log_path: str, want_plot: bool) -> int:
    from valve_controller import ValveController, mv_to_kpa
    from finger.rs485 import Bus, open_port

    reg = {r: cfg.muscles[r].regulator for r in C.ROLES}
    all_idle = [(reg[r], plan.idle[r]) for r in C.ROLES]
    idle = plan.idle[plan.role]

    print(f"opening RS-485 {rs485_port} ...")
    bus = Bus(open_port(rs485_port, cfg.baud), cfg.counts_per_turn)
    print(f"opening valves {valve_port} ...")
    valve = ValveController(valve_port)
    rec = Recorder(bus, cfg, valve, mv_to_kpa, plan.role)

    def sleep_watch(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if rec.fault:
                raise RuntimeError(rec.fault)
            time.sleep(0.02)

    rc = 1
    try:
        valve.emergency_stop()
        time.sleep(0.3)
        rec.start()
        print(f"{plan.role} at idle {idle:g} kPa, others at {plan.idle[[r for r in C.ROLES if r != plan.role][0]]:g} kPa, "
              f"settling {plan.settle:g} s ...")
        rec.set_phase("idle", None)
        valve.set_multiple_valves(all_idle, ramp=min(2.0, plan.settle / 2))
        sleep_watch(plan.settle)
        for cycle in range(plan.cycles):
            for lv in plan.levels:
                tag = f"c{cycle}"
                print(f"[{rec.rows[-1].t if rec.rows else 0:6.1f}s] {plan.role} -> {lv:g} kPa, hold {plan.hold:g} s")
                rec.set_phase(f"step_{tag}_{lv:g}", lv)
                valve.set_valve(reg[plan.role], lv, ramp=plan.ramp)
                sleep_watch(plan.hold)
                print(f"[{rec.rows[-1].t:6.1f}s] {plan.role} -> idle {idle:g} kPa, rest {plan.rest:g} s")
                rec.set_phase(f"rest_{tag}_{lv:g}", lv)
                valve.set_valve(reg[plan.role], idle, ramp=plan.ramp)
                sleep_watch(plan.rest)
        rec.set_phase("done", None)
        rc = 0
    except KeyboardInterrupt:
        print("\n** Ctrl-C: venting **")
        rc = 130
    except Exception as exc:
        print(f"\n** {exc}: venting **")
        rc = 1
    finally:
        try:
            valve.emergency_stop()
        finally:
            rec.finish()
            time.sleep(0.2)
            try:
                valve.running = False
                if valve.ser:
                    valve.ser.close()
            except Exception:
                pass
            bus.close()

    if rec.rows:
        write_csv(rec.rows, log_path)
        print(f"log: {log_path}  ({len(rec.rows)} rows, "
              f"{len(rec.rows) / max(rec.rows[-1].t, 1e-9):.1f} Hz)")
        print_results(analyze(rec.rows, plan.role, idle), plan.role, idle)
        if want_plot:
            try:
                plot(rec.rows, plan.role, os.path.splitext(log_path)[0] + ".png")
            except Exception as exc:
                print(f"plot failed: {exc}")
    return rc


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Open-loop pressure step response of one finger muscle.")
    ap.add_argument("role", help="ED FDS FDP DI PI")
    ap.add_argument("--levels", help="comma-separated kPa levels (default: 4 equal steps idle..ceiling)")
    ap.add_argument("--idle", type=float, default=None,
                    help="override the tested muscle's idle_kpa from the config")
    ap.add_argument("--others", type=float, default=DEFAULT_OTHERS_KPA,
                    help="pressure on the four untested muscles (kPa, default 10)")
    ap.add_argument("--hold", type=float, default=5.0, help="seconds at each level")
    ap.add_argument("--rest", type=float, default=5.0, help="seconds back at idle after each level")
    ap.add_argument("--settle", type=float, default=4.0, help="seconds at idle before the first step")
    ap.add_argument("--ramp", type=float, default=0.0, help="ramp time per transition (0 = step)")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--config", default=C.DEFAULT_CONFIG_PATH)
    ap.add_argument("--mapping", default=C.DEFAULT_MAPPING_PATH)
    ap.add_argument("--valve-port", help="override the valve port from the config")
    ap.add_argument("--rs485-port", help="override the RS-485 port from the config")
    ap.add_argument("--log", help="CSV path (default runs/step_<role>_<time>.csv)")
    ap.add_argument("--plot", action="store_true", help="also write a PNG next to the CSV")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    ap.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args(argv)

    role = args.role.upper()
    try:
        cfg = C.load(args.config, args.mapping)
        cfg.require_armable()
        levels = [float(x) for x in args.levels.split(",")] if args.levels else None
        plan = make_plan(cfg, role, levels, args.idle, args.hold, args.rest, args.settle,
                         args.ramp, args.cycles, args.others)
    except (C.ConfigError, ValueError) as exc:
        print(f"[ERR] {exc}")
        return 1

    print(describe(plan, cfg))
    if args.dry_run:
        print("[dry-run] no hardware touched")
        return 0
    if not args.yes:
        ans = input("\nThe Giga has no watchdog: stay at the bench. Start? [y/N] ").strip().lower()
        if ans != "y":
            print("cancelled")
            return 0
    log_path = args.log or os.path.join(RUNS_DIR, f"step_{role}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    return run(plan, cfg, args.valve_port or cfg.valve_port, args.rs485_port or cfg.rs485_port,
               log_path, args.plot)


if __name__ == "__main__":
    raise SystemExit(main())
