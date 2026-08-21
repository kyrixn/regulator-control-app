#!/usr/bin/env python3
"""
fcr_square_test.py

Square-wave step response of FCR alone, with every other muscle vented.

FCR is stepped between two setpoints — no ramping, the setpoint is simply
rewritten at each half-period — for a fixed burst:

    low  = 1800 mV        high = 2300 mV
    2 Hz  ->  0.25 s at each setpoint, 4 full cycles in 2 s

Every other muscle (FCU, FDS, ECU, ECR, PT) is commanded off for the whole
run, so this measures FCR's own dynamics against the passive structure rather
than against antagonist pressure. That makes it NOT directly comparable with
fcu_fcr_cycle_test.py, where the other four are held pressurised.

Only FCR's sensor (59) is polled, so the RS-485 bus carries one slave and
answers as fast as it can (~150-200 Hz at 115200 baud for the 16-register
state block) instead of being shared across six. The sampler still runs on its
own clock, so rows can repeat a sensor value between bus updates; the `fresh`
column marks the rows where the encoder actually advanced, and the run reports
the effective sensor rate.

Sequence
--------
    vent everything  ->  FCR to low, settle 2 s  ->  ZERO sensor 59
    [recording starts]
    4 x ( high for 0.25 s, low for 0.25 s )        <- exactly 2 s of driving
    tail: hold low a further 0.5 s                 <- captures the last return
    vent FCR

Output
------
Written to phase0_test/phase0_data/wrist_flex/ (git-ignored), same as the
cycle test:

    fcr_square_2hz_<stamp>.csv          raw samples, length to 4 decimals
    fcr_square_2hz_<stamp>_steps.png    3-panel summary figure

Nothing is written unless the run completes — Ctrl-C, SIGTERM, the stop-file
and the runtime watchdog all discard the buffer.

Emergency stop and port/mapping handling are shared with fcu_fcr_cycle_test.py
rather than duplicated, so both scripts stop the same way:

    Ctrl-C  |  kill <pid>  |  touch phase0_test/STOP  |  runtime watchdog

app.py must NOT be running — it would already hold both serial ports.

Usage (runnable from any working directory):
    python phase0_test/fcr_square_test.py
    python phase0_test/fcr_square_test.py --freq 2 --duration 2
    python phase0_test/fcr_square_test.py --low 1800 --high 2300 --no-live
    python phase0_test/fcr_square_test.py --dry-run
    python phase0_test/fcr_square_test.py --plot <file>.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# The e-stop machinery, port detection and mapping lookup are generic; reusing
# them keeps the two tests stopping and resolving hardware identically.
import fcu_fcr_cycle_test as rig
from fcu_fcr_cycle_test import (
    ABORT,
    Aborted,
    DEFAULT_OUT_DIR,
    DEFAULT_STOP_FILE,
    LEN_DECIMALS,
    RUNTIME_MARGIN_S,
    find_ports,
    install_signal_handlers,
    resolve_muscles,
    sleep_or_abort,
    start_watchdog,
    trigger_abort,
    wait_online,
)


# ---- Test definition ------------------------------------------------------

MOVING = "FCR"                  # the only muscle driven
VENTED = ("FCU", "FDS", "ECU", "ECR", "PT")   # commanded off throughout

LOW_MV = 1800
HIGH_MV = 2300
FREQ_HZ = 2.0                   # full cycles per second
DURATION_S = 2.0                # length of the square-wave burst
SETTLE_S = 2.0                  # dwell at low before zeroing
TAIL_S = 0.5                    # extra recording at low after the last step
SAMPLE_HZ = 200.0               # sampler clock; the bus is the real limit

CSV_COLUMNS = [
    "wall_time", "elapsed_s", "cycle", "segment",
    "target_FCR", "cmd_FCR_mV", "len_FCR_mm", "abs_FCR_counts",
    "fresh", "online_FCR",
]

I_T = CSV_COLUMNS.index("elapsed_s")
I_CYCLE = CSV_COLUMNS.index("cycle")
I_SEGMENT = CSV_COLUMNS.index("segment")
I_LEN = CSV_COLUMNS.index("len_FCR_mm")
I_CMD = CSV_COLUMNS.index("cmd_FCR_mV")
I_TARGET = CSV_COLUMNS.index("target_FCR")


# ---- Recorder -------------------------------------------------------------

class Recorder:
    """Single-sensor sampler. Length is derived from raw counts, not from
    EncoderController.get_state()["position_mm"], which is rounded to 2 dp."""

    def __init__(self, valve, encoder, regulator, sensor, zero_counts,
                 counts_per_turn, drum_diameter_mm, sample_hz):
        self.valve = valve
        self.encoder = encoder
        self.regulator = regulator
        self.sensor = sensor
        self.zero = zero_counts
        self.mm_per_count = math.pi * drum_diameter_mm / counts_per_turn
        self.period = 1.0 / sample_hz
        self.rows = []
        self._stop = threading.Event()
        self._thread = None
        self._ctx_lock = threading.Lock()
        self._ctx = {"cycle": 0, "segment": "settle", "target": LOW_MV}
        self._last_counts = None
        self.t0 = None

    def set_context(self, cycle, segment, target):
        with self._ctx_lock:
            self._ctx = {"cycle": cycle, "segment": segment, "target": target}

    def _loop(self):
        while not self._stop.is_set():
            start = time.perf_counter()
            with self._ctx_lock:
                c = dict(self._ctx)
            with self.valve.display_lock:
                cmd = self.valve.valve_data.get(self.regulator)
            entry = {}
            for e in self.encoder.get_state().get("encoders", []):
                if e["slave"] == self.sensor:
                    entry = e
                    break
            counts = entry.get("absolute_position")
            online = bool(entry.get("online"))
            fresh = counts is not None and counts != self._last_counts
            if counts is not None:
                self._last_counts = counts
            length = (None if counts is None
                      else round((counts - self.zero) * self.mm_per_count,
                                 LEN_DECIMALS))
            self.rows.append([
                round(time.time(), 3), round(start - self.t0, 4),
                c["cycle"], c["segment"], c["target"], cmd, length, counts,
                fresh, online,
            ])
            rest = self.period - (time.perf_counter() - start)
            if rest > 0:
                time.sleep(rest)

    def start(self):
        self.t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)


# ---- Live view ------------------------------------------------------------

class LivePlot:
    """FCR length + commanded setpoint, drawn on the main thread from the
    sequence's waits (see fcu_fcr_cycle_test.LivePlot for the rationale)."""

    def __init__(self, rec, span_s, fps, low, high):
        self.rec = rec
        self.span = span_s
        self.low, self.high = low, high
        self.period = 1.0 / max(1.0, fps)
        self.fig = None
        self.dead = False
        self._next = 0.0

    def open(self):
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            if matplotlib.get_backend().lower() == "agg":
                print("[WARN] no interactive matplotlib backend; live view off")
                return False
            plt.ion()
            self.fig, (self.ax_len, self.ax_p) = plt.subplots(
                2, 1, figsize=(10, 7), sharex=True,
                gridspec_kw={"height_ratios": [1.6, 1.0]})
            self.line, = self.ax_len.plot([], [], color="#d62728", lw=1.4,
                                          label="FCR (sensor 59)")
            self.ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
            self.ax_len.set_ylabel("length (mm, rel. low)")
            self.ax_len.legend(loc="upper right", fontsize=9)
            self.ax_len.grid(alpha=0.3)

            self.cmd, = self.ax_p.step([], [], where="post", color="#333",
                                       lw=1.2, label="FCR cmd")
            for mv in (self.low, self.high):
                self.ax_p.axhline(mv, color="0.75", lw=0.7)
            self.ax_p.set_ylim(self.low - 60, self.high + 60)
            self.ax_p.set_ylabel("setpoint (mV)")
            self.ax_p.set_xlabel("time since zero (s)")
            self.ax_p.legend(loc="upper right", fontsize=8)
            self.ax_p.grid(alpha=0.3)
            for ax in (self.ax_len, self.ax_p):
                ax.set_xlim(0, self.span)
            self.fig.canvas.manager.set_window_title("FCR square-wave live")
            self.fig.tight_layout()
            self._draw()
            return True
        except Exception as exc:
            print(f"[WARN] could not open live view ({exc}); continuing without it")
            self.fig = None
            return False

    def _draw(self):
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def update(self, force=False):
        if self.fig is None or self.dead:
            return
        now = time.monotonic()
        if not force and now < self._next:
            return
        self._next = now + self.period
        rows = self.rec.rows
        n = len(rows)
        if n == 0:
            return
        view = rows[:n]
        nan = float("nan")
        num = lambda r, i: r[i] if isinstance(r[i], (int, float)) else nan
        t = [r[I_T] for r in view]
        self.line.set_data(t, [num(r, I_LEN) for r in view])
        self.cmd.set_data(t, [num(r, I_TARGET) for r in view])
        self.ax_len.relim()
        self.ax_len.autoscale_view(scalex=False, scaley=True)
        last = view[-1]
        self.ax_len.set_title(
            f"cycle {last[I_CYCLE]}  —  {last[I_SEGMENT]}  —  t={t[-1]:.2f}s",
            fontsize=10)
        try:
            self._draw()
        except Exception as exc:
            print(f"[WARN] live view stopped ({exc}); run continues")
            self.dead = True

    def close(self):
        if self.fig is None:
            return
        try:
            import matplotlib.pyplot as plt
            self.update(force=True)
            plt.ioff()
            plt.close(self.fig)
        except Exception:
            pass
        self.fig = None


# ---- Helpers --------------------------------------------------------------

def capture_zero(encoder, sensor):
    for e in encoder.get_state().get("encoders", []):
        if e["slave"] == sensor:
            counts = e.get("absolute_position")
            if counts is None:
                return None, f"sensor {sensor} reported no absolute position"
            return counts, None
    return None, f"sensor {sensor} not present in the scan"


def save_csv(rows, out_dir, freq):
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(out_dir, f"fcr_square_{freq:g}hz_{stamp}.csv")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_COLUMNS)
        w.writerows(rows)
    return path


def describe_plan(args):
    half = 0.5 / args.freq
    cycles = args.freq * args.duration
    return "\n".join([
        f"Driven muscle    : {MOVING} — stepped, no ramping",
        f"Setpoints        : low {args.low} mV  <->  high {args.high} mV",
        f"Square wave      : {args.freq:g} Hz  ({half * 1000:.0f} ms at each "
        f"setpoint), {args.duration:g}s = {cycles:g} cycles",
        f"Vented throughout: {', '.join(VENTED)}",
        f"Before recording : hold low {args.settle:g}s, then ZERO sensor",
        f"After the burst  : hold low a further {args.tail:g}s (recorded)",
        f"Sampler          : {args.sample_hz:g} Hz, length to {LEN_DECIMALS} dp",
    ])


# ---- Run ------------------------------------------------------------------

def run(args):
    from valve_controller import ValveController
    from encoder_controller import EncoderController

    muscles = resolve_muscles(args.mapping)
    missing = [n for n in (MOVING, *VENTED) if n not in muscles]
    if missing:
        print(f"[ERR] sensor_mapping.json has no entry for: {', '.join(missing)}")
        return 2
    reg = muscles[MOVING]["regulator"]
    sensor = muscles[MOVING]["sensor"]
    if sensor is None:
        print(f"[ERR] {MOVING} has no sensor mapped; it is the muscle measured")
        return 2

    print(describe_plan(args))
    print(f"\n{MOVING} = regulator {reg}, sensor {sensor}")
    print("vented: " + ", ".join(
        f"{n} (R{muscles[n]['regulator']})" for n in VENTED))

    if args.dry_run:
        print("\n[dry-run] no hardware touched.")
        return 0

    valve_port, rs485_port = find_ports(args.valve_port, args.rs485_port)
    if not valve_port or not rs485_port:
        print(f"\n[ERR] need both ports (valve={valve_port}, rs485={rs485_port}). "
              "Pass --valve-port / --rs485-port, and make sure app.py isn't running.")
        return 2
    print(f"\nValve regulator : {valve_port}\nRS-485 sensor   : {rs485_port}")

    valve = ValveController(port=valve_port)
    if not valve.connected:
        print(f"[ERR] could not open valve regulator on {valve_port}")
        return 1

    stop_file = None if args.no_stop_file else args.stop_file
    if stop_file and os.path.exists(stop_file):
        print(f"[WARN] removing stale stop-file {stop_file}")
        try:
            os.remove(stop_file)
        except OSError as exc:
            print(f"[ERR] could not remove it ({exc}); the run would abort at once")
            valve.close()
            return 2
    planned = args.settle + args.duration + args.tail
    max_runtime = (args.max_runtime if args.max_runtime > 0
                   else planned + RUNTIME_MARGIN_S)
    install_signal_handlers(valve)
    start_watchdog(valve, stop_file, max_runtime)
    print("\nE-STOP:  Ctrl-C  |  kill %d  |  touch %s   (watchdog at %.0fs)"
          % (os.getpid(), stop_file or "(disabled)", max_runtime))

    encoder = EncoderController(
        port=rs485_port,
        slave_ids=[sensor],     # one slave only: maximum bus rate
        counts_per_turn=args.counts_per_turn,
        drum_diameter_mm=args.drum_diameter,
        interval=0.0,           # spin as fast as the encoder answers
    )
    if not encoder.connected:
        print(f"[ERR] could not open RS-485 adapter on {rs485_port}")
        valve.close()
        return 1

    all_regs = [reg] + [muscles[n]["regulator"] for n in VENTED]
    rec = None
    interrupted = False
    csv_path = None
    try:
        print(f"Waiting for sensor {sensor} to come online...")
        if not wait_online(encoder, [sensor]):
            print(f"[ERR] sensor {sensor} did not come online. Check wiring / id.")
            return 1

        # Vent everything, including FCR, so the run starts from a known state.
        valve.set_multiple_valves([(r, "off") for r in all_regs], ramp=0.0)
        sleep_or_abort(0.3)

        print(f"Holding {MOVING} at {args.low} for {args.settle:g}s...")
        valve.set_valve(reg, args.low, ramp=0.0)
        sleep_or_abort(args.settle)

        zero_counts, zerr = capture_zero(encoder, sensor)
        if zero_counts is None:
            print(f"[ERR] could not zero: {zerr}")
            return 1
        print(f"Zeroed at {args.low}: {zero_counts} counts. Recording started.")

        rec = Recorder(valve, encoder, reg, sensor, zero_counts,
                       args.counts_per_turn, args.drum_diameter, args.sample_hz)
        rec.start()

        if not args.no_live:
            live = LivePlot(rec, args.duration + args.tail, args.live_fps,
                            args.low, args.high)
            if live.open():
                rig._live = live      # sleep_or_abort pumps it
                print(f"Live view open ({args.live_fps:g} fps). "
                      "Closing the window does not stop the run — use Ctrl-C.")

        # --- the square wave: rewrite the setpoint at each half-period. ----
        half = 0.5 / args.freq
        n_steps = int(round(args.duration / half))
        print(f"Driving {args.freq:g} Hz for {args.duration:g}s "
              f"({n_steps} steps of {half * 1000:.0f} ms)...")
        t0 = time.perf_counter()
        for k in range(n_steps):
            mv = args.high if k % 2 == 0 else args.low
            cycle = k // 2 + 1
            rec.set_context(cycle, "high" if k % 2 == 0 else "low", mv)
            valve.set_valve(reg, mv, ramp=0.0)      # step, never a ramp
            # Absolute deadline, so per-step overhead cannot accumulate drift.
            sleep_or_abort(max(0.0, t0 + (k + 1) * half - time.perf_counter()))

        if args.tail > 0:
            rec.set_context(n_steps // 2, "tail", args.low)
            valve.set_valve(reg, args.low, ramp=0.0)
            sleep_or_abort(args.tail)

        actual = time.perf_counter() - t0
        print(f"Burst done in {actual:.3f}s (target {args.duration:g}s).")

    except Aborted as exc:
        interrupted = True
        print(f"[STOP] run aborted ({exc}) — valves vented, nothing saved.")
    except KeyboardInterrupt:
        interrupted = True
        print("\n[STOP] interrupted — emergency stop.")
        trigger_abort(valve, "KeyboardInterrupt")
    finally:
        if rig._live is not None:
            rig._live.close()
            rig._live = None
        if rec is not None:
            rec.stop()
        try:
            if interrupted or ABORT.is_set():
                valve.emergency_stop()
            else:
                valve.set_multiple_valves([(r, "off") for r in all_regs], ramp=0.0)
        except Exception as exc:
            print(f"[WARN] could not vent valves: {exc}")
            print("[ERR] CHECK THE RIG — cut the air supply if still pressurised.")
        time.sleep(0.2)

        if interrupted or ABORT.is_set():
            n = len(rec.rows) if rec is not None else 0
            print(f"[STOP] discarded {n} sample(s); nothing written to "
                  f"{os.path.abspath(args.out)}")
        elif rec is not None and rec.rows:
            csv_path = save_csv(rec.rows, args.out, args.freq)
            print(f"\nSaved {len(rec.rows)} samples to {os.path.abspath(csv_path)}")

        valve.close()
        encoder.close()

    if csv_path and not args.no_plot:
        plot_csv(csv_path, show=args.show)
    return 0


# ---- Plotting -------------------------------------------------------------

def _read_csv(path):
    import numpy as np

    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"[ERR] {path} has no data rows")

    def col(name):
        out = []
        for r in rows:
            try:
                out.append(float(r[name]))
            except (ValueError, TypeError, KeyError):
                out.append(float("nan"))
        return np.array(out)

    return {
        "t": col("elapsed_s"),
        "cycle": np.array([int(float(r["cycle"])) for r in rows]),
        "segment": [r["segment"] for r in rows],
        "len": col("len_FCR_mm"),
        "cmd": col("cmd_FCR_mV"),
        "target": col("target_FCR"),
        "fresh": [r.get("fresh", "") == "True" for r in rows],
    }


def plot_csv(path, show=False):
    import matplotlib
    if not show and "matplotlib.pyplot" not in sys.modules:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    d = _read_csv(path)
    t, cyc = d["t"], d["cycle"]

    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.5, 0.8, 1.2], hspace=0.35)
    ax_len = fig.add_subplot(gs[0])
    ax_p = fig.add_subplot(gs[1], sharex=ax_len)
    ax_ov = fig.add_subplot(gs[2])
    C = "#d62728"

    # -- length vs time, high half-periods shaded ---------------------------
    for i in range(len(t) - 1):
        if d["segment"][i] == "high":
            ax_len.axvspan(t[i], t[i + 1], color="#ffe9c7", lw=0, zorder=0)
    ax_len.plot(t, d["len"], color=C, lw=1.4, label="FCR (sensor 59)")
    ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
    ax_len.set_ylabel("length (mm, rel. low)")
    ax_len.set_title(f"FCR square-wave step response — {os.path.basename(path)}\n"
                     "shaded = commanded high, others vented")
    ax_len.legend(loc="best", fontsize=9)
    ax_len.grid(alpha=0.3)

    # -- commanded setpoint -------------------------------------------------
    ax_p.step(t, d["target"], where="post", color="0.25", lw=1.2, label="target")
    ax_p.step(t, d["cmd"], where="post", color=C, lw=1.0, ls="--",
              label="acknowledged")
    ax_p.set_ylabel("setpoint (mV)")
    ax_p.set_xlabel("time since zero (s)")
    ax_p.legend(loc="best", fontsize=8, ncol=2)
    ax_p.grid(alpha=0.3)

    # -- cycles overlaid ----------------------------------------------------
    cycles = [c for c in sorted(set(cyc.tolist())) if c > 0
              and np.any((cyc == c) & np.array([s in ("high", "low")
                                                for s in d["segment"]]))]
    cmap = plt.get_cmap("viridis")
    for n, c in enumerate(cycles):
        idx = np.where(cyc == c)[0]
        if not len(idx):
            continue
        ax_ov.plot(t[idx] - t[idx[0]], d["len"][idx], lw=1.3,
                   color=cmap(n / max(1, len(cycles) - 1)), label=f"cycle {c}")
    ax_ov.set_title("cycles overlaid", fontsize=10, color=C)
    ax_ov.set_xlabel("time within cycle (s)")
    ax_ov.set_ylabel("length (mm)")
    ax_ov.grid(alpha=0.3)
    ax_ov.legend(fontsize=8)

    png = os.path.splitext(path)[0] + "_steps.png"
    fig.savefig(png, dpi=140, bbox_inches="tight")
    print(f"Saved plot to {os.path.abspath(png)}")

    _print_stats(d, cycles)
    if show:
        plt.show()
    plt.close(fig)
    return png


def _print_stats(d, cycles):
    import numpy as np

    span = d["t"][-1] - d["t"][0] if len(d["t"]) > 1 else 0.0
    n_fresh = sum(d["fresh"])
    if span > 0:
        print(f"\nSampler {len(d['t']) / span:.0f} Hz, "
              f"{n_fresh} fresh encoder update(s) = {n_fresh / span:.0f} Hz "
              f"effective sensor rate")

    print(f"\nPer-cycle length extremes (mm, {LEN_DECIMALS} dp)")
    print(f"{'cycle':>5}  {'min':>10} {'max':>10} {'peak-peak':>10}")
    pps = []
    for c in cycles:
        v = d["len"][d["cycle"] == c]
        v = v[~np.isnan(v)]
        if not len(v):
            continue
        pps.append(v.max() - v.min())
        print(f"{c:>5}  {v.min():>10.4f} {v.max():>10.4f} "
              f"{v.max() - v.min():>10.4f}")
    if len(pps) > 1:
        pp = np.array(pps)
        print(f"\nStroke {pp.mean():.4f} mm mean, spread {pp.max() - pp.min():.4f} mm "
              f"across {len(pp)} cycles")
        print("If the stroke is far below the quasi-static travel between these "
              "two setpoints,\nthe regulator is not keeping up at this frequency.")


# ---- CLI ------------------------------------------------------------------

def build_parser():
    from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM

    p = argparse.ArgumentParser(
        description="FCR square-wave step response (others vented)")
    p.add_argument("--valve-port", default=None, help="Giga R1 regulator port")
    p.add_argument("--rs485-port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--low", type=int, default=LOW_MV, help="low setpoint (mV)")
    p.add_argument("--high", type=int, default=HIGH_MV, help="high setpoint (mV)")
    p.add_argument("--freq", type=float, default=FREQ_HZ,
                   help="square-wave frequency in Hz (default 2)")
    p.add_argument("--duration", type=float, default=DURATION_S,
                   help="length of the burst in seconds (default 2)")
    p.add_argument("--settle", type=float, default=SETTLE_S,
                   help="hold at low before zeroing (s)")
    p.add_argument("--tail", type=float, default=TAIL_S,
                   help="extra recording at low after the burst (s)")
    p.add_argument("--sample-hz", type=float, default=SAMPLE_HZ,
                   help="sampler rate (default 200)")
    p.add_argument("--out", default=DEFAULT_OUT_DIR,
                   help="directory for the CSV and PNG (created if absent)")
    p.add_argument("--mapping", default=None, help="path to sensor_mapping.json")
    p.add_argument("--counts-per-turn", type=lambda v: int(v, 0),
                   default=DEFAULT_COUNTS_PER_TURN)
    p.add_argument("--drum-diameter", type=float, default=DEFAULT_DRUM_DIAMETER_MM)
    p.add_argument("--stop-file", default=DEFAULT_STOP_FILE,
                   help="touch this path from another terminal to e-stop a run")
    p.add_argument("--no-stop-file", action="store_true",
                   help="disable the stop-file watchdog")
    p.add_argument("--max-runtime", type=float, default=0.0,
                   help="hard ceiling in seconds before the run self-aborts")
    p.add_argument("--dry-run", action="store_true",
                   help="resolve the mapping and print the plan, touch no hardware")
    p.add_argument("--no-live", action="store_true", help="skip the live window")
    p.add_argument("--live-fps", type=float, default=20.0,
                   help="live view redraw rate (default 20)")
    p.add_argument("--no-plot", action="store_true",
                   help="record the CSV but skip plotting")
    p.add_argument("--show", action="store_true",
                   help="open the plot window as well as saving the PNG")
    p.add_argument("--plot", metavar="CSV", default=None,
                   help="plot an existing CSV and exit (no hardware)")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.plot:
        plot_csv(args.plot, show=args.show)
        return 0
    if args.freq <= 0 or args.duration <= 0:
        print("[ERR] --freq and --duration must be positive")
        return 2
    if args.high <= args.low:
        print("[ERR] --high must exceed --low")
        return 2
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
