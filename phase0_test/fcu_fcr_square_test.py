#!/usr/bin/env python3
"""
fcu_fcr_square_test.py

Square-wave step response of FCU and FCR together, with the other four muscles
vented.

Both muscles are stepped between the same two setpoints, in phase and in one
batched serial command so they move together — no ramping, the setpoints are
simply rewritten at each half-period:

    low = 1800 mV        high = 2300 mV
    2 Hz  ->  0.25 s at each setpoint, 8 full cycles in 4 s

FDS, ECU, ECR and PT are commanded off for the whole run, so this measures the
two flexors against the passive structure rather than against extensor
pressure. That makes it NOT directly comparable with fcu_fcr_cycle_test.py,
where those four are held pressurised.

Sampling
--------
Both sensors are live (FCU = 58, FCR = 59), so the RS-485 bus alternates
between two slaves and each one updates at roughly half the single-slave rate
(~75-100 Hz at 115200 baud for the 16-register state block). The sampler runs
on its own 200 Hz clock, so rows can repeat a sensor value between bus
updates; the `fresh_*` columns mark the rows where that encoder actually
advanced, and the run reports the effective per-sensor rate so the true
resolution is visible rather than assumed.

Sequence
--------
    vent everything  ->  both to low, settle 2 s  ->  ZERO sensors 58 and 59
    [recording starts]
    8 x ( high for 0.25 s, low for 0.25 s )        <- exactly 4 s of driving
    tail: hold low a further 0.5 s                 <- captures the last return
    vent both

Output
------
Written to phase0_test/phase0_data/wrist_flex/ (git-ignored):

    fcu_fcr_square_2hz_<stamp>.csv          raw samples, length to 4 decimals
    fcu_fcr_square_2hz_<stamp>_steps.png    3-panel summary figure

Nothing is written unless the run completes — Ctrl-C, SIGTERM, the stop-file
and the runtime watchdog all discard the buffer. A completed run then asks
before writing anything — answer y to keep it, anything else (including a bare
Enter) throws it away. Pass -y to skip the question; a non-interactive run
keeps its data rather than prompting.

Emergency stop and port/mapping handling are shared with fcu_fcr_cycle_test.py
rather than duplicated, so both scripts stop the same way:

    Ctrl-C  |  kill <pid>  |  touch phase0_test/STOP  |  runtime watchdog

app.py must NOT be running — it would already hold both serial ports.

Usage (runnable from any working directory):
    python phase0_test/fcu_fcr_square_test.py
    python phase0_test/fcu_fcr_square_test.py --freq 2 --duration 4
    python phase0_test/fcu_fcr_square_test.py --low 1800 --high 2300 --no-live
    python phase0_test/fcu_fcr_square_test.py --dry-run
    python phase0_test/fcu_fcr_square_test.py --plot <file>.csv
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
    confirm_save,
    find_ports,
    install_signal_handlers,
    resolve_muscles,
    sleep_or_abort,
    start_watchdog,
    trigger_abort,
    wait_online,
)


# ---- Test definition ------------------------------------------------------

MOVING = ("FCU", "FCR")                 # stepped together, in phase
VENTED = ("FDS", "ECU", "ECR", "PT")    # commanded off throughout

LOW_MV = 1900
HIGH_MV = 2200
FREQ_HZ = 5                # full cycles per second
DURATION_S = 4.0                # length of the square-wave burst
SETTLE_S = 2.0                  # dwell at low before zeroing
TAIL_S = 0.5                    # extra recording at low after the burst
SAMPLE_HZ = 200.0               # sampler clock; the bus is the real limit

CSV_COLUMNS = [
    "wall_time", "elapsed_s", "cycle", "segment", "target_mV",
    "cmd_FCU_mV", "cmd_FCR_mV", "len_FCU_mm", "len_FCR_mm",
    "abs_FCU_counts", "abs_FCR_counts", "fresh_FCU", "fresh_FCR",
    "online_FCU", "online_FCR",
]

I_T = CSV_COLUMNS.index("elapsed_s")
I_CYCLE = CSV_COLUMNS.index("cycle")
I_SEGMENT = CSV_COLUMNS.index("segment")
I_TARGET = CSV_COLUMNS.index("target_mV")
I_LEN_FCU = CSV_COLUMNS.index("len_FCU_mm")
I_LEN_FCR = CSV_COLUMNS.index("len_FCR_mm")


# ---- Recorder -------------------------------------------------------------

class Recorder:
    """Two-sensor sampler. Length is derived from raw counts, not from
    EncoderController.get_state()["position_mm"], which is rounded to 2 dp."""

    def __init__(self, valve, encoder, regs, sensors, zero_counts,
                 counts_per_turn, drum_diameter_mm, sample_hz):
        self.valve = valve
        self.encoder = encoder
        self.reg_fcu, self.reg_fcr = regs
        self.sen_fcu, self.sen_fcr = sensors
        self.zero = zero_counts                 # {slave: counts}
        self.mm_per_count = math.pi * drum_diameter_mm / counts_per_turn
        self.period = 1.0 / sample_hz
        self.rows = []
        self._stop = threading.Event()
        self._thread = None
        self._ctx_lock = threading.Lock()
        self._ctx = {"cycle": 0, "segment": "settle", "target": LOW_MV}
        self._last = {}
        self.t0 = None

    def set_context(self, cycle, segment, target):
        with self._ctx_lock:
            self._ctx = {"cycle": cycle, "segment": segment, "target": target}

    def _read(self, by, slave):
        """(length_mm, counts, fresh, online) for one sensor, full precision."""
        e = by.get(slave, {})
        counts = e.get("absolute_position")
        online = bool(e.get("online"))
        fresh = counts is not None and counts != self._last.get(slave)
        if counts is not None:
            self._last[slave] = counts
        zero = self.zero.get(slave)
        length = (None if counts is None or zero is None
                  else round((counts - zero) * self.mm_per_count, LEN_DECIMALS))
        return length, counts, fresh, online

    def _loop(self):
        while not self._stop.is_set():
            start = time.perf_counter()
            with self._ctx_lock:
                c = dict(self._ctx)
            with self.valve.display_lock:
                data = dict(self.valve.valve_data)
            by = {e["slave"]: e for e in
                  self.encoder.get_state().get("encoders", [])}
            l_u, a_u, f_u, on_u = self._read(by, self.sen_fcu)
            l_r, a_r, f_r, on_r = self._read(by, self.sen_fcr)
            self.rows.append([
                round(time.time(), 3), round(start - self.t0, 4),
                c["cycle"], c["segment"], c["target"],
                data.get(self.reg_fcu), data.get(self.reg_fcr),
                l_u, l_r, a_u, a_r, f_u, f_r, on_u, on_r,
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
    """FCU/FCR length + commanded setpoint, drawn on the main thread from the
    sequence's waits (see fcu_fcr_cycle_test.LivePlot for the rationale)."""

    C_FCU, C_FCR = "#1f77b4", "#d62728"

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
            self.l_fcu, = self.ax_len.plot([], [], color=self.C_FCU, lw=1.4,
                                           label="FCU (sensor 58)")
            self.l_fcr, = self.ax_len.plot([], [], color=self.C_FCR, lw=1.4,
                                           label="FCR (sensor 59)")
            self.ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
            self.ax_len.set_ylabel("length (mm, rel. low)")
            self.ax_len.legend(loc="upper right", fontsize=9)
            self.ax_len.grid(alpha=0.3)

            self.cmd, = self.ax_p.step([], [], where="post", color="#333",
                                       lw=1.2, label="target (both)")
            for mv in (self.low, self.high):
                self.ax_p.axhline(mv, color="0.75", lw=0.7)
            self.ax_p.set_ylim(self.low - 60, self.high + 60)
            self.ax_p.set_ylabel("setpoint (mV)")
            self.ax_p.set_xlabel("time since zero (s)")
            self.ax_p.legend(loc="upper right", fontsize=8)
            self.ax_p.grid(alpha=0.3)
            for ax in (self.ax_len, self.ax_p):
                ax.set_xlim(0, self.span)
            self.fig.canvas.manager.set_window_title("FCU/FCR square-wave live")
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
        self.l_fcu.set_data(t, [num(r, I_LEN_FCU) for r in view])
        self.l_fcr.set_data(t, [num(r, I_LEN_FCR) for r in view])
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

def capture_zero(encoder, sensors):
    """Snapshot each sensor's absolute_position as its zero, in raw counts."""
    by = {e["slave"]: e for e in encoder.get_state().get("encoders", [])}
    zero = {}
    for s in sensors:
        counts = by.get(s, {}).get("absolute_position")
        if counts is None:
            return None, f"sensor {s} reported no absolute position"
        zero[s] = counts
    return zero, None


def save_csv(rows, out_dir, freq):
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(out_dir, f"fcu_fcr_square_{freq:g}hz_{stamp}.csv")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_COLUMNS)
        w.writerows(rows)
    return path


def describe_plan(args):
    half = 0.5 / args.freq
    return "\n".join([
        f"Driven muscles   : {' + '.join(MOVING)} — stepped together, in phase",
        f"Setpoints        : low {args.low} mV  <->  high {args.high} mV",
        f"Square wave      : {args.freq:g} Hz  ({half * 1000:.0f} ms at each "
        f"setpoint), {args.duration:g}s = {args.freq * args.duration:g} cycles",
        f"Vented throughout: {', '.join(VENTED)}",
        f"Before recording : hold low {args.settle:g}s, then ZERO both sensors",
        f"After the burst  : hold low a further {args.tail:g}s (recorded)",
        f"Sampler          : {args.sample_hz:g} Hz, length to {LEN_DECIMALS} dp",
    ])


# ---- Run ------------------------------------------------------------------

def run(args):
    from valve_controller import ValveController
    from encoder_controller import EncoderController

    muscles = resolve_muscles(args.mapping)
    missing = [n for n in (*MOVING, *VENTED) if n not in muscles]
    if missing:
        print(f"[ERR] sensor_mapping.json has no entry for: {', '.join(missing)}")
        return 2
    regs = tuple(muscles[n]["regulator"] for n in MOVING)
    sensors = tuple(muscles[n]["sensor"] for n in MOVING)
    if any(s is None for s in sensors):
        print(f"[ERR] {MOVING} must both have a sensor mapped; got {sensors}")
        return 2

    print(describe_plan(args))
    print()
    for name, r, s in zip(MOVING, regs, sensors):
        print(f"{name} = regulator {r}, sensor {s}")
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
    print(f"\nValve regulator : {valve_port}\nRS-485 sensors  : {rs485_port}")

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
        slave_ids=list(sensors),
        counts_per_turn=args.counts_per_turn,
        drum_diameter_mm=args.drum_diameter,
        interval=0.0,           # spin as fast as the two encoders answer
    )
    if not encoder.connected:
        print(f"[ERR] could not open RS-485 adapter on {rs485_port}")
        valve.close()
        return 1

    all_regs = list(regs) + [muscles[n]["regulator"] for n in VENTED]
    rec = None
    interrupted = False
    csv_path = None
    try:
        print(f"Waiting for sensors {sensors[0]} and {sensors[1]} to come online...")
        if not wait_online(encoder, list(sensors)):
            print(f"[ERR] sensors {sensors} did not come online. "
                  "Check wiring / slave ids.")
            return 1

        # Vent everything first, so the run starts from a known state.
        valve.set_multiple_valves([(r, "off") for r in all_regs], ramp=0.0)
        sleep_or_abort(0.3)

        print(f"Holding {' + '.join(MOVING)} at {args.low} for {args.settle:g}s...")
        valve.set_multiple_valves([(r, args.low) for r in regs], ramp=0.0)
        sleep_or_abort(args.settle)

        zero_counts, zerr = capture_zero(encoder, sensors)
        if zero_counts is None:
            print(f"[ERR] could not zero: {zerr}")
            return 1
        print(f"Zeroed at {args.low}: "
              + ", ".join(f"{n}={zero_counts[s]}" for n, s in zip(MOVING, sensors))
              + " counts. Recording started.")

        rec = Recorder(valve, encoder, regs, sensors, zero_counts,
                       args.counts_per_turn, args.drum_diameter, args.sample_hz)
        rec.start()

        if not args.no_live:
            live = LivePlot(rec, args.duration + args.tail, args.live_fps,
                            args.low, args.high)
            if live.open():
                rig._live = live      # sleep_or_abort pumps it
                print(f"Live view open ({args.live_fps:g} fps). "
                      "Closing the window does not stop the run — use Ctrl-C.")

        # --- the square wave: rewrite both setpoints at each half-period. --
        half = 0.5 / args.freq
        n_steps = int(round(args.duration / half))
        print(f"Driving {args.freq:g} Hz for {args.duration:g}s "
              f"({n_steps} steps of {half * 1000:.0f} ms)...")
        t0 = time.perf_counter()
        for k in range(n_steps):
            mv = args.high if k % 2 == 0 else args.low
            rec.set_context(k // 2 + 1, "high" if k % 2 == 0 else "low", mv)
            # One batched command, so both muscles step on the same write.
            valve.set_multiple_valves([(r, mv) for r in regs], ramp=0.0)
            # Absolute deadline, so per-step overhead cannot accumulate drift.
            sleep_or_abort(max(0.0, t0 + (k + 1) * half - time.perf_counter()))

        if args.tail > 0:
            rec.set_context(n_steps // 2, "tail", args.low)
            valve.set_multiple_valves([(r, args.low) for r in regs], ramp=0.0)
            sleep_or_abort(args.tail)

        print(f"Burst done in {time.perf_counter() - t0:.3f}s "
              f"(target {args.duration:g}s).")

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
            if confirm_save(len(rec.rows), os.path.abspath(args.out), args.yes):
                csv_path = save_csv(rec.rows, args.out, args.freq)
                print(f"Saved {len(rec.rows)} samples to {os.path.abspath(csv_path)}")
            else:
                print(f"[INFO] not saved — discarded {len(rec.rows)} sample(s).")

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
        "fcu": col("len_FCU_mm"),
        "fcr": col("len_FCR_mm"),
        "target": col("target_mV"),
        "fresh_fcu": [r.get("fresh_FCU", "") == "True" for r in rows],
        "fresh_fcr": [r.get("fresh_FCR", "") == "True" for r in rows],
    }


def plot_csv(path, show=False):
    import matplotlib
    if not show and "matplotlib.pyplot" not in sys.modules:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    d = _read_csv(path)
    t, cyc = d["t"], d["cycle"]
    C_FCU, C_FCR = "#1f77b4", "#d62728"

    fig = plt.figure(figsize=(13, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.5, 0.8, 1.1], hspace=0.35,
                          wspace=0.22)
    ax_len = fig.add_subplot(gs[0, :])
    ax_p = fig.add_subplot(gs[1, :], sharex=ax_len)
    ax_u = fig.add_subplot(gs[2, 0])
    ax_r = fig.add_subplot(gs[2, 1])

    # -- length vs time, high half-periods shaded ---------------------------
    for i in range(len(t) - 1):
        if d["segment"][i] == "high":
            ax_len.axvspan(t[i], t[i + 1], color="#ffe9c7", lw=0, zorder=0)
    ax_len.plot(t, d["fcu"], color=C_FCU, lw=1.3, label="FCU (sensor 58)")
    ax_len.plot(t, d["fcr"], color=C_FCR, lw=1.3, label="FCR (sensor 59)")
    ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
    ax_len.set_ylabel("length (mm, rel. low)")
    ax_len.set_title(f"FCU/FCR square-wave step response — {os.path.basename(path)}\n"
                     "shaded = commanded high, other four vented")
    ax_len.legend(loc="best", fontsize=9)
    ax_len.grid(alpha=0.3)

    # -- commanded setpoint (shared by both muscles) ------------------------
    ax_p.step(t, d["target"], where="post", color="0.25", lw=1.2, label="target")
    ax_p.set_ylabel("setpoint (mV)")
    ax_p.set_xlabel("time since zero (s)")
    ax_p.legend(loc="best", fontsize=8)
    ax_p.grid(alpha=0.3)

    # -- cycles overlaid, one axes per sensor -------------------------------
    driven = np.array([s in ("high", "low") for s in d["segment"]])
    cycles = [c for c in sorted(set(cyc.tolist()))
              if c > 0 and np.any((cyc == c) & driven)]
    cmap = plt.get_cmap("viridis")
    for ax, key, name, colr in ((ax_u, "fcu", "FCU (58)", C_FCU),
                                (ax_r, "fcr", "FCR (59)", C_FCR)):
        for n, c in enumerate(cycles):
            idx = np.where(cyc == c)[0]
            if not len(idx):
                continue
            ax.plot(t[idx] - t[idx[0]], d[key][idx], lw=1.2,
                    color=cmap(n / max(1, len(cycles) - 1)), label=f"c{c}")
        ax.set_title(f"{name} — cycles overlaid", fontsize=10, color=colr)
        ax.set_xlabel("time within cycle (s)")
        ax.set_ylabel("length (mm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=2)

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
    if span > 0:
        print(f"\nSampler {len(d['t']) / span:.0f} Hz; effective sensor rate "
              f"FCU {sum(d['fresh_fcu']) / span:.0f} Hz, "
              f"FCR {sum(d['fresh_fcr']) / span:.0f} Hz")

    print(f"\nPer-cycle length extremes (mm, {LEN_DECIMALS} dp)")
    print(f"{'cycle':>5}  {'FCU min':>10} {'FCU max':>10} {'FCU p-p':>10}"
          f"  {'FCR min':>10} {'FCR max':>10} {'FCR p-p':>10}")
    pp = {"fcu": [], "fcr": []}
    for c in cycles:
        idx = d["cycle"] == c
        cells = []
        for key in ("fcu", "fcr"):
            v = d[key][idx]
            v = v[~np.isnan(v)]
            if not len(v):
                cells += ["–"] * 3
                continue
            pp[key].append(v.max() - v.min())
            cells += [f"{v.min():.4f}", f"{v.max():.4f}", f"{v.max() - v.min():.4f}"]
        print(f"{c:>5}  " + " ".join(f"{x:>10}" for x in cells))

    print()
    for key, name in (("fcu", "FCU"), ("fcr", "FCR")):
        if len(pp[key]) > 1:
            a = np.array(pp[key])
            print(f"  {name}: stroke {a.mean():.4f} mm mean, "
                  f"spread {a.max() - a.min():.4f} mm across {len(a)} cycles")
    print("If the stroke is far below the quasi-static travel between these two "
          "setpoints,\nthe regulator is not keeping up at this frequency.")


# ---- CLI ------------------------------------------------------------------

def build_parser():
    from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM

    p = argparse.ArgumentParser(
        description="FCU+FCR square-wave step response (other four vented)")
    p.add_argument("--valve-port", default=None, help="Giga R1 regulator port")
    p.add_argument("--rs485-port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--low", type=int, default=LOW_MV, help="low setpoint (mV)")
    p.add_argument("--high", type=int, default=HIGH_MV, help="high setpoint (mV)")
    p.add_argument("--freq", type=float, default=FREQ_HZ,
                   help="square-wave frequency in Hz (default 2)")
    p.add_argument("--duration", type=float, default=DURATION_S,
                   help="length of the burst in seconds (default 4)")
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
    p.add_argument("-y", "--yes", action="store_true",
                   help="save without asking (default is to prompt after the run)")
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
