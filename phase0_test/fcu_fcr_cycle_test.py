#!/usr/bin/env python3
"""
fcu_fcr_cycle_test.py

Three-phase open-loop cycling test of the two antagonist wrist flexors,
FCU and FCR, with the other four muscles held at fixed pressure throughout.

Only FCU (sensor 58) and FCR (sensor 59) carry a physically attached draw-wire
sensor; the sensors listed for PT/ECU/ECR/FDS in sensor_mapping.json are dummy
entries and are neither polled nor plotted.

Pressure schedule (setpoints in kPa; ValveController converts to the mV the
Giga R1 takes, 1 kPa = 16.67 mV)
----------------------------------------------------------------------------
Held constant through every phase:
    FDS = 17    ECU = 14    ECR = 14    PT = 20

FCU and FCR move together:
    phase 1  "init"   26 / 26     ramp in over 1 s, then hold 2 s
    phase 2  "up"     38 / 38
    phase 3  "down"   14 / 14

Timeline
--------
    ramp everything in (1 s)  ->  phase 1 hold (2 s)  ->  ZERO sensors 58/59
    [recording starts]
    phase 1 -> phase 2   1 s ramp, 0.5 s dwell
    then 4 x cycle:
        phase 2 -> phase 3   1 s ramp, 0.5 s dwell
        phase 3 -> phase 2   1 s ramp, 0.5 s dwell
    ~13.5 s recorded in total.

Output
------
Both artefacts land in phase0_test/phase0_data/wrist_flex/ (created on first
run, git-ignored):

    fcu_fcr_cycle_<stamp>.csv          raw samples, length to 4 decimals
    fcu_fcr_cycle_<stamp>_cycles.png   the 3-panel summary figure

Nothing is written unless the run completes: Ctrl-C, SIGTERM, the stop-file and
the runtime watchdog all discard the buffer, so the folder only ever holds full
runs. A completed run then asks before writing anything — answer y to keep it,
anything else (including a bare Enter) throws it away. Pass -y to skip the
question, and note that a non-interactive run keeps its data rather than
prompting.

A live window (length + commanded pressure) is shown while the sequence runs,
updating from the recorder's buffer at --live-fps. It is drawn on the main
thread from the sequence's own waits, so it never blocks the 50 Hz sampler.
Closing that window does not stop the run — use one of the e-stops. Pass
--no-live to skip it.

Length precision
----------------
EncoderController.get_state() rounds position_mm to 2 decimals, which is
coarser than the 4 decimals wanted here, so this script does not use that
field. It captures its own zero from the raw `absolute_position` counts at the
end of the phase 1 hold and converts counts to mm itself:

    length_mm = (counts - zero_counts) / counts_per_turn * pi * drum_diameter

giving the full resolution of the encoder (~2.1e-5 mm/count at 21 bit on a
14 mm drum) before rounding to 4 decimals.

Emergency stop
--------------
Four ways to abort a run in progress; all of them cancel every in-flight ramp
and send the Arduino's "s" (all valves off) before unwinding:

  Ctrl-C                  in the terminal running the test
  kill <pid>              SIGTERM, from anywhere (the pid is printed at start)
  touch phase0_test/STOP  from another terminal (polled every 50 ms)
  the runtime watchdog    self-aborts past the planned duration + 30 s

Any of them discards the recording — an aborted run writes no CSV and no PNG.

Press Ctrl-C a second time to force-quit if the graceful path ever wedges —
note that this skips venting and can leave the muscles pressurised.

This talks to the hardware directly, so the web app (app.py) must NOT be
running at the same time — it would already hold both serial ports.

Run it from anywhere (paths are resolved relative to this file):
    python phase0_test/fcu_fcr_cycle_test.py
    python phase0_test/fcu_fcr_cycle_test.py --cycles 4 --sample-hz 50
    python phase0_test/fcu_fcr_cycle_test.py --no-live      # no live window
    python phase0_test/fcu_fcr_cycle_test.py --dry-run      # no hardware, print plan
    python phase0_test/fcu_fcr_cycle_test.py --plot phase0_test/phase0_data/<file>.csv
    touch phase0_test/STOP                                  # e-stop a running test
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import threading
import time

# This script lives in phase0_test/ but drives the app's serial layers, which
# live one level up — put the project root on sys.path before importing them so
# it runs from any working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ---- Test definition ------------------------------------------------------

def mv_to_kpa(mV):
    """Giga echo (mV) -> commanded kPa. Mirrors valve_controller.mv_to_kpa, kept
    local so --plot works on a machine without pyserial."""
    return mV * 0.06 - 100.0


# Muscles held at a fixed setpoint (kPa) for the whole run.
FIXED = {"FDS": 17, "ECU": 14, "ECR": 14, "PT": 20}

# The two muscles under test; both are commanded to the same value each phase.
MOVING = ("FCU", "FCR")

PHASE1_KPA = 26         # init
PHASE2_KPA = 38         # up
PHASE3_KPA = 14         # down

INIT_RAMP_S = 1.0       # gentle ramp from vented up to the phase 1 setpoints
PHASE1_HOLD_S = 2.0     # dwell at phase 1 before zeroing
RAMP_S = 1.0            # every phase-to-phase transition
DWELL_S = 0.5           # dwell at each of phase 2 / phase 3
CYCLES = 4              # one cycle = p2 -> p3 -> p2
SAMPLE_HZ = 50.0

LEN_DECIMALS = 4        # recorded length precision

# Raw CSV + the finished PNG both land here; created on first run.
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "phase0_data", "wrist_flex")

# Touch this file from another terminal to abort a run in progress.
DEFAULT_STOP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "STOP")
WATCHDOG_POLL_S = 0.05  # stop-file / runtime check period
RUNTIME_MARGIN_S = 30.0 # slack over the planned duration before the watchdog fires

CSV_COLUMNS = [
    "wall_time", "elapsed_s", "cycle", "segment",
    "target_FCU", "target_FCR", "cmd_FCU_kPa", "cmd_FCR_kPa",
    "len_FCU_mm", "len_FCR_mm", "abs_FCU_counts", "abs_FCR_counts",
    "online_FCU", "online_FCR",
    "fix_FDS", "fix_ECU", "fix_ECR", "fix_PT",
]


# Column positions in a recorder row, so the live view can read the buffer
# without re-deriving the layout.
I_T = CSV_COLUMNS.index("elapsed_s")
I_CYCLE = CSV_COLUMNS.index("cycle")
I_SEGMENT = CSV_COLUMNS.index("segment")
I_LEN_FCU = CSV_COLUMNS.index("len_FCU_mm")
I_LEN_FCR = CSV_COLUMNS.index("len_FCR_mm")
I_CMD_FCU = CSV_COLUMNS.index("cmd_FCU_kPa")
I_CMD_FCR = CSV_COLUMNS.index("cmd_FCR_kPa")


# ---- Serial port detection (mirrors app.py, kept standalone) --------------

def _blob(p):
    return " ".join(str(x) for x in (
        p.description, getattr(p, "product", None),
        getattr(p, "manufacturer", None), p.hwid)).lower()


def find_ports(valve_port, rs485_port):
    import serial.tools.list_ports
    from encoder_controller import is_ch9344_port, list_ch9344_ports

    ports = [p for p in serial.tools.list_ports.comports()
             if ("ttyACM" in p.device or "ttyUSB" in p.device
                 or p.device.upper().startswith("COM"))
             and not is_ch9344_port(p.device)]
    if not valve_port:
        valve_port = next(
            (p.device for p in ports
             if any(k in _blob(p) for k in ("giga", "arduino", "2341:"))), None)
    if not rs485_port:
        # EKU081 8-port adapter (CH9344): globbed from /dev, since its
        # out-of-tree driver may not show up in comports(). Lowest port first.
        ch9344 = [d for d in list_ch9344_ports() if d != valve_port]
        rs485_port = ch9344[0] if ch9344 else None
        if not rs485_port:
            rs485_port = next(
                (p.device for p in ports
                 if p.device != valve_port and any(k in _blob(p) for k in (
                     "1a86", "ch340", "ch343", "ch9344", "single serial",
                     "0403", "ftdi", "10c4", "cp210"))), None)
        if not rs485_port:
            rs485_port = next(
                (p.device for p in ports if p.device != valve_port), None)
    return valve_port, rs485_port


# ---- Mapping --------------------------------------------------------------

def resolve_muscles(mapping_path):
    """Return {muscle_name: {"regulator": int, "sensor": int|None}} by name.

    sensor_mapping.json is keyed by regulator, so invert it onto the muscle
    names this test speaks in (FCU, FCR, FDS, ECU, ECR, PT).
    """
    from sensor_mapping import load_mapping

    mapping = load_mapping(mapping_path) if mapping_path else load_mapping()
    by_name = {}
    for regulator, entry in mapping.items():
        by_name[str(entry.get("muscle"))] = {
            "regulator": regulator,
            "sensor": entry.get("sensor"),
        }
    return by_name


def check_muscles(by_name):
    """Verify every muscle this test drives is present in the mapping."""
    missing = [n for n in (*MOVING, *FIXED) if n not in by_name]
    if missing:
        return (f"sensor_mapping.json has no entry for: {', '.join(missing)}. "
                f"Found: {', '.join(sorted(by_name)) or '(none)'}")
    for name in MOVING:
        if by_name[name]["sensor"] is None:
            return f"{name} has no sensor mapped; it is the muscle being measured"
    return None


# ---- Emergency stop -------------------------------------------------------

# Set by any abort path: Ctrl-C, SIGTERM, the stop-file, or the runtime
# watchdog. Every wait in the sequence blocks on this instead of time.sleep(),
# so an abort unwinds at once rather than driving the next setpoint first.
ABORT = threading.Event()
_abort_reason = ""
_estop_lock = threading.Lock()


class Aborted(Exception):
    """Raised inside the sequence when an e-stop path fires."""


def trigger_abort(valve, reason):
    """Vent immediately, then tell the main sequence to unwind.

    Safe to call from any thread and more than once. The stop is sent under a
    lock and retried, because a single dropped serial write would otherwise
    leave all six muscles pressurised.
    """
    global _abort_reason
    with _estop_lock:
        if not _abort_reason:
            _abort_reason = reason
        if valve is not None:
            for attempt in range(3):
                try:
                    valve.emergency_stop()   # cancels every ramp, then sends "s"
                    break
                except Exception as exc:
                    print(f"[WARN] e-stop send failed ({exc}), retrying...")
                    time.sleep(0.05)
            else:
                print("[ERR] e-stop could not be sent! Cut the air supply.")
    ABORT.set()


# Set by run() while a live view is open; consulted by sleep_or_abort.
_live = None


def sleep_or_abort(seconds):
    """Interruptible sleep; raises Aborted if an e-stop fired during it.

    When a live view is open it is redrawn here, in 1/fps slices. Drawing has
    to happen on the main thread (matplotlib is not thread-safe) and must stay
    off the recorder's thread, so pumping it from the sequence's own waits is
    the one place that satisfies both.
    """
    if _live is None:
        if ABORT.wait(seconds):
            raise Aborted(_abort_reason)
        return
    deadline = time.monotonic() + seconds
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            break
        if ABORT.wait(min(_live.period, left)):
            raise Aborted(_abort_reason)
        _live.update()
    _live.update()


def install_signal_handlers(valve):
    """Ctrl-C (SIGINT) and `kill <pid>` (SIGTERM) both vent the rig.

    A second interrupt restores the default handler, so the user can always
    force-quit even if the graceful path is wedged.
    """
    def handler(signum, _frame):
        name = signal.Signals(signum).name
        if ABORT.is_set():
            signal.signal(signum, signal.SIG_DFL)
            print("\n[E-STOP] already stopping — press Ctrl-C again to "
                  "force-quit (valves may stay pressurised).")
            return
        print(f"\n[E-STOP] {name} received — venting.")
        trigger_abort(valve, name)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass        # not the main thread, or unsupported platform


def restore_signal_handlers():
    """Hand SIGINT/SIGTERM back to Python once the rig is vented.

    Called before prompting: with the e-stop handler still installed, Ctrl-C at
    the prompt would print an e-stop banner and leave input() waiting, instead
    of simply answering "no".
    """
    for sig, handler in ((signal.SIGINT, signal.default_int_handler),
                         (signal.SIGTERM, signal.SIG_DFL)):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


def confirm_save(n_samples, out_dir, assume_yes=False):
    """Ask whether this completed run is worth keeping. True to save.

    Defaults to NOT saving, because most runs are throwaway and the folder is
    the thing that gets cluttered. A run costs seconds to repeat; a directory
    of near-identical files costs more to sort out later.

    Runs with no terminal attached keep the data instead of prompting, so a
    scripted or piped invocation cannot hang or silently discard a result.
    """
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print("[INFO] not a terminal — keeping the run automatically")
        return True
    restore_signal_handlers()
    try:
        answer = input(f"\nRun complete: {n_samples} sample(s). "
                       f"Save to {out_dir}? [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer in ("y", "yes")


def start_watchdog(valve, stop_file, max_runtime):
    """Background abort paths: the stop-file, and a hard runtime ceiling."""
    def watch():
        started = time.monotonic()
        while not ABORT.is_set():
            if stop_file and os.path.exists(stop_file):
                print(f"\n[E-STOP] stop-file {stop_file} appeared — venting.")
                try:
                    os.remove(stop_file)
                except OSError:
                    pass
                trigger_abort(valve, "stop-file")
                return
            if max_runtime and time.monotonic() - started > max_runtime:
                print(f"\n[E-STOP] exceeded max runtime {max_runtime:g}s — venting.")
                trigger_abort(valve, "max-runtime")
                return
            ABORT.wait(WATCHDOG_POLL_S)

    t = threading.Thread(target=watch, daemon=True)
    t.start()
    return t


# ---- Recorder -------------------------------------------------------------

class Recorder:
    """Background sampler: one CSV row per tick, from the moment it starts.

    Length is computed here from raw encoder counts rather than read from
    EncoderController.get_state()["position_mm"], which is rounded to 2 dp.
    """

    def __init__(self, valve, encoder, regs, sensors, zero_counts,
                 counts_per_turn, drum_diameter_mm, sample_hz):
        self.valve = valve
        self.encoder = encoder
        self.reg_fcu, self.reg_fcr = regs
        self.sen_fcu, self.sen_fcr = sensors
        self.zero_counts = zero_counts          # {slave: counts}
        self.mm_per_count = math.pi * drum_diameter_mm / counts_per_turn
        self.period = 1.0 / sample_hz
        self.rows = []
        self._stop = threading.Event()
        self._thread = None
        self._ctx_lock = threading.Lock()
        self._ctx = {"cycle": 0, "segment": "start",
                     "t_fcu": PHASE1_KPA, "t_fcr": PHASE1_KPA}
        self.t0 = None

    def set_context(self, cycle, segment, t_fcu, t_fcr):
        with self._ctx_lock:
            self._ctx = {"cycle": cycle, "segment": segment,
                         "t_fcu": t_fcu, "t_fcr": t_fcr}

    def _cmd(self):
        """Last commanded pressure (kPa) of FCU and FCR; None while off."""
        with self.valve.display_lock:
            data = dict(self.valve.valve_data)      # echoed by the Giga, in mV

        def kpa(reg):
            mV = data.get(reg)
            return None if mV is None else round(mv_to_kpa(mV), 2)
        return kpa(self.reg_fcu), kpa(self.reg_fcr)

    def _length(self, entry, slave):
        """(length_mm, abs_counts, online) for one sensor, full precision."""
        counts = entry.get("absolute_position")
        online = bool(entry.get("online"))
        zero = self.zero_counts.get(slave)
        if counts is None or zero is None:
            return None, counts, online
        return round((counts - zero) * self.mm_per_count, LEN_DECIMALS), counts, online

    def _loop(self):
        while not self._stop.is_set():
            start = time.perf_counter()
            with self._ctx_lock:
                c = dict(self._ctx)
            cmd_fcu, cmd_fcr = self._cmd()
            by = {e["slave"]: e for e in
                  self.encoder.get_state().get("encoders", [])}
            l_fcu, a_fcu, on_fcu = self._length(by.get(self.sen_fcu, {}), self.sen_fcu)
            l_fcr, a_fcr, on_fcr = self._length(by.get(self.sen_fcr, {}), self.sen_fcr)
            self.rows.append([
                round(time.time(), 3), round(start - self.t0, 4),
                c["cycle"], c["segment"], c["t_fcu"], c["t_fcr"],
                cmd_fcu, cmd_fcr, l_fcu, l_fcr, a_fcu, a_fcr, on_fcu, on_fcr,
                FIXED["FDS"], FIXED["ECU"], FIXED["ECR"], FIXED["PT"],
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
    """Length + commanded-pressure trace, updated while the sequence runs.

    It reads the recorder's row buffer instead of sampling the hardware itself,
    so the 50 Hz recording thread is never blocked by drawing. All matplotlib
    calls happen on the main thread, driven from sleep_or_abort().
    """

    def __init__(self, rec, recorded_s, fps):
        self.rec = rec
        self.recorded_s = recorded_s
        self.period = 1.0 / max(1.0, fps)
        self.fig = None
        self.dead = False
        self._next = 0.0

    def open(self):
        """Create the window. Returns False if no interactive backend works."""
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            if matplotlib.get_backend().lower() == "agg":
                print("[WARN] no interactive matplotlib backend; live view off")
                return False
            plt.ion()
            self.fig, (self.ax_len, self.ax_p) = plt.subplots(
                2, 1, figsize=(10, 7), sharex=True,
                gridspec_kw={"height_ratios": [1.5, 1.0]})
            self.l_fcu, = self.ax_len.plot([], [], color="#1f77b4", lw=1.5,
                                           label="FCU (sensor 58)")
            self.l_fcr, = self.ax_len.plot([], [], color="#d62728", lw=1.5,
                                           label="FCR (sensor 59)")
            self.ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
            self.ax_len.set_ylabel("length (mm, rel. phase 1)")
            self.ax_len.legend(loc="upper right", fontsize=9)
            self.ax_len.grid(alpha=0.3)

            self.p_fcu, = self.ax_p.plot([], [], color="#1f77b4", lw=1.2,
                                         label="FCU cmd")
            self.p_fcr, = self.ax_p.plot([], [], color="#d62728", lw=1.2,
                                         ls="--", label="FCR cmd")
            for kpa, lbl in ((PHASE2_KPA, "p2 up"), (PHASE3_KPA, "p3 down")):
                self.ax_p.axhline(kpa, color="0.75", lw=0.7)
                self.ax_p.text(self.recorded_s, kpa, f" {lbl}", va="center",
                               fontsize=8, color="0.45")
            self.ax_p.set_ylim(min(PHASE3_KPA, PHASE1_KPA) - 4, PHASE2_KPA + 4)
            self.ax_p.set_ylabel("setpoint (kPa)")
            self.ax_p.set_xlabel("time since zero (s)")
            self.ax_p.legend(loc="upper right", fontsize=8)
            self.ax_p.grid(alpha=0.3)

            for ax in (self.ax_len, self.ax_p):
                ax.set_xlim(0, self.recorded_s)
            self.fig.canvas.manager.set_window_title("FCU/FCR live")
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
        """Redraw from the recorder buffer, at most once per 1/fps."""
        if self.fig is None or self.dead:
            return
        now = time.monotonic()
        if not force and now < self._next:
            return
        self._next = now + self.period

        rows = self.rec.rows
        n = len(rows)                 # snapshot the length: the sampler appends
        if n == 0:                    # concurrently, and list append is atomic
            return
        view = rows[:n]
        nan = float("nan")
        t = [r[I_T] for r in view]
        num = lambda r, i: r[i] if isinstance(r[i], (int, float)) else nan

        self.l_fcu.set_data(t, [num(r, I_LEN_FCU) for r in view])
        self.l_fcr.set_data(t, [num(r, I_LEN_FCR) for r in view])
        self.p_fcu.set_data(t, [num(r, I_CMD_FCU) for r in view])
        self.p_fcr.set_data(t, [num(r, I_CMD_FCR) for r in view])

        self.ax_len.relim()
        self.ax_len.autoscale_view(scalex=False, scaley=True)
        last = view[-1]
        self.ax_len.set_title(
            f"cycle {last[I_CYCLE]}  —  {last[I_SEGMENT]}  —  t={t[-1]:.1f}s",
            fontsize=10)
        try:
            self._draw()
        except Exception as exc:
            # Window closed mid-run, or the backend died: keep the test going,
            # the CSV and the final PNG are what actually matter.
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

def wait_online(encoder, sensors, timeout=10.0):
    """Block until every sensor id reports online, or timeout.

    Aborts early (raising Aborted) if an e-stop fires while waiting, so a rig
    with a dead sensor can still be stopped with Ctrl-C.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        by = {e["slave"]: e for e in encoder.get_state().get("encoders", [])}
        if all(by.get(s, {}).get("online") for s in sensors):
            return True
        sleep_or_abort(0.2)
    return False


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


def save_csv(rows, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    fname = f"fcu_fcr_cycle_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    path = os.path.join(out_dir, fname)
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)
    return path


def describe_plan(cycles):
    """Human-readable schedule, printed before every run."""
    fixed = "  ".join(f"{n}={v}" for n, v in FIXED.items())
    recorded = RAMP_S + DWELL_S + cycles * 2 * (RAMP_S + DWELL_S)
    return "\n".join([
        f"Fixed throughout : {fixed}  (kPa)",
        f"FCU/FCR phases   : p1 init {PHASE1_KPA}  ->  p2 up {PHASE2_KPA}  "
        f"->  p3 down {PHASE3_KPA}  (kPa)",
        f"Ramp in          : {INIT_RAMP_S:g}s, then phase 1 hold "
        f"{PHASE1_HOLD_S:g}s, then ZERO sensors",
        f"Transitions      : {RAMP_S:g}s ramp, {DWELL_S:g}s dwell at each phase",
        f"Cycles           : {cycles} x (p2 -> p3 -> p2)",
        f"Recorded         : ~{recorded:g}s at {SAMPLE_HZ:g} Hz, "
        f"length to {LEN_DECIMALS} dp",
    ])


# ---- Run ------------------------------------------------------------------

def run(args):
    global _live
    from valve_controller import ValveController
    from encoder_controller import EncoderController

    muscles = resolve_muscles(args.mapping)
    err = check_muscles(muscles)
    if err:
        print(f"[ERR] {err}")
        return 2

    reg_fcu = muscles["FCU"]["regulator"]
    reg_fcr = muscles["FCR"]["regulator"]
    sen_fcu = muscles["FCU"]["sensor"]
    sen_fcr = muscles["FCR"]["sensor"]
    sensors = (sen_fcu, sen_fcr)

    print(describe_plan(args.cycles))
    print()
    print(f"FCU = regulator {reg_fcu}, sensor {sen_fcu}")
    print(f"FCR = regulator {reg_fcr}, sensor {sen_fcr}")
    for name, kpa in FIXED.items():
        print(f"{name:<4}= regulator {muscles[name]['regulator']}, held at {kpa} kPa")

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

    # Arm every e-stop path before anything is ever pressurised.
    stop_file = None if args.no_stop_file else args.stop_file
    if stop_file and os.path.exists(stop_file):
        print(f"[WARN] removing stale stop-file {stop_file}")
        try:
            os.remove(stop_file)
        except OSError as exc:
            print(f"[ERR] could not remove it ({exc}); the run would abort at once")
            valve.close()
            return 2
    planned = (INIT_RAMP_S + PHASE1_HOLD_S + RAMP_S + DWELL_S
               + args.cycles * 2 * (RAMP_S + DWELL_S))
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
        interval=0.01,      # poll the two live sensors as fast as they answer
    )
    if not encoder.connected:
        print(f"[ERR] could not open RS-485 adapter on {rs485_port}")
        valve.close()
        return 1

    all_regs = [reg_fcu, reg_fcr] + [muscles[n]["regulator"] for n in FIXED]
    rec = None
    interrupted = False
    csv_path = None
    try:
        print(f"Waiting for sensors {sen_fcu} and {sen_fcr} to come online...")
        if not wait_online(encoder, sensors):
            print(f"[ERR] sensors {sen_fcu}/{sen_fcr} did not come online. "
                  "Check wiring / slave ids.")
            return 1

        # --- Phase 1: ramp every muscle in, hold, then zero. ---------------
        print(f"Phase 1: ramping in over {INIT_RAMP_S:g}s "
              f"(FCU/FCR {PHASE1_KPA}, others fixed)...")
        init = [(reg_fcu, PHASE1_KPA), (reg_fcr, PHASE1_KPA)]
        init += [(muscles[n]["regulator"], kpa) for n, kpa in FIXED.items()]
        valve.set_multiple_valves(init, ramp=INIT_RAMP_S)
        sleep_or_abort(INIT_RAMP_S)
        print(f"Phase 1: holding {PHASE1_HOLD_S:g}s...")
        sleep_or_abort(PHASE1_HOLD_S)

        zero_counts, zerr = capture_zero(encoder, sensors)
        if zero_counts is None:
            print(f"[ERR] could not zero: {zerr}")
            return 1
        print(f"Zeroed at phase 1: FCU={zero_counts[sen_fcu]} counts, "
              f"FCR={zero_counts[sen_fcr]} counts. Recording started.")

        rec = Recorder(valve, encoder, (reg_fcu, reg_fcr), sensors, zero_counts,
                       args.counts_per_turn, args.drum_diameter, args.sample_hz)
        rec.start()

        if not args.no_live:
            recorded_s = RAMP_S + DWELL_S + args.cycles * 2 * (RAMP_S + DWELL_S)
            live = LivePlot(rec, recorded_s, args.live_fps)
            if live.open():
                _live = live      # sleep_or_abort pumps it from here on
                print(f"Live view open ({args.live_fps:g} fps). "
                      "Closing the window does not stop the run — use Ctrl-C.")

        def go(cycle, segment, kpa):
            """Ramp FCU/FCR to kpa over RAMP_S, then dwell DWELL_S.

            Both waits are abort-aware, so an e-stop unwinds here instead of
            letting the sequence command the next setpoint.
            """
            rec.set_context(cycle, f"{segment}_ramp", kpa, kpa)
            valve.set_multiple_valves([(reg_fcu, kpa), (reg_fcr, kpa)], ramp=RAMP_S)
            sleep_or_abort(RAMP_S)
            rec.set_context(cycle, f"{segment}_dwell", kpa, kpa)
            sleep_or_abort(DWELL_S)

        # --- Phase 1 -> phase 2 (approach, not counted as a cycle). --------
        print(f"Phase 1 -> 2: {PHASE1_KPA} -> {PHASE2_KPA} over {RAMP_S:g}s")
        go(0, "p1_to_p2", PHASE2_KPA)

        # --- 4 x (phase 2 -> phase 3 -> phase 2). --------------------------
        for cycle in range(1, args.cycles + 1):
            go(cycle, "p2_to_p3", PHASE3_KPA)
            go(cycle, "p3_to_p2", PHASE2_KPA)
            print(f"  cycle {cycle}/{args.cycles} done ({len(rec.rows)} samples)")

    except Aborted as exc:
        interrupted = True
        print(f"[STOP] run aborted ({exc}) — valves vented, nothing saved.")
    except KeyboardInterrupt:
        # Backstop: only reached if the SIGINT handler was not installed.
        interrupted = True
        print("\n[STOP] interrupted — emergency stop.")
        trigger_abort(valve, "KeyboardInterrupt")
    finally:
        if _live is not None:
            _live.close()
            _live = None
        if rec is not None:
            rec.stop()
        try:
            if interrupted or ABORT.is_set():
                # trigger_abort already sent "s"; repeat it as confirmation,
                # since this is the last chance to leave the rig depressurised.
                valve.emergency_stop()
            else:
                # Normal completion: vent every regulator this test drove.
                valve.set_multiple_valves([(r, "off") for r in all_regs], ramp=0.0)
        except Exception as exc:
            print(f"[WARN] could not vent valves: {exc}")
            print("[ERR] CHECK THE RIG — cut the air supply if still pressurised.")
        time.sleep(0.2)

        # An aborted run is an incomplete run: it has fewer than the requested
        # cycles and a truncated final ramp, so writing it would leave files
        # that look like real data. Discard the buffer instead — this covers
        # Ctrl-C, SIGTERM, the stop-file and the runtime watchdog alike.
        if interrupted or ABORT.is_set():
            n = len(rec.rows) if rec is not None else 0
            print(f"[STOP] discarded {n} sample(s); nothing written to "
                  f"{os.path.abspath(args.out)}")
        elif rec is not None and rec.rows:
            if confirm_save(len(rec.rows), os.path.abspath(args.out), args.yes):
                csv_path = save_csv(rec.rows, args.out)
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

    def col(name, cast=float):
        out = []
        for r in rows:
            v = r.get(name, "")
            try:
                out.append(cast(v))
            except (ValueError, TypeError):
                out.append(np.nan if cast is float else v)
        return np.array(out) if cast is float else out

    def cmd_col(name_kpa, name_mv):
        """Commanded pressure in kPa; CSVs from before the kPa switch hold mV."""
        if name_kpa in rows[0]:
            return col(name_kpa)
        return np.array([mv_to_kpa(v) for v in col(name_mv)])

    return {
        "t": col("elapsed_s"),
        "cycle": np.array([int(float(r["cycle"])) for r in rows]),
        "segment": [r["segment"] for r in rows],
        "fcu": col("len_FCU_mm"),
        "fcr": col("len_FCR_mm"),
        "cmd_fcu": cmd_col("cmd_FCU_kPa", "cmd_FCU_mV"),
        "cmd_fcr": cmd_col("cmd_FCR_kPa", "cmd_FCR_mV"),
        "tgt_fcu": col("target_FCU"),
    }


def plot_csv(path, show=False):
    """Three-panel figure: length vs time, pressure vs time, cycle overlay."""
    import matplotlib
    # Only force the headless backend if nothing has already set one up — the
    # live view imports pyplot under TkAgg, and switching out from under it
    # would break savefig.
    if not show and "matplotlib.pyplot" not in sys.modules:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    d = _read_csv(path)
    t, cyc = d["t"], d["cycle"]

    fig = plt.figure(figsize=(13, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.25, 0.85, 1.0], hspace=0.35,
                          wspace=0.22)
    ax_len = fig.add_subplot(gs[0, :])
    ax_p = fig.add_subplot(gs[1, :], sharex=ax_len)
    ax_fcu = fig.add_subplot(gs[2, 0])
    ax_fcr = fig.add_subplot(gs[2, 1])

    C_FCU, C_FCR = "#1f77b4", "#d62728"

    # -- Panel 1: length vs time, with dwell bands shaded. ------------------
    for i in range(len(t) - 1):
        if d["segment"][i].endswith("_dwell"):
            ax_len.axvspan(t[i], t[i + 1], color="0.85", lw=0, zorder=0)
    ax_len.plot(t, d["fcu"], color=C_FCU, lw=1.4, label="FCU (sensor 58)")
    ax_len.plot(t, d["fcr"], color=C_FCR, lw=1.4, label="FCR (sensor 59)")
    ax_len.axhline(0, color="0.5", lw=0.8, ls=":")
    for c in range(1, cyc.max() + 1):
        idx = np.where(cyc == c)[0]
        if len(idx):
            ax_len.axvline(t[idx[0]], color="0.6", lw=0.8, ls="--")
            ax_len.text(t[idx[0]], ax_len.get_ylim()[1], f" c{c}",
                        va="top", ha="left", fontsize=8, color="0.4")
    ax_len.set_ylabel("length (mm, rel. phase 1)")
    ax_len.set_title(f"FCU/FCR three-phase cycling — {os.path.basename(path)}\n"
                     "grey bands = dwell, dashed = cycle start")
    ax_len.legend(loc="best", fontsize=9)
    ax_len.grid(alpha=0.3)

    # -- Panel 2: commanded pressure vs time. ------------------------------
    ax_p.plot(t, d["cmd_fcu"], color=C_FCU, lw=1.2, label="FCU cmd")
    ax_p.plot(t, d["cmd_fcr"], color=C_FCR, lw=1.2, ls="--", label="FCR cmd")
    ax_p.plot(t, d["tgt_fcu"], color="0.4", lw=0.9, ls=":", label="target")
    for kpa, lbl in ((PHASE2_KPA, "p2 up"), (PHASE3_KPA, "p3 down")):
        ax_p.axhline(kpa, color="0.7", lw=0.7)
        ax_p.text(t[-1], kpa, f" {lbl}", va="center", fontsize=8, color="0.45")
    ax_p.set_ylabel("setpoint (kPa)")
    ax_p.set_xlabel("time since zero (s)")
    ax_p.legend(loc="best", fontsize=8, ncol=3)
    ax_p.grid(alpha=0.3)

    # -- Panel 3: the 4 cycles overlaid, one axes per sensor. --------------
    cycles = [c for c in range(1, cyc.max() + 1) if np.any(cyc == c)]
    cmap = plt.get_cmap("viridis")
    for ax, key, name, colr in ((ax_fcu, "fcu", "FCU (58)", C_FCU),
                                (ax_fcr, "fcr", "FCR (59)", C_FCR)):
        for n, c in enumerate(cycles):
            idx = np.where(cyc == c)[0]
            if not len(idx):
                continue
            shade = cmap(n / max(1, len(cycles) - 1))
            ax.plot(t[idx] - t[idx[0]], d[key][idx], lw=1.3, color=shade,
                    label=f"cycle {c}")
        ax.set_title(f"{name} — cycles overlaid", fontsize=10, color=colr)
        ax.set_xlabel("time within cycle (s)")
        ax.set_ylabel("length (mm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

    png = os.path.splitext(path)[0] + "_cycles.png"
    fig.savefig(png, dpi=140, bbox_inches="tight")
    print(f"Saved plot to {os.path.abspath(png)}")

    _print_stats(d, cycles)
    if show:
        plt.show()
    plt.close(fig)
    return png


def _print_stats(d, cycles):
    """Per-cycle extremes and cycle-to-cycle spread, to 4 dp."""
    import numpy as np

    print(f"\nPer-cycle length extremes (mm, {LEN_DECIMALS} dp)")
    print(f"{'cycle':>5}  {'FCU min':>10} {'FCU max':>10} {'FCU p-p':>10}"
          f"  {'FCR min':>10} {'FCR max':>10} {'FCR p-p':>10}")
    mins = {"fcu": [], "fcr": []}
    maxs = {"fcu": [], "fcr": []}
    for c in cycles:
        idx = np.where(d["cycle"] == c)[0]
        cells = []
        for key in ("fcu", "fcr"):
            v = d[key][idx]
            v = v[~np.isnan(v)]
            if not len(v):
                cells += ["–"] * 3
                continue
            mins[key].append(v.min())
            maxs[key].append(v.max())
            cells += [f"{v.min():.4f}", f"{v.max():.4f}", f"{v.max() - v.min():.4f}"]
        print(f"{c:>5}  " + " ".join(f"{x:>10}" for x in cells))

    print("\nCycle-to-cycle repeatability (spread of the per-cycle extremes)")
    for key, name in (("fcu", "FCU"), ("fcr", "FCR")):
        if len(mins[key]) < 2:
            continue
        lo, hi = np.array(mins[key]), np.array(maxs[key])
        print(f"  {name}: spread {hi.max() - hi.min():.4f} mm across the "
              f"per-cycle maxima, {lo.max() - lo.min():.4f} mm across the minima; "
              f"drift {lo[-1] - lo[0]:+.4f} mm from cycle 1 to {len(lo)}")


# ---- CLI ------------------------------------------------------------------

def build_parser():
    from encoder_controller import DEFAULT_COUNTS_PER_TURN, DEFAULT_DRUM_DIAMETER_MM

    p = argparse.ArgumentParser(
        description="Three-phase FCU/FCR cycling test (sensors 58/59)")
    p.add_argument("--valve-port", default=None, help="Giga R1 regulator port")
    p.add_argument("--rs485-port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--cycles", type=int, default=CYCLES,
                   help="number of p2->p3->p2 cycles (default 4)")
    p.add_argument("--sample-hz", type=float, default=SAMPLE_HZ,
                   help="recording rate")
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
                   help="hard ceiling in seconds before the run self-aborts "
                        "(default: planned duration + 30 s)")
    p.add_argument("--dry-run", action="store_true",
                   help="resolve the mapping and print the plan, touch no hardware")
    p.add_argument("-y", "--yes", action="store_true",
                   help="save without asking (default is to prompt after the run)")
    p.add_argument("--no-live", action="store_true",
                   help="skip the live window; record and plot at the end only")
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
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
