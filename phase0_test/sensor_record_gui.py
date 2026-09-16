#!/usr/bin/env python3
"""
sensor_record_gui.py

Tk GUI for ONE GJW RS-485 encoder on the EKU081 adapter: samples the slave on
a fixed 100 Hz clock, shows the last 10 s as a rolling plot in millimetres
(3 decimals), and records stretches of it to CSV.

Conversion is the same draw-wire arc length the web app uses:

    position_mm = (raw - zero) / counts_per_turn * pi * drum_diameter_mm

`drum_diameter_mm` is editable live in the GUI (default 14.0, the app's
DEFAULT_DRUM_DIAMETER_MM); changing it rescales the plot and every later
sample. The raw counts are always kept alongside, so a recording can be
re-converted afterwards if the diameter turns out to be wrong.

GUI
    Zero               offset = current raw value (display + recording zero)
    Clear zero         back to absolute counts
    Start recording    begin buffering samples (no file yet)
    End recording      stop, then a save-as dialog; Cancel discards the run
    Drum diameter      textbox; Enter or Apply to take effect
    keys               z = zero, c = clear zero, r = start/end recording,
                       Esc = quit

CSV columns (one row per 100 Hz tick):
    wall_time         ISO-8601 local time with microseconds
    elapsed_s         seconds since the recording started
    raw_counts        accumulative encoder position
    zero_counts       offset in force when the row was taken
    drum_diameter_mm  diameter in force when the row was taken
    position_mm       (raw - zero) converted with that diameter, 3 dp

Default save location is phase0_test/phase0_data/ (git-ignored).

Sampling has priority over the display. The reader thread owns the clock and
does nothing but read, timestamp and append raw tuples; CSV formatting happens
at save time, the plot copies the buffer under a short lock and does its
arithmetic outside it, and the interpreter switch interval is shortened so a
tick is not held up behind a redraw. The bus does one 16-register state read
in ~5-7 ms at 115200 baud, so a 100 Hz clock leaves margin. A tick whose read
fails (timeout / CRC) is skipped, not repeated, so the timeline stays regular;
the status line shows the achieved rate, read errors and the number of ticks
that arrived late (> 1.5 periods after the previous one), and the same numbers
are printed when a recording is saved.

app.py (or another test script) must NOT be running: it would hold the port.

This is a standalone calibration aid (mainly for pinning down the drum
diameter); it does not use the muscle/sensor mapping. The one sensor to read is
given by its RS-485 slave id with -s / --sensor.

Usage (runnable from any working directory):
    python phase0_test/sensor_record_gui.py -s 58
    python phase0_test/sensor_record_gui.py -s 58 --port /dev/ttyCH9344USB0
    python phase0_test/sensor_record_gui.py -s 59 --diameter 13
    python phase0_test/sensor_record_gui.py --list-ports
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

from encoder_controller import (  # noqa: E402
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    list_ch9344_ports,
)
from modbus_rtu import (  # noqa: E402
    GJW_STATE_COUNT,
    GJW_STATE_REGISTER,
    READ_FUNCTIONS,
    SerialTimeoutError,
    build_read_request,
    compute_absolute_position,
    decode_gjw_state_registers,
    parse_read_registers_response,
)
from sensor_live_plot import find_rs485_port  # noqa: E402

# Let the sampling thread pre-empt the GUI thread promptly: the default 5 ms
# switch interval is the same order as the 10 ms sample period.
sys.setswitchinterval(0.0005)


SAMPLE_HZ = 100.0       # reader clock
WINDOW_S = 10.0         # seconds of history shown
FPS = 20.0              # plot redraws per second (display only)
MIN_SPAN_MM = 0.5       # smallest y range so noise on a still sensor
                        # doesn't fill the plot
DECIMALS = 3

DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "phase0_data")
CSV_COLUMNS = ["wall_time", "elapsed_s", "raw_counts", "zero_counts",
               "drum_diameter_mm", "position_mm"]


# ---- Reader ---------------------------------------------------------------

class PacedReader:
    """Owns the port; reads one slave on a fixed clock in a background thread.

    Each plot sample is (perf_t, wall_t, raw_counts) and the buffer keeps the
    last `keep_s`. While recording, every sample is also appended as
    (perf_t, wall_t, raw, zero_counts, drum_diameter_mm) — plain tuples, no
    formatting on the sampling thread.
    """

    def __init__(self, port, slave, baudrate=115200, parity="N",
                 counts_per_turn=DEFAULT_COUNTS_PER_TURN,
                 drum_diameter_mm=DEFAULT_DRUM_DIAMETER_MM,
                 sample_hz=SAMPLE_HZ, keep_s=WINDOW_S, timeout=0.06):
        self.slave = slave
        self.counts_per_turn = counts_per_turn
        self.drum_diameter_mm = float(drum_diameter_mm)
        self.period = 1.0 / sample_hz
        self.keep_s = keep_s

        self.ser = serial.Serial(port=port, baudrate=baudrate, bytesize=8,
                                 parity=parity, stopbits=1, timeout=timeout)
        self.port_name = port

        self.samples = deque()          # rolling window for the plot
        self.recording = None           # list of tuples while recording
        self.rec_t0 = None              # perf_counter at Start recording
        self.rec_wall0 = None
        self.offset = 0
        self.zeroed = False
        self.errors = 0
        self.last_error = ""
        self.late_ticks = 0             # samples > 1.5 periods after the last
        self._last_t = None

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self.t0 = time.perf_counter()

    # -- one Modbus transaction (same framing as EncoderController) ---------

    def _read_raw(self):
        request = build_read_request(self.slave, READ_FUNCTIONS["holding"],
                                     GJW_STATE_REGISTER, GJW_STATE_COUNT)
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self.ser.flush()

        header = self.ser.read(3)
        if len(header) != 3:
            raise SerialTimeoutError(
                f"timeout: got {len(header)} of 3 header byte(s)")
        body = self.ser.read(2 if header[1] & 0x80 else header[2] + 2)

        registers = parse_read_registers_response(
            header + body, self.slave, READ_FUNCTIONS["holding"],
            GJW_STATE_COUNT)
        state = decode_gjw_state_registers(registers)
        return compute_absolute_position(
            single_turn_position=state["single_turn_position"],
            turns=state["turns"],
            counts_per_turn=self.counts_per_turn,
        )

    # -- sampling loop -----------------------------------------------------

    def _loop(self):
        next_tick = time.perf_counter()
        while not self._stop.is_set():
            try:
                raw = self._read_raw()
            except Exception as exc:
                with self._lock:
                    self.errors += 1
                    self.last_error = str(exc).replace("\n", " ")[:60]
                raw = None

            if raw is not None:
                now = time.perf_counter()
                wall = time.time()
                if (self._last_t is not None
                        and now - self._last_t > 1.5 * self.period):
                    self.late_ticks += 1
                self._last_t = now
                with self._lock:
                    self.samples.append((now - self.t0, wall, raw))
                    cutoff = now - self.t0 - self.keep_s
                    while self.samples and self.samples[0][0] < cutoff:
                        self.samples.popleft()
                    if self.recording is not None:
                        self.recording.append(
                            (now, wall, raw, self.offset,
                             self.drum_diameter_mm))

            # Fixed-rate clock: if a read overran, skip the missed ticks
            # rather than bunching catch-up reads together.
            next_tick += self.period
            now = time.perf_counter()
            if next_tick < now:
                next_tick = now + self.period
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    # -- API used by the GUI -----------------------------------------------

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.ser.close()

    def wait_online(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self.samples:
                    return True
            time.sleep(0.05)
        return False

    def snapshot(self):
        """(times, mm_values, now, current_mm, zeroed, errors, last_error,
        late_ticks, rec_count, rec_elapsed).

        Only the buffer copy happens under the lock; the mm conversion runs
        on the GUI thread afterwards so the sampler is never held up by it.
        """
        with self._lock:
            samples = list(self.samples)
            d, offset, zeroed = self.drum_diameter_mm, self.offset, self.zeroed
            if self.recording is not None:
                rec_count = len(self.recording)
                rec_elapsed = time.perf_counter() - self.rec_t0
            else:
                rec_count, rec_elapsed = None, None
        now = time.perf_counter() - self.t0
        scale = math.pi * d / self.counts_per_turn
        times = [smp[0] for smp in samples]
        mm = [(smp[2] - offset) * scale for smp in samples]
        return (times, mm, now, mm[-1] if mm else None, zeroed,
                self.errors, self.last_error, self.late_ticks,
                rec_count, rec_elapsed)

    def zero(self):
        with self._lock:
            if not self.samples:
                return False
            self.offset = self.samples[-1][2]
            self.zeroed = True
            return True

    def clear_zero(self):
        with self._lock:
            self.offset = 0
            self.zeroed = False

    def set_diameter(self, diameter_mm):
        with self._lock:
            self.drum_diameter_mm = float(diameter_mm)

    @property
    def is_recording(self):
        return self.recording is not None

    def start_recording(self):
        with self._lock:
            self.recording = []
            self.rec_t0 = time.perf_counter()
            self.rec_wall0 = time.time()

    def end_recording(self):
        """Stop buffering; return (samples, perf_start, wall_start)."""
        with self._lock:
            rows, t0, wall0 = self.recording, self.rec_t0, self.rec_wall0
            self.recording = None
            self.rec_t0 = self.rec_wall0 = None
        return rows or [], t0, wall0

    def format_rows(self, samples, t0):
        """Recorded tuples -> CSV rows (done off the sampling thread)."""
        out = []
        for now, wall, raw, zero, d in samples:
            mm = (raw - zero) / self.counts_per_turn * math.pi * d
            out.append([
                datetime.fromtimestamp(wall).isoformat(timespec="microseconds"),
                f"{now - t0:.4f}",
                raw,
                zero,
                f"{d:g}",
                f"{mm:.{DECIMALS}f}",
            ])
        return out


# ---- GUI ------------------------------------------------------------------

class App:
    def __init__(self, root, reader, args):
        self.root = root
        self.reader = reader
        self.args = args
        self.out_dir = args.out_dir

        root.title(f"sensor {reader.slave} @ {reader.port_name} — mm recorder")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # -- top bar: live value + controls ---------------------------------
        top = ttk.Frame(root, padding=(8, 6))
        top.pack(side=tk.TOP, fill=tk.X)

        self.value_var = tk.StringVar(value="—")
        ttk.Label(top, textvariable=self.value_var,
                  font=("TkFixedFont", 22, "bold"), width=14,
                  anchor="e").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(top, text="mm", font=("TkDefaultFont", 12)).pack(
            side=tk.LEFT, padx=(0, 16))

        self.b_zero = ttk.Button(top, text="Zero", command=self.do_zero)
        self.b_zero.pack(side=tk.LEFT, padx=2)
        self.b_clear = ttk.Button(top, text="Clear zero",
                                  command=self.do_clear_zero)
        self.b_clear.pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y,
                                                    padx=10)

        self.b_start = ttk.Button(top, text="Start recording",
                                  command=self.do_start_rec)
        self.b_start.pack(side=tk.LEFT, padx=2)
        self.b_end = ttk.Button(top, text="End recording",
                                command=self.do_end_rec, state=tk.DISABLED)
        self.b_end.pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y,
                                                    padx=10)

        ttk.Label(top, text="Drum diameter (mm):").pack(side=tk.LEFT)
        self.diam_var = tk.StringVar(value=f"{reader.drum_diameter_mm:g}")
        self.diam_entry = ttk.Entry(top, textvariable=self.diam_var, width=7)
        self.diam_entry.pack(side=tk.LEFT, padx=(4, 2))
        self.diam_entry.bind("<Return>", self.apply_diameter)
        self.diam_entry.bind("<FocusOut>", self.apply_diameter)
        ttk.Button(top, text="Apply", command=self.apply_diameter,
                   width=6).pack(side=tk.LEFT)

        # -- status line ----------------------------------------------------
        self.status_var = tk.StringVar(value="")
        ttk.Label(root, textvariable=self.status_var, font="TkFixedFont",
                  padding=(10, 0)).pack(side=tk.TOP, fill=tk.X)

        # -- plot -----------------------------------------------------------
        self.fig = Figure(figsize=(10, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        (self.line,) = self.ax.plot([], [], lw=1.2, color="#1f77b4")
        self.ax.set_xlim(-args.window, 0)
        self.ax.set_ylim(-MIN_SPAN_MM / 2, MIN_SPAN_MM / 2)
        self.ax.set_xlabel("time (s, 0 = now)")
        self.ax.set_ylabel("position (mm)")
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(0, color="0.6", lw=0.8)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH,
                                         expand=True)
        self.canvas.draw()

        # Key shortcuts, ignored while the diameter box is being typed into
        # (Apply / Enter hands focus back to the window).
        root.bind("<z>", self._hotkey(self.do_zero))
        root.bind("<c>", self._hotkey(self.do_clear_zero))
        root.bind("<r>", self._hotkey(self.toggle_rec))
        root.bind("<Escape>", lambda _e: self.on_close())

        self._after_id = None
        self._tick()

    # -- button handlers ----------------------------------------------------

    def _hotkey(self, fn):
        def handler(_evt):
            if self.root.focus_get() is not self.diam_entry:
                fn()
        return handler

    def do_zero(self):
        if not self.reader.zero():
            self.status_var.set("cannot zero: no sample yet")

    def do_clear_zero(self):
        self.reader.clear_zero()

    def apply_diameter(self, _evt=None):
        text = self.diam_var.get().strip()
        try:
            d = float(text)
            if not d > 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Drum diameter",
                                 f"'{text}' is not a positive number.")
            self.diam_var.set(f"{self.reader.drum_diameter_mm:g}")
            return
        self.reader.set_diameter(d)
        self.diam_var.set(f"{d:g}")
        self.root.focus_set()  # take focus off the entry so keys work again

    def toggle_rec(self):
        if self.reader.is_recording:
            self.do_end_rec()
        else:
            self.do_start_rec()

    def do_start_rec(self):
        if self.reader.is_recording:
            return
        self.reader.start_recording()
        self.b_start.config(state=tk.DISABLED)
        self.b_end.config(state=tk.NORMAL)

    def do_end_rec(self):
        if not self.reader.is_recording:
            return
        samples, t0, wall0 = self.reader.end_recording()
        self.b_start.config(state=tk.NORMAL)
        self.b_end.config(state=tk.DISABLED)
        if not samples:
            messagebox.showinfo("Recording", "No samples were recorded.")
            return
        self.save_rows(samples, t0, wall0)

    def save_rows(self, samples, t0, wall0):
        rows = self.reader.format_rows(samples, t0)
        span = samples[-1][0] - samples[0][0]
        rate = (len(samples) - 1) / span if span > 0 else 0.0
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        late = sum(1 for g in gaps if g > 1.5 * self.reader.period)
        summary = (f"{len(rows)} rows, {span:.2f} s, {rate:.1f} Hz, "
                   f"{late} late tick(s), max gap "
                   f"{max(gaps) * 1000 if gaps else 0:.1f} ms")

        stamp = datetime.fromtimestamp(wall0).strftime("%Y%m%d-%H%M%S")
        default = f"sensor{self.reader.slave}_{stamp}.csv"
        os.makedirs(self.out_dir, exist_ok=True)
        path = filedialog.asksaveasfilename(
            parent=self.root, title=f"Save recording ({summary})",
            initialdir=self.out_dir, initialfile=default,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*")])
        if not path:
            self.status_var.set(f"recording discarded ({summary})")
            print(f"[--] discarded recording ({summary})")
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(CSV_COLUMNS)
            w.writerows(rows)
        self.out_dir = os.path.dirname(path)
        self.status_var.set(f"saved {summary} -> {path}")
        print(f"[OK] saved {summary} -> {path}")

    def on_close(self):
        if self.reader.is_recording:
            keep = messagebox.askyesnocancel(
                "Recording in progress",
                "A recording is still running.\nSave it before quitting?")
            if keep is None:
                return
            samples, t0, wall0 = self.reader.end_recording()
            if keep and samples:
                self.save_rows(samples, t0, wall0)
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        self.root.destroy()

    # -- periodic redraw ----------------------------------------------------

    def _tick(self):
        (times, mm, now, cur, zeroed, errors, last_error, late,
         rec_count, rec_elapsed) = self.reader.snapshot()

        if cur is None:
            self.value_var.set("—")
            self.status_var.set(f"no data from slave {self.reader.slave}   "
                                f"errors {errors}  {last_error}")
        else:
            self.value_var.set(f"{cur:+.{DECIMALS}f}")
            ages = [t - now for t in times]
            self.line.set_data(ages, mm)

            lo, hi = min(mm), max(mm)
            mid, span = (lo + hi) / 2, max(hi - lo, MIN_SPAN_MM)
            pad = span * 0.15
            self.ax.set_ylim(mid - span / 2 - pad, mid + span / 2 + pad)

            elapsed = times[-1] - times[0]
            rate = (len(times) - 1) / elapsed if elapsed > 0 else 0.0
            rec = (f"REC {rec_elapsed:6.1f} s  {rec_count} rows"
                   if rec_count is not None else "idle")
            self.status_var.set(
                f"{'zeroed' if zeroed else 'absolute'}   "
                f"{rate:5.1f} Hz   errors {errors}   late {late}   "
                f"d = {self.reader.drum_diameter_mm:g} mm   |   {rec}")
            self.canvas.draw_idle()

        self._after_id = self.root.after(int(1000 / FPS), self._tick)


# ---- Main -----------------------------------------------------------------

def run(args):
    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        for d in list_ch9344_ports():
            print(f"  {d} - CH9344 USB-RS485 (EKU081)")
        return 0

    if args.sensor is None:
        print("[ERR] give the sensor's slave id, e.g. "
              "`python phase0_test/sensor_record_gui.py -s 58`")
        return 2
    if not 1 <= args.sensor <= 247:
        print(f"[ERR] slave id {args.sensor} outside the Modbus range 1-247")
        return 2

    port = args.port or find_rs485_port()
    if not port:
        print("[ERR] no RS-485 adapter found. Pass --port, or run "
              "--list-ports to see what is attached.")
        return 2

    try:
        reader = PacedReader(port, args.sensor, baudrate=args.baud,
                             parity=args.parity,
                             counts_per_turn=args.counts_per_turn,
                             drum_diameter_mm=args.diameter,
                             sample_hz=args.rate, keep_s=args.window)
    except serial.SerialException as exc:
        print(f"[ERR] could not open {port}: {exc}\n"
              "      Is app.py (or another test script) still running?")
        return 1

    print(f"Reading slave {args.sensor} on {port} @ {args.baud} "
          f"8{args.parity}1, {args.rate:g} Hz, drum {args.diameter:g} mm")
    reader.start()
    try:
        if not reader.wait_online():
            print(f"[ERR] slave {args.sensor} did not answer "
                  f"({reader.errors} failed reads: {reader.last_error}). "
                  "Check the slave id, wiring and port.")
            return 1
        root = tk.Tk()
        App(root, reader, args)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        description="GUI: live mm plot + CSV recorder for one RS-485 encoder")
    p.add_argument("-s", "--sensor", type=lambda v: int(v, 0), default=None,
                   metavar="ID", help="RS-485 slave id of the sensor to read "
                                      "(e.g. 58)")
    p.add_argument("--port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--parity", default="N", choices=["N", "E", "O"])
    p.add_argument("--counts-per-turn", type=lambda v: int(v, 0),
                   default=DEFAULT_COUNTS_PER_TURN)
    p.add_argument("--diameter", type=float, default=DEFAULT_DRUM_DIAMETER_MM,
                   help="initial drum diameter in mm (default 14.0; "
                        "editable in the GUI)")
    p.add_argument("--rate", type=float, default=SAMPLE_HZ,
                   help="sample clock in Hz (default 100)")
    p.add_argument("--window", type=float, default=WINDOW_S,
                   help="seconds of history shown (default 10)")
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR,
                   help="initial folder for the save dialog")
    p.add_argument("--list-ports", action="store_true",
                   help="list serial ports and exit")
    return p


def main(argv=None):
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
