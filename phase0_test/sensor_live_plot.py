#!/usr/bin/env python3
"""
sensor_live_plot.py

Real-time scope for ONE GJW RS-485 encoder: reads a single slave id as fast as
the bus answers and plots the RAW ACCUMULATIVE position

    value = single_turn_position + counts_per_turn * turns

(the same number encoder_controller calls `absolute_position`, in encoder
counts — no mm conversion) over a rolling 8-second window.

Why a dedicated reader instead of EncoderController: that class polls every
configured slave on a shared thread and only keeps the latest sample, so its
rate is capped by the poll interval. Here one thread owns the port and loops on
a single slave with no sleep, which is what "high enough sampling rate" needs
(~150-200 Hz at 115200 baud for the 16-register state block). Acquisition and
drawing are decoupled: samples land in a ring buffer at bus rate, the plot
redraws at --fps.

Zeroing is a display offset captured from the newest sample, so it is instant,
repeatable, and reversible — the encoder itself is never written to.

The web app (app.py) must NOT be running at the same time; it would hold the
RS-485 port.

Controls
    z / Zero button     zero here (offset = current raw value)
    c / Clear button    clear the offset (show absolute counts again)
    f / Freeze y button hold the y-axis range where it is; press again to
                        release and resume autoscaling. Frozen also means the
                        toolbar's zoom/pan on y survives the next redraw.
    q                   quit

Usage (runnable from any working directory):
    python phase0_test/sensor_live_plot.py 57
    python phase0_test/sensor_live_plot.py 57 --port /dev/ttyCH9344USB0
    python phase0_test/sensor_live_plot.py 66 --window 8 --fps 30 --zero-at-start
    python phase0_test/sensor_live_plot.py --list-ports
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from collections import deque

# This script lives in phase0_test/ but drives the app's serial layer, which
# lives one level up — put the project root on sys.path before importing it so
# it runs from any working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    is_ch9344_port,
    list_ch9344_ports,
)
from modbus_rtu import (
    GJW_STATE_COUNT,
    GJW_STATE_REGISTER,
    READ_FUNCTIONS,
    SerialTimeoutError,
    build_read_request,
    compute_absolute_position,
    decode_gjw_state_registers,
    parse_read_registers_response,
)


WINDOW_S = 8.0      # seconds of history shown (and kept)
FPS = 30.0          # plot redraws per second; independent of the sample rate
MIN_SPAN = 200      # smallest y range (counts), so a still sensor's noise
                    # doesn't get autoscaled up into a full-height wiggle


# ---- Serial port detection (mirrors Phase0test.py, kept standalone) --------

def _blob(p):
    return " ".join(str(x) for x in (
        p.description, getattr(p, "product", None),
        getattr(p, "manufacturer", None), p.hwid)).lower()


def find_rs485_port():
    """Best guess at the USB-RS-485 adapter node, EKU081 (CH9344) first."""
    # The EKU081's out-of-tree driver may not show up in comports(), so glob
    # /dev for its nodes and take the lowest port number.
    ch9344 = list_ch9344_ports()
    if ch9344:
        return ch9344[0]
    ports = [p for p in serial.tools.list_ports.comports()
             if ("ttyACM" in p.device or "ttyUSB" in p.device
                 or p.device.upper().startswith("COM"))
             and not is_ch9344_port(p.device)]
    return next((p.device for p in ports
                 if any(k in _blob(p) for k in (
                     "1a86", "ch340", "ch343", "ch9344", "single serial",
                     "0403", "ftdi", "10c4", "cp210"))),
                ports[0].device if ports else None)


# ---- Reader ---------------------------------------------------------------

class SensorReader:
    """Owns the serial port and hammers one slave id in a background thread."""

    def __init__(self, port, slave, baudrate=115200, parity="N",
                 counts_per_turn=DEFAULT_COUNTS_PER_TURN, timeout=0.06,
                 keep_s=WINDOW_S):
        self.slave = slave
        self.counts_per_turn = counts_per_turn
        self.keep_s = keep_s

        self.ser = serial.Serial(port=port, baudrate=baudrate, bytesize=8,
                                 parity=parity, stopbits=1, timeout=timeout)
        self.port_name = port

        # (t, raw_value) newest last; trimmed to keep_s by the reader thread.
        self.samples = deque()
        self.offset = 0              # display zero, in raw counts
        self.zeroed = False
        self.errors = 0              # cumulative failed reads
        self.last_error = ""

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self.t0 = time.perf_counter()

    # -- one Modbus transaction (same framing as EncoderController) ---------

    def _read_raw(self):
        """Return the raw accumulative position, or raise."""
        request = build_read_request(self.slave, READ_FUNCTIONS["holding"],
                                     GJW_STATE_REGISTER, GJW_STATE_COUNT)
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self.ser.flush()

        header = self.ser.read(3)
        if len(header) != 3:
            raise SerialTimeoutError(f"timeout: got {len(header)} of 3 header byte(s)")
        body = self.ser.read(2 if header[1] & 0x80 else header[2] + 2)

        registers = parse_read_registers_response(
            header + body, self.slave, READ_FUNCTIONS["holding"], GJW_STATE_COUNT)
        state = decode_gjw_state_registers(registers)
        return compute_absolute_position(
            single_turn_position=state["single_turn_position"],
            turns=state["turns"],
            counts_per_turn=self.counts_per_turn,
        )

    def _loop(self):
        while not self._stop.is_set():
            try:
                raw = self._read_raw()
            except Exception as exc:
                with self._lock:
                    self.errors += 1
                    self.last_error = str(exc).replace("\n", " ")[:60]
                continue
            now = time.perf_counter() - self.t0
            with self._lock:
                self.samples.append((now, raw))
                cutoff = now - self.keep_s
                while self.samples and self.samples[0][0] < cutoff:
                    self.samples.popleft()

    # -- API used by the plot ----------------------------------------------

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.ser.close()

    def snapshot(self):
        """(times, values, now, offset, zeroed, errors, last_error)."""
        with self._lock:
            times = [t for t, _ in self.samples]
            values = [v for _, v in self.samples]
            return (times, values, time.perf_counter() - self.t0, self.offset,
                    self.zeroed, self.errors, self.last_error)

    def zero(self):
        """Capture the newest raw value as the display zero."""
        with self._lock:
            if not self.samples:
                return False
            self.offset = self.samples[-1][1]
            self.zeroed = True
            return True

    def clear_zero(self):
        with self._lock:
            self.offset = 0
            self.zeroed = False

    def wait_online(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self.samples:
                    return True
            time.sleep(0.05)
        return False


# ---- Plot -----------------------------------------------------------------

def run_plot(reader, args):
    """Rolling-window scope; x is age in seconds so xlim never moves."""
    fig, ax = plt.subplots(figsize=(10, 5.5))
    fig.canvas.manager.set_window_title(f"sensor {reader.slave} live")
    fig.subplots_adjust(bottom=0.2)

    (line,) = ax.plot([], [], lw=1.2, color="#1f77b4")
    ax.set_xlim(-args.window, 0)
    ax.set_ylim(-MIN_SPAN / 2, MIN_SPAN / 2)
    ax.set_xlabel("time (s, 0 = now)")
    ax.set_ylabel("raw position (counts)")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="0.6", lw=0.8)
    ax.set_title(f"sensor {reader.slave} @ {reader.port_name} — "
                 f"raw = single_turn + {reader.counts_per_turn} * turns")

    readout = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", ha="left",
                      family="monospace", fontsize=11,
                      bbox=dict(fc="white", ec="0.8", alpha=0.85))
    hint = ax.text(0.99, 0.03, "z = zero   c = clear zero   f = freeze y   q = quit",
                   transform=ax.transAxes, va="bottom", ha="right",
                   fontsize=9, color="0.45")

    # While frozen, update() simply stops touching the y-limits, so whatever is
    # on screen stays — including a range the toolbar's zoom/pan set by hand.
    frozen = {"on": False}

    b_freeze = Button(fig.add_axes([0.54, 0.03, 0.14, 0.06]), "Freeze y (f)")
    b_zero = Button(fig.add_axes([0.70, 0.03, 0.11, 0.06]), "Zero (z)")
    b_clear = Button(fig.add_axes([0.82, 0.03, 0.13, 0.06]), "Clear (c)")

    def toggle_freeze(_evt=None):
        frozen["on"] = not frozen["on"]
        b_freeze.label.set_text("Release y (f)" if frozen["on"] else "Freeze y (f)")
        fig.canvas.draw_idle()

    b_freeze.on_clicked(toggle_freeze)
    b_zero.on_clicked(lambda _evt: reader.zero())
    b_clear.on_clicked(lambda _evt: reader.clear_zero())

    def on_key(event):
        if event.key == "z":
            reader.zero()
        elif event.key == "c":
            reader.clear_zero()
        elif event.key == "f":
            toggle_freeze()
        elif event.key in ("q", "escape"):
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        times, values, now, offset, zeroed, errors, last_error = reader.snapshot()
        if not times:
            readout.set_text(f"no data from slave {reader.slave}\n"
                             f"errors {errors}  {last_error}")
            return line, readout

        ages = [t - now for t in times]          # -window .. 0
        shown = [v - offset for v in values]
        line.set_data(ages, shown)

        if not frozen["on"]:
            lo, hi = min(shown), max(shown)
            mid, span = (lo + hi) / 2, max(hi - lo, args.min_span)
            pad = span * 0.15
            ax.set_ylim(mid - span / 2 - pad, mid + span / 2 + pad)

        # Rate over the window actually held, not the nominal one.
        elapsed = times[-1] - times[0]
        rate = (len(times) - 1) / elapsed if elapsed > 0 else 0.0
        readout.set_text(
            f"{shown[-1]:+12,d} counts {'(zeroed)' if zeroed else '(absolute)'}\n"
            f"{rate:6.1f} Hz   {len(times)} pts   errors {errors}"
            f"{'   [y frozen]' if frozen['on'] else ''}")
        return line, readout

    # blit stays off: the y-limits move every frame, which invalidates a
    # blitted background anyway.
    anim = FuncAnimation(fig, update, interval=1000.0 / args.fps,
                         blit=False, cache_frame_data=False)
    fig._live_anim = anim  # keep a reference so it isn't garbage collected
    plt.show()


# ---- Main -----------------------------------------------------------------

def run(args):
    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        for d in list_ch9344_ports():
            print(f"  {d} - CH9344 USB-RS485 (EKU081)")
        return 0

    if args.sensor is None:
        print("[ERR] give a sensor slave id, e.g. "
              "`python phase0_test/sensor_live_plot.py 57`")
        return 2

    port = args.port or find_rs485_port()
    if not port:
        print("[ERR] no RS-485 adapter found. Pass --port, or run "
              "--list-ports to see what is attached.")
        return 2

    try:
        reader = SensorReader(port, args.sensor, baudrate=args.baud,
                              parity=args.parity,
                              counts_per_turn=args.counts_per_turn,
                              keep_s=args.window)
    except serial.SerialException as exc:
        print(f"[ERR] could not open {port}: {exc}\n"
              "      Is app.py (or another test script) still running?")
        return 1

    print(f"Reading slave {args.sensor} on {port} @ {args.baud} 8{args.parity}1")
    reader.start()
    try:
        if not reader.wait_online():
            print(f"[ERR] slave {args.sensor} did not answer "
                  f"({reader.errors} failed reads: {reader.last_error}). "
                  "Check the slave id, wiring and port.")
            return 1
        if args.zero_at_start:
            reader.zero()
            print("Zeroed at start.")
        run_plot(reader, args)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        description="Real-time plot of one RS-485 encoder's raw accumulative position")
    p.add_argument("sensor", nargs="?", type=lambda v: int(v, 0),
                   help="RS-485 slave id to read (e.g. 57)")
    p.add_argument("--port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--parity", default="N", choices=["N", "E", "O"])
    p.add_argument("--counts-per-turn", type=lambda v: int(v, 0),
                   default=DEFAULT_COUNTS_PER_TURN,
                   help="max single-turn count used in pos + maxpos*turns")
    p.add_argument("--window", type=float, default=WINDOW_S,
                   help="seconds of history shown (default 8)")
    p.add_argument("--fps", type=float, default=FPS, help="plot redraws per second")
    p.add_argument("--min-span", type=float, default=MIN_SPAN,
                   help="smallest y-axis range in counts")
    p.add_argument("--zero-at-start", action="store_true",
                   help="zero on the first sample instead of waiting for 'z'")
    p.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    return p


def main(argv=None):
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
