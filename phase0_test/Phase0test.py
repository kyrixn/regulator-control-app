#!/usr/bin/env python3
"""
Phase0test.py

Open-loop characterisation run for the two antagonist muscles R57 and R58
(regulators mapped to draw-wire sensors 57 and 58 in sensor_mapping.json).

Sequence
--------
1. Pre-inflate R57 and R58 to 1950, hold 2 s, then zero sensors 57 and 58.
2. Oscillate between two antagonist setpoints, each reached by a 1 s linear ramp:
       A = (R57, R58) = (2200, 1850)
       B = (R57, R58) = (1850, 2250)
   One cycle = ramp to A, pause 0.75 s, ramp to B, pause 0.75 s.
   Repeat for 20 cycles (40 ramps, ~70 s).
3. From the instant of zeroing, sample both sensors' length (mm, relative to
   zero) and the commanded regulator setpoints, and save everything to a CSV
   on disk.
4. On normal completion vent both regulators to 0; Ctrl-C / errors trigger an
   emergency stop.

This talks to the hardware directly, so the web app (app.py) must NOT be
running at the same time (it would hold the serial ports).

Run it from anywhere (paths are resolved relative to this file):
    python phase0_test/Phase0test.py
    python phase0_test/Phase0test.py --valve-port /dev/ttyACM0 --rs485-port /dev/ttyCH9344USB0
    python phase0_test/Phase0test.py --cycles 20 --sample-hz 50 --out phase0_test/phase0_data
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time

# This script lives in phase0_test/ but drives the app's serial layers, which
# live one level up — put the project root on sys.path before importing them so
# it runs from any working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial.tools.list_ports

from valve_controller import ValveController
from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    EncoderController,
    is_ch9344_port,
    list_ch9344_ports,
)
from sensor_mapping import load_mapping


# ---- Test definition ------------------------------------------------------
SENSORS = (57, 58)          # RS-485 slave ids of the two muscles under test
PRE_INFLATE = 2000          # regulator setpoint held before zeroing
STATE_A = (2200, 1850)      # (R57, R58)
STATE_B = (1850, 2300)      # (R57, R58)
RAMP_S = 1.0                # linear-transition duration
PAUSE_S = 1              # dwell at each setpoint
PRE_PAUSE_S = 2.5           # dwell after pre-inflate, before zeroing
CYCLES = 10           # one cycle = A (ramp+pause) then B (ramp+pause)
SAMPLE_HZ = 50.0

CSV_COLUMNS = [
    "wall_time", "elapsed_s", "cycle", "segment",
    "target_R57", "target_R58", "cmd_R57_mV", "cmd_R58_mV",
    "pos_R57_mm", "pos_R58_mm", "online_R57", "online_R58",
]


# ---- Serial port detection (mirrors app.py, kept standalone) --------------

def _blob(p):
    return " ".join(str(x) for x in (
        p.description, getattr(p, "product", None),
        getattr(p, "manufacturer", None), p.hwid)).lower()


def find_ports(valve_port, rs485_port):
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


# ---- Recorder -------------------------------------------------------------

class Recorder:
    """Background sampler: appends one row per tick from the moment it starts."""

    def __init__(self, valve, encoder, valve_ids, sample_hz):
        self.valve = valve
        self.encoder = encoder
        self.v57, self.v58 = valve_ids
        self.period = 1.0 / sample_hz
        self.rows = []
        self._stop = threading.Event()
        self._thread = None
        self._ctx_lock = threading.Lock()
        self._ctx = {"cycle": 0, "segment": "zero",
                     "t57": PRE_INFLATE, "t58": PRE_INFLATE}
        self.t0 = None

    def set_context(self, cycle, segment, t57, t58):
        with self._ctx_lock:
            self._ctx = {"cycle": cycle, "segment": segment, "t57": t57, "t58": t58}

    def _cmd(self):
        with self.valve.display_lock:
            data = dict(self.valve.valve_data)
        return data.get(self.v57), data.get(self.v58)

    def _sensor(self):
        by = {e["slave"]: e for e in self.encoder.get_state().get("encoders", [])}
        e57, e58 = by.get(57, {}), by.get(58, {})
        return (e57.get("position_mm"), e58.get("position_mm"),
                e57.get("online"), e58.get("online"))

    def _loop(self):
        while not self._stop.is_set():
            start = time.perf_counter()
            with self._ctx_lock:
                c = dict(self._ctx)
            cmd57, cmd58 = self._cmd()
            p57, p58, on57, on58 = self._sensor()
            self.rows.append([
                round(time.time(), 3), round(start - self.t0, 4),
                c["cycle"], c["segment"], c["t57"], c["t58"],
                cmd57, cmd58, p57, p58, on57, on58,
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


# ---- Helpers --------------------------------------------------------------

def wait_online(encoder, sensors, timeout=10.0):
    """Block until every sensor id reports online, or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        by = {e["slave"]: e for e in encoder.get_state().get("encoders", [])}
        if all(by.get(s, {}).get("online") for s in sensors):
            return True
        time.sleep(0.2)
    return False


def resolve_valve_ids(mapping_path):
    """Return (valve_for_57, valve_for_58) from the sensor mapping."""
    mapping = load_mapping(mapping_path) if mapping_path else load_mapping()
    sensor_to_valve = {m["sensor"]: vid for vid, m in mapping.items()
                       if m.get("sensor") is not None}
    return sensor_to_valve.get(57), sensor_to_valve.get(58)


def save_csv(rows, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    fname = f"phase0test_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    path = os.path.join(out_dir, fname)
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)
    return path


# ---- Main -----------------------------------------------------------------

def run(args):
    v57, v58 = resolve_valve_ids(args.mapping)
    if v57 is None or v58 is None:
        print(f"[ERR] mapping is missing a regulator for sensor 57 or 58 "
              f"(got R57=valve {v57}, R58=valve {v58}). Check sensor_mapping.json.")
        return 2
    print(f"R57 = valve {v57} (sensor 57),  R58 = valve {v58} (sensor 58)")

    valve_port, rs485_port = find_ports(args.valve_port, args.rs485_port)
    if not valve_port or not rs485_port:
        print(f"[ERR] need both ports (valve={valve_port}, rs485={rs485_port}). "
              "Pass --valve-port / --rs485-port, and make sure app.py isn't running.")
        return 2
    print(f"Valve regulator : {valve_port}\nRS-485 sensors  : {rs485_port}")

    valve = ValveController(port=valve_port)
    if not valve.connected:
        print(f"[ERR] could not open valve regulator on {valve_port}")
        return 1

    encoder = EncoderController(
        port=rs485_port,
        slave_ids=list(SENSORS),
        counts_per_turn=args.counts_per_turn,
        drum_diameter_mm=args.drum_diameter,
        interval=0.01,      # poll the two sensors as fast as they answer
    )
    if not encoder.connected:
        print(f"[ERR] could not open RS-485 adapter on {rs485_port}")
        valve.close()
        return 1

    rec = Recorder(valve, encoder, (v57, v58), args.sample_hz)
    interrupted = False
    try:
        print(f"Waiting for sensors {SENSORS[0]} and {SENSORS[1]} to come online...")
        if not wait_online(encoder, SENSORS):
            print("[ERR] sensors 57/58 did not come online. Check wiring / slave ids.")
            return 1

        # 1) Pre-inflate, hold, then zero.
        print(f"Pre-inflating R57/R58 to {PRE_INFLATE}, holding {PRE_PAUSE_S:g}s...")
        valve.set_multiple_valves([(v57, PRE_INFLATE), (v58, PRE_INFLATE)], ramp=0.0)
        time.sleep(PRE_PAUSE_S)
        encoder.zero(57)
        encoder.zero(58)
        print("Zeroed sensors 57 and 58. Recording started.")

        # 2) Start recording, then oscillate.
        rec.start()
        for cycle in range(1, args.cycles + 1):
            for label, (t57, t58) in (("A", STATE_A), ("B", STATE_B)):
                rec.set_context(cycle, f"{label}_ramp", t57, t58)
                valve.set_multiple_valves([(v57, t57), (v58, t58)], ramp=RAMP_S)
                time.sleep(RAMP_S)
                rec.set_context(cycle, f"{label}_hold", t57, t58)
                time.sleep(PAUSE_S)
            print(f"  cycle {cycle}/{args.cycles} done "
                  f"({len(rec.rows)} samples)")

    except KeyboardInterrupt:
        interrupted = True
        print("\n[STOP] interrupted — emergency stop.")
    finally:
        rec.stop()
        try:
            if interrupted:
                valve.emergency_stop()
            else:
                # Normal completion: vent both regulators to 0.
                valve.set_multiple_valves([(v57, "off"), (v58, "off")], ramp=0.0)
        except Exception as exc:
            print(f"[WARN] could not vent valves: {exc}")
        time.sleep(0.2)

        path = save_csv(rec.rows, args.out)
        print(f"\nSaved {len(rec.rows)} samples to {os.path.abspath(path)}")

        valve.close()
        encoder.close()
    return 0


def build_parser():
    p = argparse.ArgumentParser(description="Phase 0 open-loop test for muscles R57/R58")
    p.add_argument("--valve-port", default=None, help="Giga R1 regulator port")
    p.add_argument("--rs485-port", default=None, help="USB-RS-485 adapter port")
    p.add_argument("--cycles", type=int, default=CYCLES, help="number of A-B cycles")
    p.add_argument("--sample-hz", type=float, default=SAMPLE_HZ, help="recording rate")
    p.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "phase0_data"),
                   help="output directory for the CSV")
    p.add_argument("--mapping", default=None, help="path to sensor_mapping.json")
    p.add_argument("--counts-per-turn", type=lambda v: int(v, 0),
                   default=DEFAULT_COUNTS_PER_TURN)
    p.add_argument("--drum-diameter", type=float, default=DEFAULT_DRUM_DIAMETER_MM)
    return p


def main(argv=None):
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
