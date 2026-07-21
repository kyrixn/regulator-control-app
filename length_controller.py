#!/usr/bin/env python3
"""
length_controller.py

Simple, conservative closed-loop LENGTH control for the PAM muscles.

Each muscle is identified by its draw-wire SENSOR id (the sensor is attached to
the muscle; the regulator/valve that drives it may be re-wired, so targets are
keyed by sensor, and the current valve is looked up live from the mapping).

Control law: a conservative PI (Kd available but 0 by default), tuned to remove
steady-state error even if the response is slow — low response time is fine, a
standing offset is not. The command is a regulator pressure in mV:

    e   = sign * (target_mm - length_mm)      # sign encodes the plant direction
    I  += Ki * e * dt                          # integrator holds the steady output
    out = clamp(Kp*e + I + Kd*de,  0 .. max_mv)
    I  += (out - raw)                          # back-calculation anti-windup

`sign = -1` means "more pressure shortens the muscle" (pressure up -> length
down), so to lengthen we lower pressure. On the first tick the integrator is
preloaded so `out` equals the muscle's current pressure — a bumpless takeover.

Within `tolerance_mm` (default 0.1 mm) of the target the loop enters a deadband:
it freezes the integrator and stops re-commanding, so it does not hunt for
precision finer than the muscle's few-mm working range needs (this prevents a
slow 1-mV limit cycle / valve chatter around the setpoint).

Output is hard-clamped to `max_mv` (default 3000) so a mis-set gain or wrong
sign can only drive to a safe rail, never past the muscle pressure limit.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional


DEFAULT_KP = 25.0      # mV per mm of error
DEFAULT_KI = 10.0      # mV per (mm * second)
DEFAULT_KD = 0.0       # off by default (draw-wire noise makes D touchy)
DEFAULT_SIGN = -1      # pressure up -> length down
DEFAULT_TOLERANCE_MM = 0.1  # deadband: within this of target, stop trimming
MUSCLE_MAX_MV = 3000   # hard pressure ceiling for muscles
MUSCLE_MIN_MV = 0
DEFAULT_RATE_HZ = 20.0


@dataclass
class _PID:
    target_mm: float
    integ: float = 0.0
    prev_err: float = 0.0
    prev_t: float = 0.0
    output: float = 0.0
    error_mm: float = 0.0
    started: bool = False
    at_target: bool = False


class LengthController:
    """Background PI loop driving muscle length via regulator pressure."""

    def __init__(
        self,
        valve_controller,
        encoder_controller,
        sensor_to_reg: Callable[[], Dict[int, int]],
        kp: float = DEFAULT_KP,
        ki: float = DEFAULT_KI,
        kd: float = DEFAULT_KD,
        sign: int = DEFAULT_SIGN,
        tolerance_mm: float = DEFAULT_TOLERANCE_MM,
        max_mv: int = MUSCLE_MAX_MV,
        min_mv: int = MUSCLE_MIN_MV,
        rate_hz: float = DEFAULT_RATE_HZ,
        message_queue=None,
    ):
        self.valve = valve_controller
        self.encoder = encoder_controller
        self.sensor_to_reg = sensor_to_reg  # live {sensor_id: valve_id}
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.sign = -1 if sign < 0 else 1
        self.tolerance_mm = float(tolerance_mm)
        self.max_mv, self.min_mv = int(max_mv), int(min_mv)
        self.period = 1.0 / rate_hz
        self.msgs = message_queue

        self.lock = threading.Lock()
        self.pids: Dict[int, _PID] = {}     # keyed by SENSOR id
        self.last_sent: Dict[int, int] = {}
        self.running = False
        self.thread = None

    # ------------------------------------------------------------------
    def _log(self, m):
        if self.msgs is not None:
            self.msgs.append(m)

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)

    # ------------------------------------------------------------------
    # Commands (all keyed by sensor id)
    # ------------------------------------------------------------------
    def set_gains(self, kp=None, ki=None, kd=None):
        with self.lock:
            if kp is not None:
                self.kp = float(kp)
            if ki is not None:
                self.ki = float(ki)
            if kd is not None:
                self.kd = float(kd)
        self._log(f"[LEN] gains Kp={self.kp:g} Ki={self.ki:g} Kd={self.kd:g}")

    def set_length(self, sensor: int, target_mm: float):
        sensor = int(sensor)
        target = float(target_mm)
        with self.lock:
            pid = self.pids.get(sensor)
            if pid is None:
                self.pids[sensor] = _PID(target_mm=target)
            else:
                pid.target_mm = target
                pid.started = False  # re-preload integrator for a bumpless move
        reg = self.sensor_to_reg().get(sensor)
        self._log(f"[LEN] sensor {sensor} -> {target:.2f} mm (reg {reg})")

    def clear(self, sensor: int):
        sensor = int(sensor)
        with self.lock:
            existed = self.pids.pop(sensor, None) is not None
            self.last_sent.pop(sensor, None)
        if existed:
            self._log(f"[LEN] sensor {sensor} length control off")

    def clear_all(self):
        with self.lock:
            n = len(self.pids)
            self.pids.clear()
            self.last_sent.clear()
        if n:
            self._log(f"[LEN] HOLD — {n} length controller(s) off, pressure held")

    def is_controlled(self, sensor: int) -> bool:
        with self.lock:
            return int(sensor) in self.pids

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _loop(self):
        while self.running:
            t0 = time.time()
            try:
                self._tick()
            except Exception as exc:  # never let the loop die
                self._log(f"[LEN][ERR] {exc}")
            rest = self.period - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)

    def _tick(self):
        with self.lock:
            if not self.pids:
                return
            sensors = list(self.pids.keys())
            kp, ki, kd, sign = self.kp, self.ki, self.kd, self.sign

        enc = self.encoder.get_state() if self.encoder else {}
        lengths = {e["slave"]: (e.get("position_mm"), e.get("online"))
                   for e in enc.get("encoders", [])}
        s2r = self.sensor_to_reg()
        now = time.time()

        for sensor in sensors:
            reg = s2r.get(sensor)
            if reg is None:
                continue  # sensor not currently mapped to a valve
            mm, online = lengths.get(sensor, (None, False))
            if mm is None or not online:
                continue  # no reading — hold last command, don't integrate

            with self.valve.display_lock:
                cur_p = self.valve.valve_data.get(reg, 0)

            command = None
            with self.lock:
                pid = self.pids.get(sensor)
                if pid is None:
                    continue
                phys_err = pid.target_mm - mm
                e = sign * phys_err
                if not pid.started:
                    # Preload so out == current pressure on the first tick.
                    pid.integ = cur_p - kp * e
                    pid.prev_t = now - self.period
                    pid.prev_err = e
                    pid.started = True
                dt = now - pid.prev_t
                if dt <= 0:
                    dt = self.period
                pid.error_mm = phys_err
                if abs(phys_err) <= self.tolerance_mm:
                    # Within tolerance: hold the current pressure, freeze the
                    # integrator, and stop commanding (no chase below tolerance).
                    pid.at_target = True
                    pid.prev_err = e
                    pid.prev_t = now
                else:
                    integ = pid.integ + ki * e * dt
                    de = (e - pid.prev_err) / dt
                    raw = kp * e + integ + kd * de
                    out = min(self.max_mv, max(self.min_mv, raw))
                    integ += (out - raw)  # back-calculation anti-windup
                    pid.integ = integ
                    pid.prev_err = e
                    pid.prev_t = now
                    pid.output = out
                    pid.at_target = False
                    command = int(round(out))

            if command is not None and self.last_sent.get(sensor) != command:
                self.valve.set_valve(reg, command, ramp=0.0)
                self.last_sent[sensor] = command

    # ------------------------------------------------------------------
    def get_state(self) -> dict:
        with self.lock:
            controllers = {
                sensor: {
                    "target_mm": round(pid.target_mm, 2),
                    "output_mv": int(round(pid.output)),
                    "error_mm": round(pid.error_mm, 3),
                    "at_target": pid.at_target,
                }
                for sensor, pid in self.pids.items()
            }
            return {
                "available": True,
                "gains": {"kp": self.kp, "ki": self.ki, "kd": self.kd},
                "sign": self.sign,
                "tolerance_mm": self.tolerance_mm,
                "max_mv": self.max_mv,
                "controllers": controllers,
            }
