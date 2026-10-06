#!/usr/bin/env python3
"""
station.py

Both devices of the station for a scripted experiment: the Giga's regulators
and the RS-485 length sensors, with every reading, command, ADC sample and
phase mark written to one long-format CSV (see Recorder).

Dry by default: sensors are read and logged, commands are logged with the note
"dry" and never sent. Only armed=True sends. Ramps and holds check the
experiment's sensors on every tick; a sensor that stops answering, an
exception or Ctrl-C sends the Giga's stop command, so every valve vents.

Run it from the experiments in experiments/; app.py must not be running,
since a COM port can be open in only one program.
"""

from __future__ import annotations

import csv
import math
import threading
import time
from typing import Dict, List, Optional, Sequence

from encoder_controller import EncoderReading, list_ch9344_ports
from sensor_hub import SensorHub
from valve_controller import ValveController, check_max_kpa, find_giga_port, mv_to_kpa


class Abort(RuntimeError):
    """The experiment stopped itself; the valves have been told to stop."""


class Recorder:
    """Thread-safe long-format CSV with columns t, kind, id, value, note.

    kind  len   id = sensor,    value = absolute counts (note = error if offline)
          cmd   id = regulator, value = kPa or "off"    (note = "dry" if not sent)
          adc   id = channel,   value = volts
          mark  note = phase label
    """

    FIELDS = ["t", "kind", "id", "value", "note"]

    def __init__(self, path: str):
        self.path = path
        self._fh = open(path, "w", newline="")
        self._w = csv.writer(self._fh)
        self._w.writerow(self.FIELDS)
        self._lock = threading.Lock()

    def row(self, t: float, kind: str, id_="", value="", note: str = "") -> None:
        with self._lock:
            if not self._fh.closed:
                self._w.writerow([f"{t:.4f}", kind, id_, value, note])

    def close(self) -> None:
        with self._lock:
            if not self._fh.closed:
                self._fh.close()


def health_problem(latest: Dict[int, EncoderReading], sensors: Sequence[int],
                   now: float, stale_s: float) -> Optional[str]:
    """Why the given sensors cannot be trusted right now, or None."""
    for s in sensors:
        r = latest.get(s)
        if r is None:
            return f"sensor {s}: no reading"
        if not r.online:
            return f"sensor {s}: offline ({r.message})"
        if now - r.timestamp > stale_s:
            return f"sensor {s}: no fresh reading for {now - r.timestamp:.2f} s"
    return None


class Station:
    """Regulators and sensors of the muscles an experiment uses.

    muscles: {name: {"regulator": int | None, "sensor": int | None}}. Only these
    regulators can be commanded, and only these sensors are health-checked.
    """

    def __init__(self, recorder: Recorder, muscles: Dict[str, dict], armed: bool = False,
                 max_kpa: float = 50.0, valve_port: Optional[str] = None,
                 rs485_ports: Optional[Sequence[str]] = None, adc_hz: float = 0.0,
                 stale_s: float = 0.5, hub=None, valves=None):
        self.recorder = recorder
        self.muscles = dict(muscles)
        self.armed = bool(armed)
        self.max_kpa = check_max_kpa(max_kpa)
        self.stale_s = stale_s
        self.clock = time.time
        self.sleep = time.sleep
        self.commanded: Dict[int, float] = {}          # regulator -> setpoint, without dither
        self.dither = None                             # (amp kPa, hz, {regulator: sign})
        self._dither_t0 = 0.0
        # Last *online* reading per sensor: a single missed read does not abort,
        # only stale_s without a good one does.
        self.latest: Dict[int, EncoderReading] = {}
        self.sensors = sorted({m["sensor"] for m in self.muscles.values()
                               if m.get("sensor") is not None})
        self.regulators = sorted({m["regulator"] for m in self.muscles.values()
                                  if m.get("regulator") is not None})
        self._adc_stop = threading.Event()
        self._adc_thread = None

        self.hub = hub if hub is not None else SensorHub(
            list(rs485_ports or list_ch9344_ports()), self.sensors, interval=0.0)
        self.hub.add_listener(self._on_reading)

        self.valves = valves
        if self.valves is None and (self.armed or adc_hz > 0):
            port = valve_port or find_giga_port()
            if port is None:
                raise RuntimeError("Giga not found; pass --valve-port")
            self.valves = ValveController(port=port, max_kpa=self.max_kpa)
        if self.valves is not None:
            self.valves.adc_listeners.append(self._on_adc)
            if adc_hz > 0:
                self._adc_thread = threading.Thread(target=self._adc_loop, args=(adc_hz,),
                                                    daemon=True)
                self._adc_thread.start()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _on_reading(self, r: EncoderReading) -> None:
        if r.online:
            self.recorder.row(r.timestamp, "len", r.slave, r.absolute_position)
            self.latest[r.slave] = r
        else:
            self.recorder.row(r.timestamp, "len", r.slave, "", r.message)

    def _on_adc(self, volts: Dict[int, float], t: float) -> None:
        for ch in self.regulators:
            if ch in volts:
                self.recorder.row(t, "adc", ch, volts[ch])

    def _adc_loop(self, hz: float) -> None:
        while not self._adc_stop.wait(1.0 / hz):
            self.valves.send("a")

    def mark(self, label: str) -> None:
        self.recorder.row(self.clock(), "mark", note=label)

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def wait_for_sensors(self, timeout: float = 10.0) -> List[int]:
        """Wait until every sensor has a fresh reading; return the ones missing."""
        end = time.monotonic() + timeout
        while True:
            now = self.clock()
            missing = [s for s in self.sensors
                       if s not in self.latest or now - self.latest[s].timestamp > self.stale_s]
            if not missing or time.monotonic() > end:
                return missing
            time.sleep(0.1)

    def counts(self) -> Dict[int, int]:
        return {s: r.absolute_position for s, r in self.latest.items()}

    def check(self) -> None:
        problem = health_problem(self.latest, self.sensors, self.clock(), self.stale_s)
        if problem:
            self.abort(problem)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def ping(self, timeout: float = 1.5) -> bool:
        """True if the Giga answers a ping. False if it is not responding or
        ignores serial (touchscreen standalone mode), where commands do nothing."""
        if self.valves is None:
            return False
        queue = self.valves.message_queue
        queue.clear()
        self.valves.send("p")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(str(m).startswith("PONG") for m in list(queue)):
                return True
            time.sleep(0.05)
        return False

    def sync_commanded(self) -> List[int]:
        """Read the Giga's setpoints into `commanded`, so ramps start from where
        the regulators really are. Returns other regulators that are on."""
        if self.valves is None:
            return []
        self.valves.query_status()
        self.sleep(0.5)
        others = []
        for reg, mV in dict(self.valves.valve_data).items():
            if reg in self.regulators:
                self.commanded[reg] = mv_to_kpa(mV)
            elif mV:
                others.append(reg)
        return sorted(others)

    def _resolve(self, targets: Dict[str, float]) -> Dict[int, float]:
        out = {}
        for name, kpa in targets.items():
            m = self.muscles.get(name)
            if m is None or m.get("regulator") is None:
                raise ValueError(f"{name}: not a regulated muscle of this experiment")
            kpa = float(kpa)
            if not 0.0 <= kpa <= self.max_kpa:
                raise ValueError(f"{name}: {kpa:g} kPa outside 0..{self.max_kpa:g}")
            out[m["regulator"]] = kpa
        return out

    # Dither: a sine superimposed on some muscles' setpoints. Opposite signs on
    # an antagonist pair keep its sum, so only the joint torque wobbles.
    DITHER_TICK = 0.05                 # stream at 20 Hz while dithering

    def set_dither(self, amp: float, hz: float, signs: Dict[str, int]) -> None:
        """From now on add sign * amp * sin(2 pi hz t) kPa to the named muscles."""
        regs = {self.muscles[n]["regulator"]: int(sg) for n, sg in signs.items()}
        self.dither = (float(amp), float(hz), regs)
        self._dither_t0 = self.clock()
        self.mark(f"dither {amp:g} kPa {hz:g} Hz on "
                  + ",".join(f"{'+' if sg > 0 else '-'}{n}" for n, sg in signs.items()))

    def clear_dither(self) -> None:
        """Stop dithering and send the plain setpoints again."""
        if self.dither is None:
            return
        regs = self.dither[2]
        self.dither = None
        self.mark("dither off")
        base = {r: self.commanded[r] for r in regs if r in self.commanded}
        if base:
            self._emit(base)

    def _with_dither(self, base: Dict[int, float], t: float) -> Dict[int, float]:
        if self.dither is None:
            return dict(base)
        amp, hz, regs = self.dither
        offset = amp * math.sin(2 * math.pi * hz * (t - self._dither_t0))
        return {r: k + regs.get(r, 0) * offset for r, k in base.items()}

    def _emit(self, base: Dict[int, float]) -> None:
        """Send setpoints (plus the dither, if on); log what was sent and, while
        dithering, the plain setpoints as "base" rows."""
        t = self.clock()
        sent = self._with_dither(base, t)
        for reg, kpa in sent.items():
            if not 0.0 <= kpa <= self.max_kpa:
                self.abort(f"V{reg}: {kpa:.1f} kPa (with dither) outside 0..{self.max_kpa:g}")
        if self.armed:
            if not self.valves.set_multiple_valves(sorted(sent.items())):
                queue = getattr(self.valves, "message_queue", None)
                detail = queue[-1] if queue else "no detail"
                self.abort(f"the valve layer refused a command ({detail})")
        note = "" if self.armed else "dry"
        for reg, kpa in sorted(sent.items()):
            self.recorder.row(t, "cmd", reg, round(kpa, 3), note)
        if self.dither is not None:
            for reg, kpa in sorted(base.items()):
                self.recorder.row(t, "base", reg, round(kpa, 3), note)
        self.commanded.update(base)

    def send(self, targets: Dict[str, float]) -> None:
        """Set muscles to kPa at once (validated against the cap)."""
        self._emit(self._resolve(targets))

    def ramp(self, targets: Dict[str, float], rate: float, tick: float = 0.2) -> None:
        """Move linearly from the commanded pressures to `targets`, the largest
        change at `rate` kPa/s, checking the sensors before every step."""
        if self.dither is not None:
            tick = min(tick, self.DITHER_TICK)
        goal = self._resolve(targets)
        start = {r: self.commanded.get(r, 0.0) for r in goal}
        delta = max((abs(goal[r] - start[r]) for r in goal), default=0.0)
        steps = max(1, math.ceil(delta / rate / tick)) if rate > 0 and tick > 0 else 1
        for k in range(1, steps + 1):
            frac = k / steps
            self.check()
            self._emit({r: start[r] + (goal[r] - start[r]) * frac for r in goal})
            self.sleep(tick)

    def hold(self, seconds: float, tick: float = 0.1) -> None:
        """Keep the setpoints for `seconds` (streaming the dither, if on)."""
        if self.dither is not None:
            tick = min(tick, self.DITHER_TICK)
        end = self.clock() + seconds
        while self.clock() < end:
            self.check()
            if self.dither is not None:
                regs = self.dither[2]
                self._emit({r: self.commanded[r] for r in regs if r in self.commanded})
            self.sleep(tick)

    def off(self) -> None:
        """Switch this experiment's valves off (0 V: vented)."""
        t = self.clock()
        if self.armed:
            self.valves.set_multiple_valves([(r, "off") for r in self.regulators])
        for reg in self.regulators:
            self.recorder.row(t, "cmd", reg, "off", "" if self.armed else "dry")
            self.commanded.pop(reg, None)

    def abort(self, reason: str) -> None:
        self.mark(f"ABORT: {reason}")
        if self.armed and self.valves is not None:
            self.valves.emergency_stop()
        self.commanded.clear()
        raise Abort(reason)

    # ------------------------------------------------------------------
    # Lifetime
    # ------------------------------------------------------------------

    def close(self, error: bool = False) -> None:
        self._adc_stop.set()
        if error and self.armed and self.valves is not None:
            self.mark("STOP: closed after an error")
            self.valves.emergency_stop()
        self.hub.close()
        if self.valves is not None:
            self.valves.close()

    def __enter__(self) -> "Station":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close(error=exc_type is not None)
        return False
