#!/usr/bin/env python3
"""
presets.py

The web app's command line and pressure presets.

Command line (one line, pressures in kPa, like the valve cells):
  v,kPa[,v,kPa,...]   set regulators; a value may be 'off'
  s                   STOP (every valve off; also ends a running preset)
  ? | p | a           the Giga's status, ping and ADC dump, sent as they are
  ps <name>           run preset <name> from the presets file

Presets file (JSON, re-read on every 'ps'): name -> a list of steps, or a
single step for a one-line preset. A step names only the regulators it
changes; the others keep what they had:

  {"base30": {"4": 30, "6": 30, "t": 5, "p": 0},
   "flex":   [{"4": 40, "6": 20, "t": 10, "p": 3},
              {"4": 30, "6": 30, "t": 10, "p": 0}]}

  "<regulator id>": kPa or "off"
  "t": ramp time, s (linear from the current pressures; 0 = instant)
  "p": pause after the ramp before the next step, s (0 = go straight on)

Every step is checked against the pressure cap before anything is sent.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Tuple, Union

from valve_controller import NUM_VALVES

Value = Union[float, str]          # kPa, or "off"
OFF_WORDS = ("off", "o", "x")
GIGA_PASSTHROUGH = ("?", "p", "a")


class CommandError(ValueError):
    """A command line or preset that cannot be run; nothing was sent."""


@dataclass
class Step:
    pairs: List[Tuple[int, Value]]
    t: float = 0.0                 # ramp time, s
    p: float = 0.0                 # pause afterwards, s


def _regulator(raw) -> int:
    try:
        reg = int(str(raw).strip())
    except ValueError:
        raise CommandError(f"'{raw}' is not a regulator id") from None
    if not 0 <= reg < NUM_VALVES:
        raise CommandError(f"regulator {reg} out of range 0..{NUM_VALVES - 1}")
    return reg


def _pressure(raw, reg: int, max_kpa: float) -> Value:
    if isinstance(raw, str) and raw.strip().lower() in OFF_WORDS:
        return "off"
    if isinstance(raw, bool):
        raise CommandError(f"V{reg}: invalid value {raw!r}")
    try:
        kpa = float(raw)
    except (TypeError, ValueError):
        raise CommandError(f"V{reg}: invalid value {raw!r}") from None
    if not 0.0 <= kpa <= max_kpa:
        raise CommandError(f"V{reg}: {kpa:g} kPa outside 0..{max_kpa:g} kPa")
    return kpa


def _seconds(raw, key: str) -> float:
    if isinstance(raw, bool):
        raise CommandError(f"'{key}' must be seconds >= 0, got {raw!r}")
    try:
        s = float(raw)
    except (TypeError, ValueError):
        raise CommandError(f"'{key}' must be seconds >= 0, got {raw!r}") from None
    if s < 0:
        raise CommandError(f"'{key}' must be seconds >= 0, got {raw!r}")
    return s


def parse_command(text: str, max_kpa: float):
    """One typed line -> ("stop",) | ("raw", cmd) | ("set", pairs) | ("preset", name)."""
    line = text.strip()
    low = line.lower()
    if low == "s":
        return ("stop",)
    if low in GIGA_PASSTHROUGH:
        return ("raw", low)
    if low == "ps" or low.startswith("ps "):
        name = line[2:].strip()
        if not name:
            raise CommandError("ps needs a preset name: ps <name>")
        return ("preset", name)
    if not line:
        raise CommandError("empty command")
    tokens = [tok.strip() for tok in line.split(",")]
    if len(tokens) % 2:
        raise CommandError(f"'{line}': expected v,kPa pairs (or s, ?, p, a, ps <name>)")
    pairs = []
    for i in range(0, len(tokens), 2):
        reg = _regulator(tokens[i])
        pairs.append((reg, _pressure(tokens[i + 1], reg, max_kpa)))
    return ("set", pairs)


def parse_step(obj, max_kpa: float) -> Step:
    if not isinstance(obj, dict):
        raise CommandError(f"a step must be an object, got {obj!r}")
    step = Step([])
    for key, raw in obj.items():
        if key in ("t", "p"):
            setattr(step, key, _seconds(raw, key))
        else:
            reg = _regulator(key)
            step.pairs.append((reg, _pressure(raw, reg, max_kpa)))
    return step


def parse_preset(obj, max_kpa: float) -> List[Step]:
    """A preset (a list of steps, or one step) -> steps; raises on any bad step."""
    steps = [obj] if isinstance(obj, dict) else obj
    if not isinstance(steps, list) or not steps:
        raise CommandError("a preset must be a step or a non-empty list of steps")
    out = []
    for i, s in enumerate(steps, 1):
        try:
            out.append(parse_step(s, max_kpa))
        except CommandError as exc:
            raise CommandError(f"step {i}: {exc}") from None
    return out


def load_presets(path: str) -> dict:
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        raise CommandError(f"no presets file at {path}") from None
    except (OSError, ValueError) as exc:
        raise CommandError(f"cannot read {path}: {exc}") from None
    if not isinstance(data, dict):
        raise CommandError(f"{path}: expected an object of name -> preset")
    return data


def describe(step: Step) -> str:
    setp = ", ".join(f"V{r}={'off' if v == 'off' else f'{v:g}'}" for r, v in step.pairs)
    return f"{setp or 'no change'}, t {step.t:g} s, p {step.p:g} s"


class PresetRunner:
    """Runs one preset at a time on its own thread.

    stop() and sending a step share a lock, so once stop() returns no further
    step reaches the valves (the caller then sends the emergency stop).
    """

    def __init__(self, valves, log: Callable[[str], None]):
        self._valves = valves
        self._log = log
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._name = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, name: str, steps: List[Step]) -> bool:
        if self.running:
            self._log(f"[ERR] preset {self._name} is still running; STOP it first")
            return False
        self._name = name
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(name, steps), daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        with self._lock:
            self._stop.set()

    def _send(self, step: Step) -> bool:
        with self._lock:
            if self._stop.is_set():
                return False
            return bool(self._valves.set_multiple_valves(step.pairs, ramp=step.t))

    def _ramps_pending(self, regs) -> bool:
        with self._valves.ramp_lock:
            return any(r in self._valves.ramps for r in regs)

    def _run(self, name: str, steps: List[Step]) -> None:
        n = len(steps)
        for i, step in enumerate(steps, 1):
            if self._stop.is_set():
                break
            self._log(f"PRESET {name} {i}/{n}: {describe(step)}")
            if step.pairs and not self._send(step):
                if not self._stop.is_set():
                    self._log(f"[ERR] preset {name} aborted at step {i}: send failed")
                    return
                break
            # Wait out the ramp, then until its last setpoint has gone out.
            if self._stop.wait(step.t):
                break
            regs = [r for r, _ in step.pairs]
            end = time.monotonic() + 1.0
            while self._ramps_pending(regs) and time.monotonic() < end:
                if self._stop.wait(0.01):
                    break
            if self._stop.wait(step.p):
                break
        if self._stop.is_set():
            self._log(f"preset {name} stopped")
        else:
            self._log(f"preset {name} done")
