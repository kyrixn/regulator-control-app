#!/usr/bin/env python3
"""
finger/config.py

Device-local hardware configuration for the physical index finger.

`finger.local.json` (git-ignored) records what the shared model and the
station repo cannot know on their own: which regulator and RS-485 sensor
carry each anatomical muscle role, the sign that turns encoder counts into
muscle contraction, the reference count captured at the maximum-extension
pose, and the per-muscle pressure ceiling. Copy `finger.local.example.json`
to `finger.local.json` and fill it in.

Two levels of validity:

  load()            structural: every role present, ids in range, no duplicate
                    regulator or sensor, consistent with sensor_mapping.json.
                    Enough for read-only tools (acquisition, calibration).
  require_armable() everything above plus a measured sign and reference for
                    every muscle. Anything that commands pressure from length
                    feedback must call this first and must not fall back to
                    defaults.

Sign convention follows the playground: contraction is positive when the
muscle shortens, negative when it pays out, in mm relative to the reference.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

ROLES = ("ED", "FDS", "FDP", "DI", "PI")
MAX_REGULATOR = 31
MAX_SLAVE = 247
# Station-wide command cap; mirrors valve_controller.MAX_INPUT_KPA (kept as a
# literal so this module has no pyserial dependency). Tested for equality.
APP_CAP_KPA = 140.0
CONFIG_VERSION = 1

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(HERE, "finger.local.json")
EXAMPLE_CONFIG_PATH = os.path.join(HERE, "finger.local.example.json")
DEFAULT_MAPPING_PATH = os.path.join(os.path.dirname(HERE), "sensor_mapping.json")


class ConfigError(ValueError):
    """The local config is missing, malformed, inconsistent or not armable."""


@dataclass
class MuscleConfig:
    role: str
    regulator: int
    sensor: int
    ceiling_kpa: float
    sign: Optional[int] = None            # +1 or -1 once measured
    reference_counts: Optional[int] = None  # absolute counts at max extension

    @property
    def calibrated(self) -> bool:
        return self.sign is not None and self.reference_counts is not None


@dataclass
class FingerConfig:
    valve_port: str
    rs485_port: str
    baud: int
    counts_per_turn: int
    drum_diameter_mm: float
    muscles: Dict[str, MuscleConfig] = field(default_factory=dict)
    path: str = ""

    @property
    def mm_per_count(self) -> float:
        return math.pi * self.drum_diameter_mm / self.counts_per_turn

    @property
    def sensors(self) -> List[int]:
        return [self.muscles[r].sensor for r in ROLES]

    @property
    def regulators(self) -> List[int]:
        return [self.muscles[r].regulator for r in ROLES]

    def role_of_sensor(self, sensor: int) -> Optional[str]:
        for r in ROLES:
            if self.muscles[r].sensor == sensor:
                return r
        return None

    def contraction_mm(self, role: str, counts: int) -> float:
        """Muscle contraction in mm (positive = shortening) from absolute counts."""
        m = self.muscles[role]
        if not m.calibrated:
            raise ConfigError(f"{role}: sign/reference not calibrated")
        return m.sign * (counts - m.reference_counts) * self.mm_per_count

    def arm_problems(self) -> List[str]:
        """Reasons this config must not drive pressure from length feedback."""
        out = []
        for r in ROLES:
            m = self.muscles[r]
            if m.sign is None:
                out.append(f"{r}: encoder sign not measured")
            if m.reference_counts is None:
                out.append(f"{r}: reference counts not captured")
        return out

    def require_armable(self) -> "FingerConfig":
        problems = self.arm_problems()
        if problems:
            raise ConfigError("not armable: " + "; ".join(problems))
        return self


# ----------------------------------------------------------------------------

def _int(d: dict, key: str, ctx: str, lo: int, hi: int) -> int:
    v = d.get(key)
    if isinstance(v, bool) or not isinstance(v, int):
        raise ConfigError(f"{ctx}.{key}: expected an integer, got {v!r}")
    if not lo <= v <= hi:
        raise ConfigError(f"{ctx}.{key}: {v} outside {lo}..{hi}")
    return v


def _opt_int(d: dict, key: str, ctx: str) -> Optional[int]:
    v = d.get(key)
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, int):
        raise ConfigError(f"{ctx}.{key}: expected an integer or null, got {v!r}")
    return v


def _number(d: dict, key: str, ctx: str, lo: float, hi: float) -> float:
    v = d.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ConfigError(f"{ctx}.{key}: expected a finite number, got {v!r}")
    if not lo <= v <= hi:
        raise ConfigError(f"{ctx}.{key}: {v} outside {lo}..{hi}")
    return float(v)


def _string(d: dict, key: str, ctx: str) -> str:
    v = d.get(key)
    if not isinstance(v, str) or not v.strip():
        raise ConfigError(f"{ctx}.{key}: expected a non-empty string, got {v!r}")
    return v


def parse(raw: dict, path: str = "") -> FingerConfig:
    """Validate a decoded JSON object into a FingerConfig (structural checks)."""
    if not isinstance(raw, dict):
        raise ConfigError("top level must be an object")
    if raw.get("version") != CONFIG_VERSION:
        raise ConfigError(f"version must be {CONFIG_VERSION}, got {raw.get('version')!r}")

    ports = raw.get("ports")
    if not isinstance(ports, dict):
        raise ConfigError("ports: expected an object")
    valve_port = _string(ports, "valve", "ports")
    rs485_port = _string(ports, "rs485", "ports")
    if valve_port == rs485_port:
        raise ConfigError("ports: valve and rs485 must differ")

    enc = raw.get("encoder")
    if not isinstance(enc, dict):
        raise ConfigError("encoder: expected an object")
    baud = _int(enc, "baud", "encoder", 1200, 1_000_000)
    cpt = _int(enc, "counts_per_turn", "encoder", 1, 1 << 32)
    drum = _number(enc, "drum_diameter_mm", "encoder", 0.1, 1000)

    muscles_raw = raw.get("muscles")
    if not isinstance(muscles_raw, dict):
        raise ConfigError("muscles: expected an object keyed by role")
    extra = sorted(set(muscles_raw) - set(ROLES))
    missing = [r for r in ROLES if r not in muscles_raw]
    if missing:
        raise ConfigError(f"muscles: missing roles {missing}")
    if extra:
        raise ConfigError(f"muscles: unknown roles {extra}")

    muscles: Dict[str, MuscleConfig] = {}
    for role in ROLES:
        d = muscles_raw[role]
        if not isinstance(d, dict):
            raise ConfigError(f"muscles.{role}: expected an object")
        ctx = f"muscles.{role}"
        sign = d.get("sign")
        if sign is not None and sign not in (1, -1):
            raise ConfigError(f"{ctx}.sign: expected 1, -1 or null, got {sign!r}")
        muscles[role] = MuscleConfig(
            role=role,
            regulator=_int(d, "regulator", ctx, 0, MAX_REGULATOR),
            sensor=_int(d, "sensor", ctx, 1, MAX_SLAVE),
            ceiling_kpa=_number(d, "ceiling_kpa", ctx, 0.0, APP_CAP_KPA),
            sign=sign,
            reference_counts=_opt_int(d, "reference_counts", ctx),
        )

    for key in ("regulator", "sensor"):
        seen: Dict[int, str] = {}
        for role in ROLES:
            v = getattr(muscles[role], key)
            if v in seen:
                raise ConfigError(f"duplicate {key} {v}: {seen[v]} and {role}")
            seen[v] = role

    return FingerConfig(valve_port=valve_port, rs485_port=rs485_port, baud=baud,
                        counts_per_turn=cpt, drum_diameter_mm=drum,
                        muscles=muscles, path=path)


def check_against_mapping(cfg: FingerConfig, mapping_path: str = DEFAULT_MAPPING_PATH) -> None:
    """Refuse silently diverging sources: sensor_mapping.json (used by app.py)
    must name the same regulator/sensor pair for every role."""
    try:
        with open(mapping_path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        raise ConfigError(f"sensor mapping not found: {mapping_path}")
    except json.JSONDecodeError as exc:
        raise ConfigError(f"sensor mapping is not valid JSON ({mapping_path}): {exc}")
    entries = raw.get("muscles") if isinstance(raw, dict) else None
    if not isinstance(entries, list):
        raise ConfigError(f"sensor mapping has no 'muscles' list: {mapping_path}")
    by_role = {}
    for e in entries:
        if isinstance(e, dict) and e.get("muscle") in ROLES:
            by_role[e["muscle"]] = e
    for role in ROLES:
        m = cfg.muscles[role]
        e = by_role.get(role)
        if e is None:
            raise ConfigError(f"{role}: not present in {os.path.basename(mapping_path)}")
        if e.get("regulator") != m.regulator or e.get("sensor") != m.sensor:
            raise ConfigError(
                f"{role}: local config says regulator {m.regulator}/sensor {m.sensor}, "
                f"{os.path.basename(mapping_path)} says "
                f"regulator {e.get('regulator')}/sensor {e.get('sensor')}")


def load(path: str = DEFAULT_CONFIG_PATH,
         mapping_path: Optional[str] = DEFAULT_MAPPING_PATH) -> FingerConfig:
    """Load and structurally validate the local config.

    Pass mapping_path=None to skip the sensor_mapping.json cross-check.
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        raise ConfigError(
            f"no local config at {path}; copy {os.path.basename(EXAMPLE_CONFIG_PATH)} "
            "to finger.local.json and fill it in")
    except json.JSONDecodeError as exc:
        raise ConfigError(f"{path} is not valid JSON: {exc}")
    cfg = parse(raw, path)
    if mapping_path is not None:
        check_against_mapping(cfg, mapping_path)
    return cfg


def save(cfg: FingerConfig, path: Optional[str] = None) -> None:
    """Write the config back (used by calibration tools to persist sign/reference)."""
    path = path or cfg.path or DEFAULT_CONFIG_PATH
    raw = {
        "version": CONFIG_VERSION,
        "ports": {"valve": cfg.valve_port, "rs485": cfg.rs485_port},
        "encoder": {"baud": cfg.baud, "counts_per_turn": cfg.counts_per_turn,
                    "drum_diameter_mm": cfg.drum_diameter_mm},
        "muscles": {
            r: {"regulator": m.regulator, "sensor": m.sensor,
                "ceiling_kpa": m.ceiling_kpa, "sign": m.sign,
                "reference_counts": m.reference_counts}
            for r, m in ((r, cfg.muscles[r]) for r in ROLES)
        },
    }
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(raw, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, path)


if __name__ == "__main__":
    import sys
    p = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG_PATH
    try:
        c = load(p)
    except ConfigError as exc:
        print(f"[ERR] {exc}")
        raise SystemExit(1)
    print(f"{c.path}: valve {c.valve_port}, rs485 {c.rs485_port} @ {c.baud}")
    print(f"{c.mm_per_count * 1e3:.5f} um/count ({c.drum_diameter_mm} mm drum)")
    for r in ROLES:
        m = c.muscles[r]
        print(f"  {r:3} V{m.regulator:<2} id{m.sensor:<3} cap {m.ceiling_kpa:g} kPa "
              f"sign {m.sign} ref {m.reference_counts}")
    problems = c.arm_problems()
    print("ARMABLE" if not problems else "NOT ARMABLE:\n  " + "\n  ".join(problems))
