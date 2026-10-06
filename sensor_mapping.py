#!/usr/bin/env python3
"""
sensor_mapping.py

Loads the muscle / regulator / sensor mapping. Each *muscle* (tendon) is driven
by one pneumatic *regulator* (valve id 0-31) and its length is measured by one
draw-wire *sensor* (RS-485 encoder slave id). Either side may be missing while
the hand is being wired: ``"regulator": null`` is a sensor with no regulator
yet, ``"sensor": null`` a regulator whose sensor is not wired.

An optional ``"region"`` ("forearm" or "palm") places the muscle; the web app
uses it to pick the length-bar band.

Two views of ``sensor_mapping.json``:

  load_mapping()  {valve_id: {"muscle", "sensor", "region"}}  -- the regulator grid
  load_all()      that, plus {sensor_id: {"muscle", "regulator", "region"}} for
                  the all-sensors strip, which also names sensor-only muscles

The loader is deliberately tolerant: a missing or malformed file logs a warning
and yields empty maps so the app still runs.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Tuple, TypedDict


DEFAULT_MAPPING_PATH = os.path.join(os.path.dirname(__file__), "sensor_mapping.json")

# Highest regulator (valve id) the station has: 32 valves, 0-31, matching
# valve_controller.NUM_VALVES. Kept as a literal so this module stays free of
# the pyserial-dependent valve layer.
MAX_REGULATOR = 31
MAX_SLAVE = 247      # highest Modbus slave id


class MuscleMap(TypedDict):
    muscle: str
    sensor: Optional[int]
    region: Optional[str]


class SensorEntry(TypedDict):
    muscle: str
    regulator: Optional[int]
    region: Optional[str]


def _read_entries(path: str) -> List[dict]:
    """The file's 'muscles' list, or [] with a warning if it is unusable."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        print(f"[WARN] sensor mapping not found at {path}; regulators unmapped")
        return []
    except (json.JSONDecodeError, OSError) as exc:
        print(f"[WARN] could not read sensor mapping {path}: {exc}; regulators unmapped")
        return []

    muscles = raw.get("muscles") if isinstance(raw, dict) else None
    if not isinstance(muscles, list):
        print(f"[WARN] sensor mapping {path} has no 'muscles' list; regulators unmapped")
        return []
    return [e for e in muscles if isinstance(e, dict)]


def _as_int(value) -> Optional[int]:
    try:
        return int(value)
    except (ValueError, TypeError):
        return None


def _region(entry: dict) -> Optional[str]:
    value = entry.get("region")
    return value if isinstance(value, str) else None


def _by_regulator(entries: List[dict]) -> Dict[int, MuscleMap]:
    mapping: Dict[int, MuscleMap] = {}
    for entry in entries:
        regulator = _as_int(entry.get("regulator"))
        if regulator is None or not 0 <= regulator <= MAX_REGULATOR:
            continue
        mapping[regulator] = {
            "muscle": str(entry.get("muscle", f"muscle_{regulator:02d}")),
            "sensor": _as_int(entry.get("sensor")),
            "region": _region(entry),
        }
    return mapping


def _by_sensor(entries: List[dict]) -> Dict[int, SensorEntry]:
    index: Dict[int, SensorEntry] = {}
    for entry in entries:
        sensor = _as_int(entry.get("sensor"))
        if sensor is None or not 1 <= sensor <= MAX_SLAVE:
            continue
        if sensor in index:
            print(f"[WARN] sensor {sensor} is listed twice in the mapping; "
                  f"keeping {index[sensor]['muscle']}")
            continue
        regulator = _as_int(entry.get("regulator"))
        if regulator is not None and not 0 <= regulator <= MAX_REGULATOR:
            regulator = None
        index[sensor] = {
            "muscle": str(entry.get("muscle", f"sensor_{sensor}")),
            "regulator": regulator,
            "region": _region(entry),
        }
    return index


def load_mapping(path: str = DEFAULT_MAPPING_PATH) -> Dict[int, MuscleMap]:
    """Return ``{valve_id: {"muscle": str, "sensor": int | None, "region": str | None}}``.

    Entries with a missing, out-of-range or non-integer regulator are skipped.
    """
    return _by_regulator(_read_entries(path))


def muscles_by_name(path: str = DEFAULT_MAPPING_PATH) -> Dict[str, dict]:
    """``{muscle: {"regulator", "sensor", "region"}}`` for every named entry,
    including sensor-only and regulator-only ones (used by the experiments)."""
    out: Dict[str, dict] = {}
    for entry in _read_entries(path):
        name = entry.get("muscle")
        if not isinstance(name, str) or name in out:
            continue
        regulator = _as_int(entry.get("regulator"))
        sensor = _as_int(entry.get("sensor"))
        out[name] = {
            "regulator": regulator if regulator is not None and 0 <= regulator <= MAX_REGULATOR
            else None,
            "sensor": sensor if sensor is not None and 1 <= sensor <= MAX_SLAVE else None,
            "region": _region(entry),
        }
    return out


def load_all(path: str = DEFAULT_MAPPING_PATH
             ) -> Tuple[Dict[int, MuscleMap], Dict[int, SensorEntry]]:
    """Return ``(by_regulator, by_sensor)`` from one read of the file.

    ``by_sensor`` is ``{sensor_id: {"muscle": str, "regulator": int | None,
    "region": str | None}}`` and includes sensors that have no regulator yet.
    """
    entries = _read_entries(path)
    return _by_regulator(entries), _by_sensor(entries)
