#!/usr/bin/env python3
"""
sensor_mapping.py

Loads the sensor↔regulator mapping used by the closed-loop groundwork. Each
*muscle* (tendon) is driven by one pneumatic *regulator* (valve id 0-31) and its
length is measured by one draw-wire *sensor* (RS-485 encoder slave id). This
module turns ``sensor_mapping.json`` into a lookup keyed by regulator (valve id)
so the web layer can attach each valve's mapped sensor position to its bar.

The loader is deliberately tolerant: a missing or malformed file logs a warning
and yields an empty mapping so the app still runs (every bar simply shows no
length).
"""

from __future__ import annotations

import json
import os
from typing import Dict, Optional, TypedDict


DEFAULT_MAPPING_PATH = os.path.join(os.path.dirname(__file__), "sensor_mapping.json")

# Highest regulator (valve id) the station has: 32 valves, 0-31, matching
# valve_controller.NUM_VALVES. Kept as a literal so this module stays free of
# the pyserial-dependent valve layer.
MAX_REGULATOR = 31


class MuscleMap(TypedDict):
    muscle: str
    sensor: Optional[int]


def load_mapping(path: str = DEFAULT_MAPPING_PATH) -> Dict[int, MuscleMap]:
    """Return ``{valve_id: {"muscle": str, "sensor": int | None}}``.

    Tolerant of a missing/malformed file: logs a warning and returns ``{}``.
    Entries with an out-of-range or non-integer regulator are skipped.
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        print(f"[WARN] sensor mapping not found at {path}; regulators unmapped")
        return {}
    except (json.JSONDecodeError, OSError) as exc:
        print(f"[WARN] could not read sensor mapping {path}: {exc}; regulators unmapped")
        return {}

    muscles = raw.get("muscles") if isinstance(raw, dict) else None
    if not isinstance(muscles, list):
        print(f"[WARN] sensor mapping {path} has no 'muscles' list; regulators unmapped")
        return {}

    mapping: Dict[int, MuscleMap] = {}
    for entry in muscles:
        if not isinstance(entry, dict):
            continue
        try:
            regulator = int(entry["regulator"])
        except (KeyError, ValueError, TypeError):
            continue
        if not 0 <= regulator <= MAX_REGULATOR:
            continue

        sensor = entry.get("sensor")
        if sensor is not None:
            try:
                sensor = int(sensor)
            except (ValueError, TypeError):
                sensor = None

        mapping[regulator] = {
            "muscle": str(entry.get("muscle", f"muscle_{regulator:02d}")),
            "sensor": sensor,
        }

    return mapping
