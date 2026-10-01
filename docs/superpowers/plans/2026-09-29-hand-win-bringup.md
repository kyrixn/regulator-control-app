# hand_win Bring-up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the web app and a read-only scan tool read all 32 draw-wire sensors across several EKU081 ports on Windows, with the full-hand mapping and a session pressure cap.

**Architecture:** One `EncoderController` (one poll thread) per EKU081 channel, merged by a new `SensorHub` into one table keyed by sensor id. Port discovery matches the WCH COM ports by name, because the Windows driver hides the VID. The mapping file gains sensor-only and regulator-only entries, and the app scans exactly the mapped ids by default.

**Tech Stack:** Python 3.13, pyserial 3.5, Flask 3.1 + Flask-SocketIO 5.6, stdlib `unittest`, vanilla JS.

Spec: `docs/superpowers/specs/2026-09-29-hand-win-system-bringup-design.md`.

## Global Constraints

- Interpreter: `C:\Users\14747\.conda\envs\py313\python.exe` (in Git Bash: `PY=/c/Users/14747/.conda/envs/py313/python.exe`). No new dependencies beyond `requirements.txt`.
- Windows and Linux both work. Linux CH9344 nodes `/dev/ttyCH9344USB*` stay supported.
- Windows EKU081 ports: `WCH USB-SERIAL Ch A (COM7)` … `Ch H (COM14)`, manufacturer `wch.cn`, hwid `USB VID:PID=0000:0000`.
- `valve_controller.MAX_INPUT_KPA` stays `140.0` (`finger/config.py` tests equality).
- Sensor ids are unique across ports; Modbus slave range 1–247; regulators 0–31.
- Tests and `scan_sensors.py` never open the Giga and never send a valve command.
- No git commits or pushes unless the user asks (user rule). Each task ends with a test run instead.
- Match the surrounding code: module docstrings, short comments that explain why, type hints as in `encoder_controller.py`.

Test commands, from the repo root:
- New suite: `$PY -m unittest discover -s tests -t .`
- Finger suite: `$PY -m unittest discover -s finger -p "test_*.py" -t .`

---

### Task 1: Full-hand mapping and sensor-keyed index

**Files:**
- Modify: `sensor_mapping.py` (whole file)
- Modify: `sensor_mapping.json` (whole file)
- Modify: `finger/test_config.py:47` (skip one test)
- Create: `tests/__init__.py`, `tests/test_sensor_mapping.py`

**Interfaces:**
- Produces: `sensor_mapping.load_all(path) -> Tuple[Dict[int, MuscleMap], Dict[int, SensorEntry]]`, where `MuscleMap = {"muscle": str, "sensor": Optional[int]}` keyed by regulator and `SensorEntry = {"muscle": str, "regulator": Optional[int]}` keyed by sensor id. `load_mapping(path)` is unchanged (regulator-keyed). `MAX_SLAVE = 247`.

- [ ] **Step 1: Write the failing test** — `tests/__init__.py` (empty) and `tests/test_sensor_mapping.py`:

```python
"""Unit tests for sensor_mapping.py and the station's sensor_mapping.json."""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sensor_mapping as M  # noqa: E402

ENTRIES = [
    {"muscle": "FDP1", "regulator": 8, "sensor": 50},
    {"muscle": "FDP4", "regulator": None, "sensor": 53},
    {"muscle": "FPL", "regulator": 11, "sensor": None, "note": "sensor 58 not connected yet"},
    {"muscle": "brachialis", "regulator": 31, "sensor": 175},
    {"muscle": "dup", "regulator": 5, "sensor": 50},
    {"muscle": "bad", "regulator": 40, "sensor": 300},
]


def write(d, entries):
    p = os.path.join(d, "sensor_mapping.json")
    with open(p, "w", encoding="utf-8") as fh:
        json.dump({"muscles": entries}, fh)
    return p


class LoadAllTests(unittest.TestCase):
    def test_by_regulator_skips_sensor_only_and_out_of_range(self):
        with tempfile.TemporaryDirectory() as d:
            by_reg, _ = M.load_all(write(d, ENTRIES))
        self.assertEqual(by_reg, {
            8: {"muscle": "FDP1", "sensor": 50},
            11: {"muscle": "FPL", "sensor": None},
            31: {"muscle": "brachialis", "sensor": 175},
            5: {"muscle": "dup", "sensor": 50},
        })

    def test_by_sensor_keeps_sensor_only_entries_and_first_duplicate(self):
        with tempfile.TemporaryDirectory() as d:
            _, by_sensor = M.load_all(write(d, ENTRIES))
        self.assertEqual(by_sensor, {
            50: {"muscle": "FDP1", "regulator": 8},
            53: {"muscle": "FDP4", "regulator": None},
            175: {"muscle": "brachialis", "regulator": 31},
        })

    def test_load_mapping_matches_load_all(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, ENTRIES)
            self.assertEqual(M.load_mapping(p), M.load_all(p)[0])

    def test_missing_file_is_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(M.load_all(os.path.join(d, "none.json")), ({}, {}))


class StationMappingTests(unittest.TestCase):
    """Pins the transcription of bionic_hand/muscle_id.xlsx (2026-09-29)."""

    def setUp(self):
        self.by_reg, self.by_sensor = M.load_all(M.DEFAULT_MAPPING_PATH)

    def test_counts(self):
        self.assertEqual(len(self.by_sensor), 32)
        self.assertEqual(len(self.by_reg), 28)

    def test_sensor_ids(self):
        expected = (set(range(50, 58)) | {60} | set(range(61, 71)) | set(range(75, 80))
                    | set(range(81, 85)) | {86, 173, 174, 175})
        self.assertEqual(set(self.by_sensor), expected)

    def test_unused_regulators(self):
        self.assertEqual(set(range(32)) - set(self.by_reg), {0, 7, 19, 28})

    def test_spot_checks(self):
        self.assertEqual(self.by_reg[8], {"muscle": "FDP1", "sensor": 50})
        self.assertEqual(self.by_reg[11], {"muscle": "FPL", "sensor": None})
        self.assertEqual(self.by_sensor[53], {"muscle": "FDP4", "regulator": None})
        self.assertEqual(self.by_sensor[175], {"muscle": "brachialis", "regulator": 31})


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_sensor_mapping -v`
Expected: errors with `AttributeError: module 'sensor_mapping' has no attribute 'load_all'`.

- [ ] **Step 3: Implement** — replace `sensor_mapping.py` with:

```python
#!/usr/bin/env python3
"""
sensor_mapping.py

Loads the muscle / regulator / sensor mapping. Each *muscle* (tendon) is driven
by one pneumatic *regulator* (valve id 0-31) and its length is measured by one
draw-wire *sensor* (RS-485 encoder slave id). Either side may be missing while
the hand is being wired: ``"regulator": null`` is a sensor with no regulator
yet, ``"sensor": null`` a regulator whose sensor is not wired.

Two views of ``sensor_mapping.json``:

  load_mapping()  {valve_id: {"muscle", "sensor"}}  -- the regulator grid
  load_all()      that, plus {sensor_id: {"muscle", "regulator"}} for the
                  all-sensors strip, which also names sensor-only muscles

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


class SensorEntry(TypedDict):
    muscle: str
    regulator: Optional[int]


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


def _by_regulator(entries: List[dict]) -> Dict[int, MuscleMap]:
    mapping: Dict[int, MuscleMap] = {}
    for entry in entries:
        regulator = _as_int(entry.get("regulator"))
        if regulator is None or not 0 <= regulator <= MAX_REGULATOR:
            continue
        mapping[regulator] = {
            "muscle": str(entry.get("muscle", f"muscle_{regulator:02d}")),
            "sensor": _as_int(entry.get("sensor")),
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
        }
    return index


def load_mapping(path: str = DEFAULT_MAPPING_PATH) -> Dict[int, MuscleMap]:
    """Return ``{valve_id: {"muscle": str, "sensor": int | None}}``.

    Entries with a missing, out-of-range or non-integer regulator are skipped.
    """
    return _by_regulator(_read_entries(path))


def load_all(path: str = DEFAULT_MAPPING_PATH
             ) -> Tuple[Dict[int, MuscleMap], Dict[int, SensorEntry]]:
    """Return ``(by_regulator, by_sensor)`` from one read of the file.

    ``by_sensor`` is ``{sensor_id: {"muscle": str, "regulator": int | None}}``
    and includes sensors that have no regulator yet.
    """
    entries = _read_entries(path)
    return _by_regulator(entries), _by_sensor(entries)
```

Replace `sensor_mapping.json` with:

```json
{
  "_comment": "Muscle -> regulator (valve id 0-31) and length sensor (RS-485 slave id). regulator null = sensor wired, no regulator yet; sensor null = regulator wired, sensor not yet (planned id in note). Transcribed from bionic_hand/muscle_id.xlsx on 2026-09-29. Edits take effect on the next RESCAN.",
  "muscles": [
    { "muscle": "FDP1", "regulator": 8,    "sensor": 50 },
    { "muscle": "FDP2", "regulator": 9,    "sensor": 51 },
    { "muscle": "FDP3", "regulator": 10,   "sensor": 52 },
    { "muscle": "FDP4", "regulator": null, "sensor": 53 },
    { "muscle": "FDS1", "regulator": 12,   "sensor": 54 },
    { "muscle": "FDS2", "regulator": 13,   "sensor": 55 },
    { "muscle": "FDS3", "regulator": 14,   "sensor": 56 },
    { "muscle": "FDS4", "regulator": null, "sensor": 57 },
    { "muscle": "FPL",  "regulator": 11,   "sensor": null, "note": "sensor 58 not connected yet" },
    { "muscle": "EPL",  "regulator": 15,   "sensor": 60 },
    { "muscle": "ED1",  "regulator": 16,   "sensor": 61 },
    { "muscle": "ED2",  "regulator": 17,   "sensor": 62 },
    { "muscle": "ED3",  "regulator": 18,   "sensor": 63 },
    { "muscle": "ED4",  "regulator": null, "sensor": 64 },
    { "muscle": "PT",   "regulator": 1,    "sensor": 65 },
    { "muscle": "Sup",  "regulator": 2,    "sensor": 66 },
    { "muscle": "FCU",  "regulator": 3,    "sensor": 67 },
    { "muscle": "FCR",  "regulator": 4,    "sensor": 68 },
    { "muscle": "ECU",  "regulator": 5,    "sensor": 69 },
    { "muscle": "ECR",  "regulator": 6,    "sensor": 70 },
    { "muscle": "DI1",  "regulator": 20,   "sensor": 75 },
    { "muscle": "DI2",  "regulator": 21,   "sensor": 76 },
    { "muscle": "DI3",  "regulator": 22,   "sensor": 77 },
    { "muscle": "DI4",  "regulator": null, "sensor": 78 },
    { "muscle": "APB",  "regulator": 23,   "sensor": 79 },
    { "muscle": "PI1",  "regulator": 24,   "sensor": 81 },
    { "muscle": "PI2",  "regulator": 25,   "sensor": 82 },
    { "muscle": "PI3",  "regulator": 26,   "sensor": 83 },
    { "muscle": "PI4",  "regulator": null, "sensor": 84 },
    { "muscle": "OP",   "regulator": 27,   "sensor": 86 },
    { "muscle": "biceps brevis", "regulator": 29, "sensor": 173 },
    { "muscle": "biceps longus", "regulator": 30, "sensor": 174 },
    { "muscle": "brachialis",    "regulator": 31, "sensor": 175 }
  ]
}
```

In `finger/test_config.py`, add after the `good_entries()` helper:

```python
def _real_mapping_has_finger_roles():
    try:
        C.read_mapping(C.DEFAULT_MAPPING_PATH)
        return True
    except C.ConfigError:
        return False
```

and decorate `test_example_loads_against_the_real_mapping`:

```python
    @unittest.skipUnless(_real_mapping_has_finger_roles(),
                         "sensor_mapping.json holds the full-hand mapping (hand_win); "
                         "the finger tools are not adapted to it yet")
    def test_example_loads_against_the_real_mapping(self):
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_sensor_mapping -v` → 8 tests OK.
Run the finger suite → `OK (skipped=1)`.

---

### Task 2: Port discovery that works on Windows

**Files:**
- Modify: `encoder_controller.py:49-71` (discovery block)
- Create: `tests/test_ports.py`

**Interfaces:**
- Produces: `wch_channel(port_info) -> Optional[str]`, `list_wch_com_ports() -> List[str]`, `list_ch9344_ports() -> List[str]` (Linux nodes by number, then Windows channels A–H), `is_ch9344_port(device) -> bool`. `port_index` is unchanged.

- [ ] **Step 1: Write the failing test** — `tests/test_ports.py`:

```python
"""Port discovery: Windows WCH COM ports by name, Linux /dev nodes by glob."""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import encoder_controller as E  # noqa: E402


def com(dev, desc, manu, hwid="USB VID:PID=0000:0000"):
    return SimpleNamespace(device=dev, description=desc, manufacturer=manu, hwid=hwid)


# As pyserial listed them on the station PC on 2026-09-29, plus a Giga.
GIGA = com("COM5", "USB Serial Device (COM5)", "Microsoft", "USB VID:PID=2341:0266 SER=0043")
BLUETOOTH = com("COM3", "Standard Serial over Bluetooth link (COM3)", "Microsoft",
                "BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}")
WINDOWS = [
    com("COM10", "WCH USB-SERIAL Ch D (COM10)", "wch.cn"),
    BLUETOOTH,
    com("COM7", "WCH USB-SERIAL Ch A (COM7)", "wch.cn"),
    com("COM14", "WCH USB-SERIAL Ch H (COM14)", "wch.cn"),
    com("COM8", "WCH USB-SERIAL Ch B (COM8)", "wch.cn"),
    GIGA,
]


class WindowsDiscoveryTests(unittest.TestCase):
    def setUp(self):
        mock.patch("serial.tools.list_ports.comports", return_value=WINDOWS).start()
        mock.patch.object(E.glob, "glob", return_value=[]).start()
        self.addCleanup(mock.patch.stopall)

    def test_channels_sorted_by_letter(self):
        self.assertEqual(E.list_ch9344_ports(), ["COM7", "COM8", "COM10", "COM14"])

    def test_channel_letter(self):
        self.assertEqual(E.wch_channel(WINDOWS[0]), "D")
        self.assertIsNone(E.wch_channel(BLUETOOTH))
        self.assertIsNone(E.wch_channel(GIGA))

    def test_is_ch9344_port(self):
        self.assertTrue(E.is_ch9344_port("COM7"))
        self.assertFalse(E.is_ch9344_port("COM5"))
        self.assertFalse(E.is_ch9344_port("COM3"))
        self.assertFalse(E.is_ch9344_port(None))

    def test_single_port_wch_chip_is_not_a_channel(self):
        ch340 = com("COM20", "USB-SERIAL CH340 (COM20)", "wch.cn")
        with mock.patch("serial.tools.list_ports.comports", return_value=[ch340]):
            self.assertEqual(E.list_ch9344_ports(), [])


class LinuxDiscoveryTests(unittest.TestCase):
    def test_dev_nodes_sorted_by_number(self):
        nodes = {"/dev/ttyCH9344USB*": ["/dev/ttyCH9344USB10", "/dev/ttyCH9344USB2",
                                        "/dev/ttyCH9344USB0"],
                 "/dev/ttyCH343USB*": []}
        with mock.patch("serial.tools.list_ports.comports", return_value=[]), \
                mock.patch.object(E.glob, "glob", side_effect=lambda pat: nodes[pat]):
            self.assertEqual(E.list_ch9344_ports(), ["/dev/ttyCH9344USB0",
                                                     "/dev/ttyCH9344USB2",
                                                     "/dev/ttyCH9344USB10"])
            self.assertTrue(E.is_ch9344_port("/dev/ttyCH9344USB3"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_ports -v`
Expected: `AttributeError: module 'encoder_controller' has no attribute 'wch_channel'` and a wrong list from `list_ch9344_ports()`.

- [ ] **Step 3: Implement** — replace lines 49–71 of `encoder_controller.py` (the block from the `# EKU081 8-port` comment through `list_ch9344_ports`) with:

```python
# EKU081 8-port USB↔RS-485 adapter (a WCH CH348; its Linux driver is `ch9344`).
# Linux: the vendor driver creates one node per port, /dev/ttyCH9344USB0 …
# USB7, and being out of tree those nodes are not always enumerated by
# pyserial, so we glob /dev as well. ttyCH343USB* covers the 1/2-port CH343
# sibling driver.
# Windows: WCH's USBMSER driver names the ports "WCH USB-SERIAL Ch A (COM7)" …
# "Ch H", manufacturer "wch.cn", and hides the USB VID/PID from pyserial
# (hwid "USB VID:PID=0000:0000"), so the channels are matched by name.
CH9344_PREFIXES = ("ttyCH9344", "ttyCH343")
CH9344_GLOBS = ("/dev/ttyCH9344USB*", "/dev/ttyCH343USB*")
WCH_MANUFACTURER = "wch.cn"
WCH_CHANNEL_RE = re.compile(r"USB-SERIAL Ch ([A-Z])\b")


def port_index(device: str) -> int:
    """Trailing number of a device node, for lowest-port-first ordering."""
    m = re.search(r"(\d+)$", device or "")
    return int(m.group(1)) if m else 0


def wch_channel(port_info) -> Optional[str]:
    """Channel letter (A-H) of a Windows WCH multi-port COM port, else None."""
    if (getattr(port_info, "manufacturer", None) or "").lower() != WCH_MANUFACTURER:
        return None
    m = WCH_CHANNEL_RE.search(getattr(port_info, "description", None) or "")
    return m.group(1) if m else None


def list_wch_com_ports() -> List[str]:
    """Windows WCH multi-port COM ports, channel A first."""
    chans = [(wch_channel(p), port_index(p.device), p.device)
             for p in serial.tools.list_ports.comports()]
    return [dev for _, _, dev in sorted(c for c in chans if c[0])]


def list_ch9344_ports() -> List[str]:
    """EKU081 channels: Linux nodes by number, then Windows COM ports by channel."""
    linux = sorted({d for pat in CH9344_GLOBS for d in glob.glob(pat)}, key=port_index)
    return linux + [d for d in list_wch_com_ports() if d not in linux]


def is_ch9344_port(device: str) -> bool:
    """True for a channel of the EKU081 adapter (Linux node or Windows COM port)."""
    if any(pre in (device or "") for pre in CH9344_PREFIXES):
        return True
    return bool(device) and device in list_wch_com_ports()
```

(The old `is_ch9344_port`, `port_index` and `list_ch9344_ports` definitions in that block are removed; `port_index` moves up unchanged.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_ports -v` → 5 tests OK.

---

### Task 3: EncoderController rescans on its own thread and idles when empty

**Files:**
- Modify: `encoder_controller.py` (`__init__`, `_scan`, `_poll_loop`, `rescan`, `get_state`, `close`)
- Create: `tests/test_encoder_rescan.py`

**Interfaces:**
- Produces: `EncoderController(..., autostart: bool = True)` (False opens no port; for tests), `request_rescan(slave_ids: Optional[Sequence[int]] = None)`, `_poll_once()`. `rescan()` stays as an alias. `get_state()["scanned_range"]` is `None` for an empty id list. After a scan, `readings` holds only ids that answered.

- [ ] **Step 1: Write the failing test** — `tests/test_encoder_rescan.py`:

```python
"""EncoderController scan/rescan logic, with the serial read faked out."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import encoder_controller as E  # noqa: E402


def make(answering, ids):
    c = E.EncoderController(slave_ids=ids, interval=0.0, autostart=False)
    c.running = True
    c.reads = []

    def fake_read(slave):
        c.reads.append(slave)
        if slave not in answering:
            raise E.SerialTimeoutError("timeout")
        return E.EncoderReading(slave=slave, online=True, absolute_position=slave,
                                message="ok", timestamp=0.0)

    c._read_encoder = fake_read
    return c


class ScanTests(unittest.TestCase):
    def test_autostart_false_opens_nothing(self):
        c = E.EncoderController(slave_ids=[1], autostart=False)
        self.assertIsNone(c.ser)
        self.assertFalse(c.connected)

    def test_scan_keeps_answering_ids(self):
        c = make({50, 52}, [50, 51, 52])
        c._scan()
        self.assertEqual(c.active_slaves, [50, 52])
        self.assertEqual(sorted(c.readings), [50, 52])
        self.assertIn("Scan complete: 2", c.drain_messages()[-1])

    def test_rescan_drops_ids_that_stopped_answering(self):
        answering = {50, 51}
        c = make(answering, [50, 51])
        c._scan()
        answering.discard(51)
        c.request_rescan()
        c._poll_once()
        self.assertEqual(c.active_slaves, [50])
        self.assertEqual(sorted(c.readings), [50])

    def test_new_id_list_applies_on_the_poll_thread(self):
        c = make({173}, [50])
        c._scan()
        c.request_rescan([173])
        self.assertEqual(c.slave_ids, [50])      # not yet: the poll thread swaps it
        c._poll_once()
        self.assertEqual(c.slave_ids, [173])
        self.assertEqual(c.active_slaves, [173])

    def test_empty_port_idles_without_reading(self):
        c = make(set(), [50, 51])
        c._scan()
        c.reads.clear()
        c._poll_once()
        self.assertEqual(c.reads, [])
        self.assertIn("idle until RESCAN", c.drain_messages()[-1])

    def test_polls_active_ids(self):
        c = make({50}, [50, 51])
        c._scan()
        c.reads.clear()
        c._poll_once()
        self.assertEqual(c.reads, [50])

    def test_empty_id_list(self):
        c = make(set(), [])
        c._scan()
        self.assertIsNone(c.get_state()["scanned_range"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_encoder_rescan -v`
Expected: `TypeError: EncoderController.__init__() got an unexpected keyword argument 'autostart'`.

- [ ] **Step 3: Implement** in `encoder_controller.py`:

Constructor signature and the tail of `__init__`:

```python
        timeout: float = 0.06,
        interval: float = 0.2,
        autostart: bool = True,
    ):
```

```python
        self.state_lock = threading.Lock()
        self.message_queue = deque(maxlen=50)

        # RESCAN requests from other threads. The poll thread runs the scan, so
        # only one thread ever talks on the port (two would interleave frames).
        self._rescan_requested = threading.Event()
        self._pending_ids: Optional[List[int]] = None

        if not autostart:
            return            # tests: no port, no thread
        if port:
            self.connect(port)
        else:
            self.auto_connect()
```

Replace `_scan` and `_poll_loop` with:

```python
    def _ids_text(self) -> str:
        return (f"ids {self.slave_ids[0]}-{self.slave_ids[-1]}"
                if self.slave_ids else "no ids")

    def _scan(self) -> None:
        """One pass over every configured id to find the encoders present.

        Readings of ids that no longer answer are dropped, so a sensor that
        went away does not keep showing its last value.
        """
        found: Dict[int, EncoderReading] = {}
        for slave in self.slave_ids:
            if not self.running:
                return
            try:
                found[slave] = self._read_encoder(slave)
            except Exception:
                continue
        with self.state_lock:
            self.readings = found
        self.active_slaves = list(found)
        self.message_queue.append(
            f"Scan complete: {len(found)} encoder(s) online in {self._ids_text()}"
            if found else f"No encoders found in {self._ids_text()}; idle until RESCAN")

    def _poll_loop(self) -> None:
        """Scan once, then keep refreshing the encoders that answered."""
        self._scan()
        while self.running:
            self._poll_once()

    def _poll_once(self) -> None:
        """One poll-thread step: a requested rescan, an idle wait, or a sweep."""
        if self._rescan_requested.is_set():
            self._rescan_requested.clear()
            if self._pending_ids is not None:
                self.slave_ids, self._pending_ids = self._pending_ids, None
            self._scan()
            return
        if not self.active_slaves:
            # Nothing answered on this port: stay quiet until RESCAN.
            self._rescan_requested.wait(0.5)
            return
        for slave in self.active_slaves:
            if not self.running:
                return
            try:
                reading = self._read_encoder(slave)
            except Exception as exc:
                reading = EncoderReading(
                    slave=slave,
                    online=False,
                    message=str(exc).replace("\n", " ")[:40],
                    timestamp=time.time(),
                )
            with self.state_lock:
                self.readings[slave] = reading
        # Sleep between sweeps, but wake at once for a RESCAN.
        self._rescan_requested.wait(self.interval)
```

Replace `rescan` with:

```python
    def request_rescan(self, slave_ids: Optional[Sequence[int]] = None) -> None:
        """Ask the poll thread to re-sweep, optionally over a new id list."""
        if slave_ids is not None:
            self._pending_ids = list(slave_ids)
        self.message_queue.append("Rescanning...")
        self._rescan_requested.set()

    def rescan(self) -> None:
        """Alias of request_rescan() for older callers."""
        self.request_rescan()
```

In `get_state`, replace the `"scanned_range"` line with:

```python
            "scanned_range": ([self.slave_ids[0], self.slave_ids[-1]]
                              if self.slave_ids else None),
```

In `close`, wake the thread before joining:

```python
        self.running = False
        self._rescan_requested.set()      # wake an idle or sleeping poll thread
        if self.poll_thread:
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_encoder_rescan tests.test_ports -v` → all OK.

---

### Task 4: SensorHub

**Files:**
- Create: `sensor_hub.py`, `tests/test_sensor_hub.py`

**Interfaces:**
- Consumes: `EncoderController(port=, slave_ids=, baudrate=, parity=, counts_per_turn=, drum_diameter_mm=, timeout=, interval=)` with `.connected`, `.active_slaves`, `.get_state()`, `.zero(slave)`, `.zero_all()`, `.clear_zero()`, `.request_rescan(ids)`, `.drain_messages()`, `.close()`.
- Produces: `SensorHub(ports, slave_ids, factory=None, **encoder_kwargs)` with `.connected`, `.ports`, `.owners()`, `.duplicates()`, `.get_state()` → `{"connected", "ports": [{"port", "connected", "found", "online"}], "encoders": [entry + "port"], "online", "zeroed", "duplicates": [ids]}`, `.zero(slave)`, `.zero_all()`, `.clear_zero()`, `.request_rescan(slave_ids=None)`, `.drain_messages()`, `.close()`. `factory(port, slave_ids)` returns a controller.

- [ ] **Step 1: Write the failing test** — `tests/test_sensor_hub.py`:

```python
"""SensorHub merging, duplicate detection and routing, with fake controllers."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensor_hub import SensorHub  # noqa: E402


class FakeController:
    def __init__(self, port, slave_ids, answering):
        self.port_name = port
        self.slave_ids = list(slave_ids)
        self.active_slaves = [s for s in slave_ids if s in answering]
        self.connected = True
        self.calls = []
        self.messages = [f"[OK] Connected to {port}"]

    def get_state(self):
        enc = [{"slave": s, "online": True, "position_mm": float(s), "zeroed": False,
                "message": "ok"} for s in self.active_slaves]
        return {"connected": self.connected, "encoders": enc, "online": len(enc)}

    def zero(self, slave):
        self.calls.append(("zero", slave))

    def zero_all(self):
        self.calls.append(("zero_all",))

    def clear_zero(self):
        self.calls.append(("clear_zero",))

    def request_rescan(self, slave_ids=None):
        self.calls.append(("rescan", slave_ids))

    def drain_messages(self):
        out, self.messages = self.messages, []
        return out

    def close(self):
        self.calls.append(("close",))


def make_hub(wiring, ids=(50, 51, 52, 173, 174)):
    fakes = {}

    def factory(port, slave_ids):
        if port == "BAD":
            raise OSError("could not open port 'BAD'")
        fakes[port] = FakeController(port, slave_ids, wiring.get(port, ()))
        return fakes[port]

    return SensorHub(list(wiring), ids, factory=factory), fakes


WIRING = {"COM7": {50, 51}, "COM8": {52, 173}, "COM9": set()}


class HubTests(unittest.TestCase):
    def test_merged_state(self):
        hub, _ = make_hub(WIRING)
        st = hub.get_state()
        self.assertTrue(st["connected"])
        self.assertEqual([e["slave"] for e in st["encoders"]], [50, 51, 52, 173])
        self.assertEqual({e["slave"]: e["port"] for e in st["encoders"]},
                         {50: "COM7", 51: "COM7", 52: "COM8", 173: "COM8"})
        self.assertEqual(st["online"], 4)
        self.assertEqual([(p["port"], p["found"]) for p in st["ports"]],
                         [("COM7", 2), ("COM8", 2), ("COM9", 0)])
        self.assertEqual(st["duplicates"], [])

    def test_duplicate_id_keeps_first_port_and_warns_once(self):
        hub, _ = make_hub({"COM7": {50}, "COM8": {50, 51}})
        st = hub.get_state()
        self.assertEqual(st["duplicates"], [50])
        self.assertEqual(st["encoders"][0]["port"], "COM7")
        warnings = [m for m in hub.drain_messages() if "[WARN]" in m]
        self.assertEqual(warnings, ["[WARN] sensor 50 answers on COM7 and COM8; showing COM7"])
        self.assertEqual([m for m in hub.drain_messages() if "[WARN]" in m], [])

    def test_zero_goes_to_the_owning_port(self):
        hub, fakes = make_hub(WIRING)
        hub.zero(173)
        self.assertEqual(fakes["COM8"].calls, [("zero", 173)])
        self.assertEqual(fakes["COM7"].calls, [])

    def test_zero_unknown_sensor_warns(self):
        hub, fakes = make_hub(WIRING)
        hub.drain_messages()
        hub.zero(99)
        self.assertEqual(hub.drain_messages(),
                         ["[WARN] cannot zero sensor 99: not found on any port"])
        self.assertTrue(all(f.calls == [] for f in fakes.values()))

    def test_fan_out(self):
        hub, fakes = make_hub(WIRING)
        hub.zero_all()
        hub.clear_zero()
        hub.request_rescan([1, 2])
        hub.close()
        for f in fakes.values():
            self.assertEqual(f.calls, [("zero_all",), ("clear_zero",), ("rescan", [1, 2]),
                                       ("close",)])

    def test_messages_are_prefixed_with_the_port(self):
        hub, _ = make_hub({"COM7": {50}})
        self.assertEqual(hub.drain_messages(), ["COM7: [OK] Connected to COM7"])

    def test_a_port_that_fails_to_open_is_skipped(self):
        hub, _ = make_hub({"COM7": {50}, "BAD": set()})
        self.assertEqual(hub.ports, ["COM7"])
        self.assertIn("[ERR] BAD: could not open port 'BAD'", hub.drain_messages())

    def test_no_ports_is_not_connected(self):
        hub, _ = make_hub({})
        self.assertFalse(hub.connected)
        self.assertEqual(hub.get_state()["encoders"], [])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_sensor_hub -v`
Expected: `ModuleNotFoundError: No module named 'sensor_hub'`.

- [ ] **Step 3: Implement** — `sensor_hub.py`:

```python
#!/usr/bin/env python3
"""
sensor_hub.py

Reads the station's draw-wire sensors on several RS-485 ports at once: one
EncoderController, and so one poll thread, per EKU081 channel, merged into a
single table keyed by sensor id. Sensor ids are unique across the station, so
the hub only learns which port each id answers on. An id that answers on two
ports is reported as a duplicate, and the first port's reading is shown.
"""

from __future__ import annotations

from collections import deque
from typing import Callable, Dict, List, Optional, Sequence

from encoder_controller import EncoderController


def _default_factory(**encoder_kwargs) -> Callable:
    def make(port: str, slave_ids: Sequence[int]) -> EncoderController:
        return EncoderController(port=port, slave_ids=slave_ids, **encoder_kwargs)
    return make


class SensorHub:
    """Several EncoderControllers behind the single-controller interface."""

    def __init__(self, ports: Sequence[str], slave_ids: Sequence[int],
                 factory: Optional[Callable] = None, **encoder_kwargs):
        make = factory or _default_factory(**encoder_kwargs)
        self.message_queue = deque(maxlen=50)
        self.controllers: Dict[str, EncoderController] = {}
        for port in ports:
            try:
                self.controllers[port] = make(port, list(slave_ids))
            except Exception as exc:
                # One bad channel must not keep the others from running.
                self.message_queue.append(f"[ERR] {port}: {exc}")
        self._reported_duplicates: Dict[int, List[str]] = {}

    @property
    def connected(self) -> bool:
        return any(c.connected for c in self.controllers.values())

    @property
    def ports(self) -> List[str]:
        return list(self.controllers)

    def owners(self) -> Dict[int, List[str]]:
        """Sensor id -> the ports it answered on, in port order."""
        out: Dict[int, List[str]] = {}
        for port, c in self.controllers.items():
            for slave in list(c.active_slaves):
                out.setdefault(slave, []).append(port)
        return out

    def duplicates(self) -> Dict[int, List[str]]:
        return {s: ps for s, ps in self.owners().items() if len(ps) > 1}

    def get_state(self) -> dict:
        """Merged snapshot: one entry per sensor id (tagged with its port),
        plus a per-port summary."""
        encoders: Dict[int, dict] = {}
        ports = []
        for port, c in self.controllers.items():
            st = c.get_state()
            ports.append({"port": port, "connected": st["connected"],
                          "found": len(c.active_slaves), "online": st["online"]})
            for e in st["encoders"]:
                encoders.setdefault(e["slave"], dict(e, port=port))
        merged = [encoders[s] for s in sorted(encoders)]
        return {
            "connected": self.connected,
            "ports": ports,
            "encoders": merged,
            "online": sum(1 for e in merged if e["online"]),
            "zeroed": sum(1 for e in merged if e["zeroed"]),
            "duplicates": sorted(self.duplicates()),
        }

    # ------------------------------------------------------------------
    # Web-driven actions
    # ------------------------------------------------------------------

    def zero(self, slave: int) -> None:
        ports = self.owners().get(slave)
        if not ports:
            self.message_queue.append(f"[WARN] cannot zero sensor {slave}: not found on any port")
            return
        self.controllers[ports[0]].zero(slave)

    def zero_all(self) -> None:
        for c in self.controllers.values():
            c.zero_all()

    def clear_zero(self) -> None:
        for c in self.controllers.values():
            c.clear_zero()

    def request_rescan(self, slave_ids: Optional[Sequence[int]] = None) -> None:
        for c in self.controllers.values():
            c.request_rescan(slave_ids)

    def drain_messages(self) -> List[str]:
        """Hub messages, then each port's messages prefixed with the port, then
        a warning for each duplicate id not reported before."""
        msgs = list(self.message_queue)
        self.message_queue.clear()
        for port, c in self.controllers.items():
            msgs += [f"{port}: {m}" for m in c.drain_messages()]
        dups = self.duplicates()
        for slave, ports in sorted(dups.items()):
            if self._reported_duplicates.get(slave) != ports:
                msgs.append(f"[WARN] sensor {slave} answers on {' and '.join(ports)}; "
                            f"showing {ports[0]}")
        self._reported_duplicates = dups
        return msgs

    def close(self) -> None:
        for c in self.controllers.values():
            c.close()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_sensor_hub -v` → 8 tests OK.

---

### Task 5: Session pressure cap

**Files:**
- Modify: `valve_controller.py` (add `check_max_kpa`; `__init__`, `_value_within_limit`, `set_valve` docstring, `get_state`)
- Create: `tests/test_valve_cap.py`

**Interfaces:**
- Produces: `check_max_kpa(max_kpa) -> float` (raises `ValueError` unless 0 < value ≤ 140), `ValveController(port=None, baudrate=115200, max_kpa=MAX_INPUT_KPA)` with `.max_kpa`; `get_state()["max_value"] == .max_kpa`.

- [ ] **Step 1: Write the failing test** — `tests/test_valve_cap.py`:

```python
"""The --max-kpa session cap can only lower the station cap."""

import os
import sys
import unittest
from collections import deque
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from valve_controller import MAX_INPUT_KPA, ValveController, check_max_kpa  # noqa: E402


def bare(max_kpa):
    """A ValveController with no serial port, for limit checks only."""
    vc = ValveController.__new__(ValveController)
    vc.max_kpa = max_kpa
    vc.message_queue = deque()
    return vc


class CapTests(unittest.TestCase):
    def test_station_cap_unchanged(self):
        self.assertEqual(MAX_INPUT_KPA, 140.0)

    def test_valid_caps(self):
        self.assertEqual(check_max_kpa(50), 50.0)
        self.assertEqual(check_max_kpa("140"), 140.0)

    def test_invalid_caps(self):
        for bad in (0, -5, 140.1, 500):
            with self.assertRaises(ValueError):
                check_max_kpa(bad)

    def test_instance_limit(self):
        vc = bare(50.0)
        self.assertTrue(vc._value_within_limit(50.0))
        self.assertFalse(vc._value_within_limit(50.5))
        self.assertIn("max 50 kPa", vc.message_queue[-1])

    def test_bad_cap_is_rejected_before_any_port_is_touched(self):
        with mock.patch.object(ValveController, "auto_connect") as ac, \
                mock.patch.object(ValveController, "connect") as cn:
            with self.assertRaises(ValueError):
                ValveController(max_kpa=200)
            ac.assert_not_called()
            cn.assert_not_called()


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_valve_cap -v`
Expected: `ImportError: cannot import name 'check_max_kpa'`.

- [ ] **Step 3: Implement** in `valve_controller.py`. After `mv_to_bar`:

```python
def check_max_kpa(max_kpa):
    """Validate a session pressure cap in kPa: it may only lower MAX_INPUT_KPA."""
    value = float(max_kpa)
    if not 0 < value <= MAX_INPUT_KPA:
        raise ValueError(
            f"max kPa must be > 0 and <= {MAX_INPUT_KPA:g}, got {max_kpa}")
    return value
```

Constructor:

```python
    def __init__(self, port=None, baudrate=115200, max_kpa=MAX_INPUT_KPA):
        # Checked first, so a bad cap never gets as far as opening a port.
        self.max_kpa = check_max_kpa(max_kpa)
        self.ser = None
```

`_value_within_limit`:

```python
    def _value_within_limit(self, kpa):
        """Return True if a kPa setpoint is allowed to be sent to the hardware."""
        if kpa > self.max_kpa:
            self.message_queue.append(
                f"[ERR] Value {kpa:g} kPa exceeds limit (max {self.max_kpa:g} kPa)"
            )
            return False
        return True
```

`set_valve` docstring first line: `"""Set a single valve to `kpa` (output pressure, -100..max_kpa kPa).`
`get_state`: `"max_value": self.max_kpa,`.
Also correct the stale bus comments in this file: the module docstring line `- Valves 16..31  on Wire1  (SDA1/SCL1)` → `- Valves 16..31  on Wire2  (SDA2/SCL2)`, and `BUS_SPLIT = 16          # valves 0..15 -> Wire, 16..31 -> Wire1` → `... -> Wire2`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_valve_cap -v` → 5 tests OK. Finger suite still OK (skipped=1).

---

### Task 6: app.py uses the hub, the new flags and Windows-safe port resolution

**Files:**
- Modify: `app.py`
- Create: `tests/test_app_ports.py`

**Interfaces:**
- Consumes: `SensorHub` (Task 4), `load_all` (Task 1), `list_ch9344_ports` (Task 2), `check_max_kpa` (Task 5), `DEFAULT_SLAVE_IDS`.
- Produces: `resolve_ports(args) -> Tuple[Optional[str], List[str]]` with `args` fields `valve_port, rs485_ports, no_valves, no_encoders`; `default_slave_ids(index) -> List[int]`; `parse_port_list(value) -> List[str]`; state key `"sensor_index"` = `{str(id): {"muscle", "regulator"}}`; `"encoders"` is the hub state.

- [ ] **Step 1: Write the failing test** — `tests/test_app_ports.py`:

```python
"""app.resolve_ports on Windows and the mapping-derived default ids."""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app  # noqa: E402
from tests.test_ports import BLUETOOTH, GIGA, WINDOWS  # noqa: E402


def args(**kw):
    base = dict(valve_port=None, rs485_ports=None, no_valves=False, no_encoders=False)
    base.update(kw)
    return SimpleNamespace(**base)


class ResolvePortsTests(unittest.TestCase):
    def ports(self, listing):
        mock.patch("serial.tools.list_ports.comports", return_value=listing).start()
        mock.patch("encoder_controller.glob.glob", return_value=[]).start()
        self.addCleanup(mock.patch.stopall)

    def test_giga_and_every_channel(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args()),
                         ("COM5", ["COM7", "COM8", "COM10", "COM14"]))

    def test_explicit_rs485_ports(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args(rs485_ports=["COM7", "COM8"])),
                         ("COM5", ["COM7", "COM8"]))

    def test_no_adapter_never_falls_back_to_giga_or_bluetooth(self):
        self.ports([GIGA, BLUETOOTH])
        self.assertEqual(app.resolve_ports(args(no_valves=True)), (None, []))

    def test_same_port_twice_is_refused(self):
        self.ports(WINDOWS)
        with self.assertRaises(SystemExit):
            app.resolve_ports(args(valve_port="COM7", rs485_ports=["COM7"]))

    def test_no_encoders(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args(no_encoders=True)), ("COM5", []))


class DefaultIdsTests(unittest.TestCase):
    def test_ids_come_from_the_mapping(self):
        self.assertEqual(app.default_slave_ids({175: {}, 50: {}, 53: {}}), [50, 53, 175])

    def test_empty_mapping_falls_back(self):
        from encoder_controller import DEFAULT_SLAVE_IDS
        self.assertEqual(app.default_slave_ids({}), list(DEFAULT_SLAVE_IDS))

    def test_port_list(self):
        self.assertEqual(app.parse_port_list(" COM7, COM8 ,"), ["COM7", "COM8"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_app_ports -v`
Expected: `AttributeError: module 'app' has no attribute 'default_slave_ids'` and wrong `resolve_ports` results.

- [ ] **Step 3: Implement** in `app.py`:

Module docstring: replace the EKU081 bullet and usage lines with:

```python
  - EKU081 8-port USB↔RS-485 adapter (GJW draw-wire sensors) — one poll
    thread per channel, merged by SensorHub. Linux: /dev/ttyCH9344USB0 … USB7;
    Windows: "WCH USB-SERIAL Ch A (COM7)" … "Ch H". Every channel is opened by
    default; --rs485-ports restricts it.

Both devices are opened at once, so the browser can command valves and watch
all sensors live side by side. Either device may be absent — the app runs with
whichever it can open.

Usage:
    python app.py                                   # auto-detect everything
    python app.py --rs485-ports COM7,COM8,COM9,COM10 --max-kpa 50
    python app.py --valve-port /dev/ttyACM0 --rs485-ports /dev/ttyCH9344USB0
    python app.py --no-valves                       # sensors only
```

Imports:

```python
import argparse
import time
from types import SimpleNamespace
from typing import List, Optional, Tuple

import serial.tools.list_ports
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO

from valve_controller import ValveController, MAX_INPUT_KPA, NUM_VALVES, check_max_kpa
from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    DEFAULT_SLAVE_IDS,
    list_ch9344_ports,
    parse_slave_ids,
)
from sensor_hub import SensorHub
from sensor_mapping import DEFAULT_MAPPING_PATH, load_all
```

Globals:

```python
# Two shared devices, created in main(). Either may stay None if absent or
# disabled. The sensors are one SensorHub over every RS-485 channel in use.
valve_controller = None
sensor_hub = None
_broadcast_started = False

# regulator (valve id) -> {"muscle", "sensor"} and sensor id -> {"muscle",
# "regulator"}; loaded in main(), reloaded on RESCAN so edits to
# sensor_mapping.json apply without restarting the server.
sensor_map = {}
sensor_index = {}
mapping_path = DEFAULT_MAPPING_PATH
ids_from_mapping = True     # False when --ids fixes the scanned ids
```

Port resolution block (replaces `_usb_ports` through `resolve_ports`; `_blob` and `_is_giga` stay):

```python
def _usb_ports(eku):
    """Serial ports that look like real USB/CDC devices, plus the EKU081
    channels in `eku` (globbed on Linux, so maybe unseen by pyserial).
    Bluetooth COM ports are skipped: opening one can stall on a connect."""
    out = []
    for p in serial.tools.list_ports.comports():
        dev = p.device or ""
        if "BTHENUM" in (p.hwid or "").upper():
            continue
        if ("ttyACM" in dev or "ttyUSB" in dev or dev in eku
                or dev.upper().startswith("COM")):
            out.append(p)
    seen = {p.device for p in out}
    out += [SimpleNamespace(device=d, description="CH9344 USB-RS485 (EKU081)",
                            hwid="", product=None, manufacturer=None)
            for d in eku if d not in seen]
    return out
```

`_is_rs485_bridge`: delete its first two lines (`if is_ch9344_port(p.device): return True`); EKU081 channels are handled before it is ever called.

```python
def resolve_ports(args) -> Tuple[Optional[str], List[str]]:
    """Pick the valve regulator port and the RS-485 sensor ports, all distinct."""
    eku = list_ch9344_ports()
    ports = _usb_ports(set(eku))

    valve_port = args.valve_port
    if not valve_port and not args.no_valves:
        # An EKU081 channel is never the regulator, whatever its description says.
        valve_port = next((p.device for p in ports
                           if _is_giga(p) and p.device not in eku), None)

    rs485_ports: List[str] = []
    if not args.no_encoders:
        if args.rs485_ports:
            rs485_ports = list(args.rs485_ports)
        else:
            # Every EKU081 channel: channels without sensors go idle after their
            # first scan. --rs485-ports restricts the set.
            rs485_ports = [d for d in eku if d != valve_port]
            if not rs485_ports:
                # No EKU081: fall back to a generic bridge, never the Giga.
                others = [p for p in ports if p.device != valve_port and not _is_giga(p)]
                fallback = (next((p.device for p in others if _is_rs485_bridge(p)), None)
                            or next((p.device for p in others), None))
                rs485_ports = [fallback] if fallback else []

    if valve_port and valve_port in rs485_ports:
        raise SystemExit(
            f"[ERR] {valve_port} is both the valve port and an RS-485 port; "
            "pass distinct --valve-port / --rs485-ports")
    return valve_port, rs485_ports


def default_slave_ids(index) -> List[int]:
    """Sensor ids to scan without --ids: every sensor in the mapping, or the old
    default range when the mapping names none."""
    return sorted(index) or list(DEFAULT_SLAVE_IDS)


def parse_port_list(value: str) -> List[str]:
    """'COM7,COM8' -> ['COM7', 'COM8']."""
    ports = [p.strip() for p in value.split(",") if p.strip()]
    if not ports:
        raise argparse.ArgumentTypeError("expected at least one port")
    return ports
```

`build_state`:

```python
def build_state():
    """Full snapshot pushed to the browser: valves, sensors, joined muscles."""
    enc_state = sensor_hub.get_state() if sensor_hub else None
    return {
        "valves": valve_controller.get_state() if valve_controller else None,
        "encoders": enc_state,
        "muscles": _muscles_state(enc_state),
        "sensor_index": {str(s): v for s, v in sensor_index.items()},
        "length_bar_min_mm": LENGTH_BAR_MIN_MM,
        "length_bar_max_mm": LENGTH_BAR_MAX_MM,
    }
```

`_broadcast_loop`: replace the encoder lines with

```python
        if sensor_hub:
            lines += [f"[ENC] {m}" for m in sensor_hub.drain_messages()]
```

`on_apply_all`: `if val > valve_controller.max_kpa:` (was `MAX_INPUT_KPA`).

Encoder events:

```python
@socketio.on('zero')
def on_zero():
    """Global ZERO ALL — zero every online sensor."""
    if sensor_hub is not None:
        sensor_hub.zero_all()


@socketio.on('zero_sensor')
def on_zero_sensor(data):
    """Individual zero — zero one sensor. data = {slave: int}."""
    if sensor_hub is None:
        return
    try:
        slave = int(data.get('slave'))
    except (ValueError, TypeError):
        return
    sensor_hub.zero(slave)


@socketio.on('clear_zero')
def on_clear_zero():
    """Global CLEAR ZERO — drop every zero offset (back to absolute mm)."""
    if sensor_hub is not None:
        sensor_hub.clear_zero()


@socketio.on('rescan')
def on_rescan():
    """Reload the mapping and re-sweep every RS-485 channel (on its own thread)."""
    global sensor_map, sensor_index
    sensor_map, sensor_index = load_all(mapping_path)
    if sensor_hub is not None:
        sensor_hub.request_rescan(
            default_slave_ids(sensor_index) if ids_from_mapping else None)
```

`main()`:

```python
def main():
    global valve_controller, sensor_hub, sensor_map, sensor_index
    global mapping_path, ids_from_mapping

    parser = argparse.ArgumentParser(
        description="vc2 pneumatic station web app (valves + RS-485 sensors)")
    # Ports
    parser.add_argument('--valve-port', default=None,
                        help='Serial port of the Giga R1 regulator (e.g. COM5, /dev/ttyACM0)')
    parser.add_argument('--rs485-ports', type=parse_port_list, default=None,
                        help='Comma list of EKU081 channels (default: all), '
                             'e.g. COM7,COM8,COM9,COM10')
    parser.add_argument('--rs485-port', default=None,
                        help='A single RS-485 port (same as --rs485-ports with one port)')
    parser.add_argument('--no-valves', action='store_true',
                        help='Do not open the valve regulator')
    parser.add_argument('--no-encoders', '--ne', action='store_true',
                        help='Do not open the RS-485 sensor bus')
    parser.add_argument('--max-kpa', type=float, default=MAX_INPUT_KPA,
                        help=f'Session pressure cap in kPa, 0 < cap <= {MAX_INPUT_KPA:g} '
                             f'(default {MAX_INPUT_KPA:g})')
    # Encoder / Modbus options
    parser.add_argument('--ids', type=parse_slave_ids, default=None,
                        help='Sensor ids to scan (default: the ids in the mapping), '
                             'e.g. 50-86,173-175')
    ... (--baud, --parity, --counts-per-turn, --drum-diameter, --timeout,
         --interval, --mapping, --host, --http-port unchanged)
    args = parser.parse_args()
    try:
        max_kpa = check_max_kpa(args.max_kpa)
    except ValueError as exc:
        parser.error(str(exc))
    if args.rs485_port and not args.rs485_ports:
        args.rs485_ports = [args.rs485_port]

    print("=" * 65)
    print("   VC2 PNEUMATIC STATION (Web App) — valves + RS-485 sensors")
    print("=" * 65)

    mapping_path = args.mapping
    sensor_map, sensor_index = load_all(mapping_path)
    ids_from_mapping = args.ids is None
    slave_ids = args.ids or default_slave_ids(sensor_index)

    valve_port, rs485_ports = resolve_ports(args)
    print(f"Valve regulator : {valve_port or '(none / disabled)'}")
    print(f"RS-485 sensors  : {', '.join(rs485_ports) or '(none / disabled)'}")
    print(f"Sensor mapping  : {mapping_path} ({len(sensor_map)} regulator(s), "
          f"{len(sensor_index)} sensor(s))")
    print(f"Scanned ids     : {len(slave_ids)} ({slave_ids[0]}..{slave_ids[-1]}, "
          f"{'from the mapping' if ids_from_mapping else '--ids'})")
    print(f"Pressure cap    : {max_kpa:g} kPa")

    if not args.no_valves and valve_port:
        vc = ValveController(port=valve_port, max_kpa=max_kpa)
        valve_controller = vc if vc.connected else None
        if valve_controller is None:
            print(f"[WARN] Could not open valve regulator on {valve_port}")

    if not args.no_encoders and rs485_ports:
        hub = SensorHub(
            rs485_ports, slave_ids,
            baudrate=args.baud,
            parity=args.parity,
            counts_per_turn=args.counts_per_turn,
            drum_diameter_mm=args.drum_diameter,
            timeout=args.timeout,
            interval=args.interval,
        )
        if hub.connected:
            sensor_hub = hub
        else:
            for m in hub.drain_messages():
                print(m)
            print(f"[WARN] Could not open any RS-485 port of {', '.join(rs485_ports)}")

    if valve_controller is None and sensor_hub is None:
        print("[ERR] Opened neither device. Check connections / --valve-port / --rs485-ports.")
        return
    ... (serve loop unchanged, except the finally block closes sensor_hub:)
        if sensor_hub is not None:
            sensor_hub.close()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest discover -s tests -t . -v` → all OK.
Run: `$PY app.py --help` → shows `--rs485-ports`, `--max-kpa`, `--ids` default text.
Run: `$PY app.py --max-kpa 200` → exits with `max kPa must be > 0 and <= 140, got 200.0`.

---

### Task 7: All-sensors strip in the web UI

**Files:**
- Modify: `templates/index.html`, `static/app.js`, `static/style.css`

**Interfaces:**
- Consumes: state keys `encoders` (hub state: `connected`, `ports[]`, `encoders[]` with `slave, port, online, position_mm, message`, `online`, `duplicates`) and `sensor_index` (`{"50": {"muscle", "regulator"}}`).

- [ ] **Step 1: `templates/index.html`** — in `.pills`, after the valve pill:

```html
      <div class="status-pill" id="sensorPill">
        <span class="dot"></span><span id="sensorText">SENSORS …</span>
      </div>
```

After the valve `</section>` and before `<section class="log-wrap">`:

```html
  <!-- ===================== SENSORS (EKU081 RS-485) ===================== -->
  <section class="panel" id="sensorPanel">
    <h2 class="panel-title">Sensors <span class="src">— EKU081 RS-485</span>
      <span class="sensor-summary" id="sensorSummary"></span></h2>
    <div class="sensor-strip" id="sensorStrip"></div>
    <p class="hint">
      Every sensor in the mapping plus any other id that answers, by id:
      muscle, regulator, port and length. <strong>Dashed</strong> = mapped but not
      found on any port; <strong>teal border</strong> = answering but not in the
      mapping; <strong>red border</strong> = the same id answers on two ports.
    </p>
  </section>
```

- [ ] **Step 2: `static/app.js`** — header comment gains a line: `// A strip below lists every sensor (mapped or answering) with its port.` Fix the bus label:

```js
let BUS_SPLIT = 16;           // valves below this are on Wire, at/above on Wire2
...
function busName(valve) {
  return valve < BUS_SPLIT ? 'Wire' : 'Wire2';
}
```

Add before the Activity-log section:

```js
// ------------------------------------------------------------------
// Rendering — all sensors: every mapped id plus any other id that answers
// ------------------------------------------------------------------
function sensorChip(id, e, info, dup) {
  const chip = document.createElement('div');
  chip.className = 'chip';
  if (!info) chip.classList.add('unmapped');
  if (!e) chip.classList.add('missing');
  else if (!e.online) chip.classList.add('offline');
  if (dup) chip.classList.add('dup');

  const head = document.createElement('div');
  head.className = 'chip-head';
  head.textContent = `${id} ${info ? info.muscle : 'unmapped'}`;
  const sub = document.createElement('div');
  sub.className = 'chip-sub';
  const reg = info && info.regulator !== null ? `V${info.regulator}` : 'no V';
  sub.textContent = `${reg} · ${e ? e.port : 'not found'}`;
  const val = document.createElement('div');
  val.className = 'chip-val';
  if (!e) val.textContent = '—';
  else if (e.online && e.position_mm !== null) val.textContent = `${e.position_mm.toFixed(2)} mm`;
  else val.textContent = e.message || 'offline';

  chip.append(head, sub, val);
  chip.title = `${head.textContent} · ${sub.textContent} · ${val.textContent}`;
  return chip;
}

function renderSensors(state) {
  const enc = state.encoders;
  const index = state.sensor_index || {};
  const pill = document.getElementById('sensorPill');
  const text = document.getElementById('sensorText');
  const summary = document.getElementById('sensorSummary');
  const strip = document.getElementById('sensorStrip');

  const found = {};
  ((enc && enc.encoders) || []).forEach((e) => { found[e.slave] = e; });
  const ids = [...new Set([...Object.keys(found), ...Object.keys(index)].map(Number))]
    .sort((a, b) => a - b);
  const dups = new Set((enc && enc.duplicates) || []);

  strip.innerHTML = '';
  ids.forEach((id) => strip.appendChild(sensorChip(id, found[id], index[String(id)], dups.has(id))));

  const expected = Object.keys(index).length || ids.length;
  if (!enc) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'SENSORS OFF';
    summary.textContent = '';
    return;
  }
  if (!enc.connected) {
    pill.className = 'status-pill disconnected';
    text.textContent = 'SENSORS LOST';
  } else {
    pill.className = enc.online > 0 ? 'status-pill active' : 'status-pill';
    text.textContent = `SENSORS ${enc.online}/${expected}`;
  }
  const perPort = (enc.ports || []).map((p) => `${p.port} ${p.online}`).join(' · ');
  summary.textContent = perPort + (dups.size ? ` · duplicate ids: ${[...dups].join(', ')}` : '');
}
```

Socket wiring:

```js
socket.on('state', (state) => { renderValves(state); renderSensors(state); });
...
socket.on('disconnect', () => {
  ['valvePill', 'sensorPill'].forEach((id) => {
    document.getElementById(id).className = 'status-pill disconnected';
  });
  document.getElementById('valveText').textContent = 'SERVER LOST';
  document.getElementById('sensorText').textContent = 'SERVER LOST';
});
```

- [ ] **Step 3: `static/style.css`** — before `/* ---- Hints & log ---- */`:

```css
/* ---- Sensor strip ---- */
.sensor-summary { color: var(--text-dim); font-weight: 400; font-size: 12px; margin-left: 10px; }
.sensor-strip {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(118px, 1fr));
  gap: 8px;
}
.chip {
  background: rgba(0,0,0,0.25);
  border: 1px solid var(--accent);
  border-radius: 6px;
  padding: 6px 8px;
  font-size: 11px;
  line-height: 1.4;
  min-width: 0;
}
.chip-head {
  font-weight: 700; color: var(--text);
  white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
}
.chip-sub { color: var(--text-dim); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.chip-val { color: var(--highlight); font-weight: 700; }
.chip.offline .chip-val, .chip.missing .chip-val { color: var(--offline); }
.chip.missing { opacity: 0.55; border-style: dashed; }
.chip.unmapped { border-color: var(--highlight); }
.chip.dup { border-color: var(--stop); }
```

- [ ] **Step 4: Verify in the browser** — start the app without the Giga (`--no-valves --host 127.0.0.1`) through the preview tool. Expect: the sensor panel shows 32 dashed chips (mapped, not found), `SENSORS 0/32`, a per-port summary `COM7 0 · … · COM14 0`, no console errors. Stop the preview afterwards so the COM ports are free.

---

### Task 8: Read-only scan CLI

**Files:**
- Create: `scan_sensors.py`, `tests/test_scan_sensors.py`

**Interfaces:**
- Consumes: `finger.rs485.open_port(port, baud, timeout)`, `finger.rs485.read_sensor(ser, slave, counts_per_turn) -> Reading(slave, outcome, counts, status, velocity, error_count, t)`, `list_ch9344_ports()`, `parse_slave_ids()`, `load_all()`.
- Produces: `PortResult`, `run_port(port, ids, baud, timeout, seconds, counts_per_turn) -> PortResult`, `id_owners(results) -> Dict[int, List[str]]`, `check_ids(owners, expected_ids, expect_n) -> List[str]`, `sensor_stats(samples, mm_per_count) -> Dict[int, dict]`, `main(argv=None) -> int` (0 = no problems, 1 = problems, 2 = no ports).

- [ ] **Step 1: Write the failing test** — `tests/test_scan_sensors.py`:

```python
"""Pure parts of scan_sensors.py, plus run_port with the serial layer faked."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scan_sensors as S  # noqa: E402
from finger.rs485 import Reading  # noqa: E402


class SummaryTests(unittest.TestCase):
    def test_owners_and_problems(self):
        results = [S.PortResult("COM7", found=[50, 51]), S.PortResult("COM8", found=[51, 52])]
        owners = S.id_owners(results)
        self.assertEqual(owners, {50: ["COM7"], 51: ["COM7", "COM8"], 52: ["COM8"]})
        self.assertEqual(S.check_ids(owners, expected_ids=[50, 51, 52, 53], expect_n=3),
                         ["sensor 51 answers on COM7 and COM8", "mapped but silent: 53"])
        self.assertEqual(S.check_ids(owners, expected_ids=[50], expect_n=4),
                         ["sensor 51 answers on COM7 and COM8", "expected 4 sensors, found 3"])

    def test_clean_result(self):
        owners = {50: ["COM7"], 51: ["COM8"]}
        self.assertEqual(S.check_ids(owners, expected_ids=[50, 51], expect_n=2), [])

    def test_sensor_stats(self):
        samples = [Reading(50, "ok", counts=100), Reading(50, "ok", counts=103),
                   Reading(50, "timeout"), Reading(51, "error")]
        st = S.sensor_stats(samples, mm_per_count=0.5)
        self.assertEqual((st[50]["ok"], st[50]["timeout"], st[50]["error"]), (2, 1, 0))
        self.assertEqual((st[50]["jitter_counts"], st[50]["jitter_mm"]), (3, 1.5))
        self.assertIsNone(st[51]["jitter_counts"])


class FakeSerial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class RunPortTests(unittest.TestCase):
    def test_scan_only(self):
        ser = FakeSerial()

        def fake_read(_ser, slave, _cpt):
            return Reading(slave, "ok" if slave in (50, 52) else "timeout", counts=0)

        with mock.patch.object(S, "open_port", return_value=ser), \
                mock.patch.object(S, "read_sensor", side_effect=fake_read):
            res = S.run_port("COM7", [49, 50, 51, 52], 115200, 0.06, 0.0, 2097152)
        self.assertIsNone(res.error)
        self.assertEqual(res.found, [50, 52])
        self.assertEqual(res.sweeps, [])
        self.assertTrue(ser.closed)

    def test_open_failure_is_reported(self):
        with mock.patch.object(S, "open_port", side_effect=OSError("access denied")):
            res = S.run_port("COM7", [50], 115200, 0.06, 0.0, 2097152)
        self.assertEqual(res.error, "access denied")
        self.assertEqual(res.found, [])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run: `$PY -m unittest tests.test_scan_sensors -v`
Expected: `ModuleNotFoundError: No module named 'scan_sensors'`.

- [ ] **Step 3: Implement** — `scan_sensors.py`:

```python
#!/usr/bin/env python3
"""
scan_sensors.py

Read-only RS-485 check of the whole station: which sensor ids answer on which
EKU081 channel, and what rate each bus delivers. It never opens the Giga and
never sends a valve command. Stop app.py first: on Windows a COM port can be
open in only one program at a time.

Phase 1 scans the id range on every channel in parallel (each silent id costs
one timeout). Phase 2 then reads the found sensors back to back for --seconds
and reports reads, timeouts, errors, latency, resting jitter and sweep rate.

    python scan_sensors.py --expect 32
    python scan_sensors.py --ports COM7,COM8,COM9,COM10 --ids 50-86,173-175
    python scan_sensors.py --seconds 20 --log runs/scan.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    list_ch9344_ports,
    parse_slave_ids,
)
from finger.rs485 import Reading, open_port, read_sensor
from sensor_mapping import DEFAULT_MAPPING_PATH, load_all


@dataclass
class PortResult:
    port: str
    error: Optional[str] = None
    found: List[int] = field(default_factory=list)
    scan_s: float = 0.0
    samples: List[Reading] = field(default_factory=list)          # phase 2
    latencies: Dict[int, List[float]] = field(default_factory=dict)
    sweeps: List[float] = field(default_factory=list)             # seconds per sweep


def run_port(port: str, ids: Sequence[int], baud: int, timeout: float,
             seconds: float, counts_per_turn: int) -> PortResult:
    """Phase 1 and 2 on one channel. Never raises on bus problems."""
    res = PortResult(port)
    try:
        ser = open_port(port, baud, timeout)
    except Exception as exc:
        res.error = str(exc)
        return res
    try:
        t0 = time.monotonic()
        for sid in ids:
            # Anything but a timeout means a device answered, even if garbled.
            if read_sensor(ser, sid, counts_per_turn).outcome != "timeout":
                res.found.append(sid)
        res.scan_s = time.monotonic() - t0

        t_end = time.monotonic() + seconds
        while res.found and time.monotonic() < t_end:
            t_sweep = time.monotonic()
            for sid in res.found:
                t1 = time.monotonic()
                rd = read_sensor(ser, sid, counts_per_turn)
                res.latencies.setdefault(sid, []).append(rd.t - t1)
                res.samples.append(rd)
            res.sweeps.append(time.monotonic() - t_sweep)
    finally:
        ser.close()
    return res


def id_owners(results: Sequence[PortResult]) -> Dict[int, List[str]]:
    """Sensor id -> the channels it answered on."""
    out: Dict[int, List[str]] = {}
    for r in results:
        for sid in r.found:
            out.setdefault(sid, []).append(r.port)
    return out


def check_ids(owners: Dict[int, List[str]], expected_ids: Sequence[int],
              expect_n: Optional[int]) -> List[str]:
    """Problems that make the exit status non-zero."""
    problems = [f"sensor {sid} answers on {' and '.join(ports)}"
                for sid, ports in sorted(owners.items()) if len(ports) > 1]
    if expect_n is not None and len(owners) != expect_n:
        problems.append(f"expected {expect_n} sensors, found {len(owners)}")
    missing = sorted(set(expected_ids) - set(owners))
    if missing:
        problems.append("mapped but silent: " + ", ".join(map(str, missing)))
    return problems


def sensor_stats(samples: Sequence[Reading], mm_per_count: float) -> Dict[int, dict]:
    """Per-sensor outcome counts and resting jitter (max - min counts)."""
    per: Dict[int, dict] = {}
    for rd in samples:
        s = per.setdefault(rd.slave, {"ok": 0, "timeout": 0, "error": 0, "counts": []})
        s[rd.outcome] += 1
        if rd.ok:
            s["counts"].append(rd.counts)
    for s in per.values():
        c = s.pop("counts")
        s["jitter_counts"] = (max(c) - min(c)) if c else None
        s["jitter_mm"] = None if s["jitter_counts"] is None else s["jitter_counts"] * mm_per_count
    return per


def _port_list(value: str) -> List[str]:
    return [p.strip() for p in value.split(",") if p.strip()]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Read-only scan of the station's RS-485 sensors (no valve I/O).")
    ap.add_argument("--ports", type=_port_list, default=None,
                    help="comma list of channels (default: every EKU081 channel)")
    ap.add_argument("--ids", type=parse_slave_ids, default=parse_slave_ids("1-247"),
                    help="ids to scan (default 1-247)")
    ap.add_argument("--expect", type=int, default=None,
                    help="number of sensors that should answer")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="phase-2 back-to-back reading time per channel (s)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.06, help="per-read timeout (s)")
    ap.add_argument("--mapping", default=DEFAULT_MAPPING_PATH)
    ap.add_argument("--log", help="write every phase-2 sample to this CSV")
    args = ap.parse_args(argv)

    ports = args.ports or list_ch9344_ports()
    if not ports:
        print("[ERR] no EKU081 channels found; pass --ports", file=sys.stderr)
        return 2
    _, index = load_all(args.mapping)
    mm_per_count = math.pi * DEFAULT_DRUM_DIAMETER_MM / DEFAULT_COUNTS_PER_TURN

    print(f"channels {', '.join(ports)} @ {args.baud} 8N1, timeout {args.timeout}s")
    print(f"phase 1: {len(args.ids)} ids ({args.ids[0]}..{args.ids[-1]}), up to "
          f"{len(args.ids) * args.timeout:.0f} s; phase 2: {args.seconds:g} s")

    results: List[Optional[PortResult]] = [None] * len(ports)

    def work(i: int, port: str) -> None:
        try:
            results[i] = run_port(port, args.ids, args.baud, args.timeout, args.seconds,
                                  DEFAULT_COUNTS_PER_TURN)
        except Exception as exc:          # e.g. the adapter unplugged mid-scan
            results[i] = PortResult(port, error=f"{type(exc).__name__}: {exc}")

    threads = [threading.Thread(target=work, args=(i, p)) for i, p in enumerate(ports)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    def name(sid: int) -> str:
        return index[sid]["muscle"] if sid in index else "unmapped"

    print("\n== ids per channel ==")
    for r in results:
        if r.error:
            print(f"{r.port}: open failed: {r.error}")
            continue
        listing = ", ".join(f"{sid} {name(sid)}" for sid in r.found) or "-"
        print(f"{r.port}: {len(r.found)} in {r.scan_s:.1f} s -> {listing}")

    ms = lambda s: f"{s * 1e3:6.1f}"  # noqa: E731
    print("\n== bus timing (phase 2) ==")
    print(f"{'chan':6} {'id':>3} {'muscle':14} {'reads':>5} {'ok':>5} {'t/o':>4} {'err':>4} "
          f"{'lat med':>8} {'max':>7}  {'jitter':>6} {'= mm':>7}")
    for r in results:
        if r.error or not r.found:
            continue
        stats = sensor_stats(r.samples, mm_per_count)
        for sid in r.found:
            s = stats.get(sid, {"ok": 0, "timeout": 0, "error": 0,
                                "jitter_counts": None, "jitter_mm": None})
            lat = r.latencies.get(sid) or [0.0]
            n = s["ok"] + s["timeout"] + s["error"]
            jc = "-" if s["jitter_counts"] is None else s["jitter_counts"]
            jm = "-" if s["jitter_mm"] is None else f"{s['jitter_mm']:.4f}"
            print(f"{r.port:6} {sid:3d} {name(sid)[:14]:14} {n:5d} {s['ok']:5d} "
                  f"{s['timeout']:4d} {s['error']:4d} {ms(statistics.median(lat)):>8} "
                  f"{ms(max(lat)):>7}  {jc:>6} {jm:>7}")
        if r.sweeps:
            print(f"{r.port}: {len(r.sweeps)} sweeps of {len(r.found)} -> "
                  f"{len(r.sweeps) / sum(r.sweeps):.1f} Hz per sensor; sweep med "
                  f"{ms(statistics.median(r.sweeps))} ms, max {ms(max(r.sweeps))} ms")

    if args.log:
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        with open(args.log, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["channel", "t_monotonic", "sensor", "muscle", "outcome",
                        "abs_counts", "status", "error_count"])
            for r in results:
                for rd in r.samples:
                    w.writerow([r.port, rd.t, rd.slave, name(rd.slave), rd.outcome,
                                rd.counts, rd.status, rd.error_count])
        print(f"\nlogged phase-2 samples to {args.log}")

    owners = id_owners(results)
    problems = check_ids(owners, [s for s in index if s in args.ids], args.expect)
    print(f"\n== result: {len(owners)} sensor(s) on {len(ports)} channel(s) ==")
    for p in problems:
        print(f"  [!] {p}")
    if not problems:
        print("  OK")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m unittest tests.test_scan_sensors -v` → 5 tests OK.
Hardware smoke (sensors not yet on the bus): `$PY scan_sensors.py --ports COM7,COM8,COM9,COM10 --ids 50-86,173-175 --seconds 1` → every channel `0 in ~2.4 s`, result lists `mapped but silent: 50, …`, exit status 1. No app may be holding the ports.

---

### Task 9: README and full verification

**Files:**
- Modify: `README.md`

- [ ] **Step 1: README** — replace the top branch note with a `hand_win` note, and add a "Windows (hand_win)" section before `## Install`:

```markdown
> **Branch `hand_win`.** The full hand on Windows: the 32-regulator Giga plus
> 32 draw-wire sensors split over the EKU081's channels, read in parallel.
> Linux still works. Design: `docs/superpowers/specs/2026-09-29-hand-win-system-bringup-design.md`.

## Windows (hand_win)

1. **EKU081 driver.** Install WCH's `USBMSER.exe` (CH9344/CH348 multi-port
   driver): https://www.wch.cn/downloads/USBMSER_EXE.html. The channels appear
   as `WCH USB-SERIAL Ch A (COMn)` … `Ch H`; the app finds them by that name
   (the driver hides the USB VID from pyserial).
2. **Python.** `C:\Users\14747\.conda\envs\py313\python.exe -m pip install -r requirements.txt`.
3. **One program per COM port.** Windows will not share a port: close the
   Arduino serial monitor, and stop `app.py` before running `scan_sensors.py`.
4. **Check the sensors first**, read-only (no valve I/O):

   ```
   python scan_sensors.py --expect 32
   python scan_sensors.py --ports COM7,COM8,COM9,COM10 --ids 50-86,173-175
   ```

5. **Run the app.** `python app.py --max-kpa 50` for a first air-on session.
   `--rs485-ports COM7,COM8,COM9,COM10` limits it to channels A–D; by default
   every channel is opened and the empty ones go idle.

`sensor_mapping.json` lists every muscle's regulator and sensor
(`"regulator": null` = sensor only, `"sensor": null` = regulator only). The app
scans exactly the mapped sensor ids unless `--ids` is given; RESCAN reloads
the file and re-sweeps.
```

Update the Options table: replace the `--rs485-port` row with `--rs485-ports` (`auto` = every EKU081 channel), add `--rs485-port` (single-port alias), `--max-kpa` (`140`, session cap, may only lower it), and change `--ids` default to `mapping` ("the sensor ids in `sensor_mapping.json`"). Add `scan_sensors.py`, `sensor_hub.py` and `tests/` to the layout block.

- [ ] **Step 2: Full verification**

Run: `$PY -m unittest discover -s tests -t .` → OK.
Run: `$PY -m unittest discover -s finger -p "test_*.py" -t .` → OK (skipped=1).
Run: `git status` → only the intended files changed; nothing committed.
