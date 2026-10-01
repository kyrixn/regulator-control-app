"""Unit tests for sensor_mapping.py and the station's sensor_mapping.json."""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sensor_mapping as M  # noqa: E402

ENTRIES = [
    {"muscle": "FDP1", "regulator": 8, "sensor": 50, "region": "forearm"},
    {"muscle": "FDP4", "regulator": None, "sensor": 53, "region": "forearm"},
    {"muscle": "FPL", "regulator": 11, "sensor": None, "note": "sensor 58 not connected yet"},
    {"muscle": "brachialis", "regulator": 31, "sensor": 175, "region": 7},
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
            8: {"muscle": "FDP1", "sensor": 50, "region": "forearm"},
            11: {"muscle": "FPL", "sensor": None, "region": None},
            31: {"muscle": "brachialis", "sensor": 175, "region": None},   # non-string region
            5: {"muscle": "dup", "sensor": 50, "region": None},
        })

    def test_by_sensor_keeps_sensor_only_entries_and_first_duplicate(self):
        with tempfile.TemporaryDirectory() as d:
            _, by_sensor = M.load_all(write(d, ENTRIES))
        self.assertEqual(by_sensor, {
            50: {"muscle": "FDP1", "regulator": 8, "region": "forearm"},
            53: {"muscle": "FDP4", "regulator": None, "region": "forearm"},
            175: {"muscle": "brachialis", "regulator": 31, "region": None},
        })

    def test_load_mapping_matches_load_all(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, ENTRIES)
            self.assertEqual(M.load_mapping(p), M.load_all(p)[0])

    def test_missing_file_is_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(M.load_all(os.path.join(d, "none.json")), ({}, {}))


class StationMappingTests(unittest.TestCase):
    """Pins the transcription of bionic_hand/muscle_id.xlsx (2026-09-29), minus
    the upper-arm muscles dropped on 2026-09-30."""

    def setUp(self):
        self.by_reg, self.by_sensor = M.load_all(M.DEFAULT_MAPPING_PATH)

    def test_counts(self):
        self.assertEqual(len(self.by_sensor), 29)
        self.assertEqual(len(self.by_reg), 25)

    def test_sensor_ids(self):
        expected = (set(range(50, 58)) | {60} | set(range(61, 71)) | set(range(75, 80))
                    | set(range(81, 85)) | {86})
        self.assertEqual(set(self.by_sensor), expected)

    def test_unused_regulators(self):
        self.assertEqual(set(range(32)) - set(self.by_reg), {0, 7, 19, 28, 29, 30, 31})

    def test_spot_checks(self):
        self.assertEqual(self.by_reg[8], {"muscle": "FDP1", "sensor": 50, "region": "forearm"})
        self.assertEqual(self.by_reg[11], {"muscle": "FPL", "sensor": None, "region": "forearm"})
        self.assertEqual(self.by_sensor[53], {"muscle": "FDP4", "regulator": None,
                                              "region": "forearm"})
        self.assertEqual(self.by_sensor[86], {"muscle": "OP", "regulator": 27, "region": "palm"})

    def test_regions(self):
        """Palmar (intrinsic) muscles are the xlsx's "hand" group; the rest are forearm."""
        palm = {s for s, e in self.by_sensor.items() if e["region"] == "palm"}
        self.assertEqual(palm, set(range(75, 80)) | set(range(81, 85)) | {86})
        self.assertEqual({e["region"] for e in self.by_sensor.values()}, {"palm", "forearm"})
        self.assertEqual({r for r, e in self.by_reg.items() if e["region"] == "palm"},
                         set(range(20, 28)))


if __name__ == "__main__":
    unittest.main()
