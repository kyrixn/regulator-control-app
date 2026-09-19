"""Unit tests for finger/config.py. Run: .venv/bin/python -m unittest finger.test_config"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import config as C  # noqa: E402

WIRING = {"ED": (18, 63), "FDS": (17, 1), "FDP": (16, 65), "DI": (19, 54), "PI": (20, 69)}


def example():
    with open(C.EXAMPLE_CONFIG_PATH, encoding="utf-8") as fh:
        return json.load(fh)


def mapping():
    return {r: {"regulator": reg, "sensor": sen} for r, (reg, sen) in WIRING.items()}


def write_mapping(path, entries):
    with open(path, "w") as fh:
        json.dump({"muscles": entries}, fh)


def good_entries():
    return [{"muscle": r, "regulator": reg, "sensor": sen} for r, (reg, sen) in WIRING.items()]


class ParseTests(unittest.TestCase):
    def test_example_parses_but_is_not_armable(self):
        cfg = C.parse(example(), mapping())
        self.assertEqual(cfg.sensors, [63, 1, 65, 54, 69])
        self.assertEqual(cfg.regulators, [18, 17, 16, 19, 20])
        self.assertEqual(cfg.role_of_sensor(54), "DI")
        self.assertIsNone(cfg.role_of_sensor(2))
        self.assertEqual(len(cfg.arm_problems()), 10)
        with self.assertRaises(C.ConfigError):
            cfg.require_armable()
        with self.assertRaises(C.ConfigError):
            cfg.contraction_mm("ED", 0)

    def test_example_loads_against_the_real_mapping(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "finger.local.json")
            with open(p, "w") as fh:
                json.dump(example(), fh)
            cfg = C.load(p)  # default mapping path = ../sensor_mapping.json
            self.assertEqual(cfg.mapping_path, C.DEFAULT_MAPPING_PATH)
            self.assertEqual(len(cfg.muscles), 5)

    def test_cap_matches_valve_layer(self):
        from valve_controller import MAX_INPUT_KPA
        self.assertEqual(C.APP_CAP_KPA, MAX_INPUT_KPA)

    def test_subset_arming(self):
        raw = example()
        for r in ("ED", "FDS", "FDP"):
            raw["muscles"][r]["sign"] = -1
            raw["muscles"][r]["reference_counts"] = 0
        cfg = C.parse(raw, mapping())
        self.assertEqual(cfg.calibrated_roles, ["ED", "FDS", "FDP"])
        cfg.require_armable(["ED", "FDS", "FDP"])
        with self.assertRaisesRegex(C.ConfigError, "DI: encoder sign"):
            cfg.require_armable(["FDP", "DI"])
        with self.assertRaisesRegex(C.ConfigError, "PI"):
            cfg.require_armable()
        self.assertEqual(cfg.arm_problems(["FCU"]), ["FCU: unknown role"])

    def test_calibrated_config_arms_and_converts(self):
        raw = example()
        for r in C.ROLES:
            raw["muscles"][r]["sign"] = -1
            raw["muscles"][r]["reference_counts"] = 1000
        cfg = C.parse(raw, mapping()).require_armable()
        # -1 sign: counts rising means the muscle paid out (negative contraction)
        self.assertAlmostEqual(cfg.contraction_mm("ED", 1000 + cfg.counts_per_turn),
                               -3.141592653589793 * 14.0)

    def rejects(self, mutate, needle):
        raw = example()
        mutate(raw)
        with self.assertRaisesRegex(C.ConfigError, needle):
            C.parse(raw, mapping())

    def test_rejections(self):
        self.rejects(lambda r: r.update(version=2), "version")
        self.rejects(lambda r: r["muscles"].pop("PI"), "missing roles")
        self.rejects(lambda r: r["muscles"].update(FCU={}), "unknown roles")
        self.rejects(lambda r: r["muscles"]["ED"].update(regulator=18), "belong in sensor_mapping")
        self.rejects(lambda r: r["muscles"]["ED"].update(sensor=63), "belong in sensor_mapping")
        self.rejects(lambda r: r["muscles"]["ED"].update(ceiling_kpa=150), "outside")
        self.rejects(lambda r: r["muscles"]["ED"].update(idle_kpa=90), "idle_kpa: 90.*outside")
        self.rejects(lambda r: r["muscles"]["ED"].pop("idle_kpa"), "idle_kpa")
        self.rejects(lambda r: r["muscles"]["ED"].update(ceiling_kpa="80"), "finite number")
        self.rejects(lambda r: r["muscles"]["ED"].update(sign=2), "sign")
        self.rejects(lambda r: r["muscles"]["ED"].update(reference_counts=1.5), "integer or null")
        self.rejects(lambda r: r["ports"].update(rs485=r["ports"]["valve"]), "must differ")
        self.rejects(lambda r: r["ports"].update(valve=""), "non-empty")


class MappingTests(unittest.TestCase):
    def check(self, entries, needle):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "sensor_mapping.json")
            write_mapping(p, entries)
            with self.assertRaisesRegex(C.ConfigError, needle):
                C.read_mapping(p)

    def test_good_mapping_ignores_other_muscles(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "sensor_mapping.json")
            write_mapping(p, good_entries() + [{"muscle": "FCU", "regulator": 3, "sensor": 58}])
            self.assertEqual(C.read_mapping(p), mapping())

    def test_bad_mappings(self):
        e = good_entries()
        self.check(e[:4], r"no entry for roles \['PI'\]")
        self.check(e + [e[0]], "appears more than once")
        e = good_entries(); e[1]["regulator"] = 18
        self.check(e, "duplicate regulator 18")
        e = good_entries(); e[1]["sensor"] = 63
        self.check(e, "duplicate sensor 63")
        e = good_entries(); e[0]["sensor"] = None
        self.check(e, "expected an integer")
        e = good_entries(); e[0]["regulator"] = 32
        self.check(e, "outside")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "sensor_mapping.json")
            with self.assertRaisesRegex(C.ConfigError, "not found"):
                C.read_mapping(p)
            with open(p, "w") as fh:
                fh.write('{"muscles": [ {"muscle": "PI"}, ]}')  # trailing comma
            with self.assertRaisesRegex(C.ConfigError, "not valid JSON"):
                C.read_mapping(p)


class LoadSaveTests(unittest.TestCase):
    def test_load_save_roundtrip_and_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            m = os.path.join(d, "sensor_mapping.json")
            write_mapping(m, good_entries())
            p = os.path.join(d, "finger.local.json")
            with self.assertRaisesRegex(C.ConfigError, "no local config"):
                C.load(p, m)
            with open(p, "w") as fh:
                json.dump(example(), fh)
            cfg = C.load(p, m)
            cfg.muscles["FDP"].sign = 1
            cfg.muscles["FDP"].reference_counts = 123456
            C.save(cfg)
            with open(p) as fh:
                saved = json.load(fh)
            self.assertNotIn("regulator", saved["muscles"]["FDP"])  # wiring is not duplicated
            self.assertEqual(saved["muscles"]["FDP"]["idle_kpa"], 20)
            again = C.load(p, m)
            self.assertEqual(again.muscles["FDP"].sign, 1)
            self.assertEqual(again.muscles["FDP"].reference_counts, 123456)
            self.assertEqual(again.muscles["FDP"].regulator, 16)
            self.assertIsNone(again.muscles["ED"].sign)
            with open(p, "w") as fh:
                fh.write("{ not json")
            with self.assertRaisesRegex(C.ConfigError, "not valid JSON"):
                C.load(p, m)


if __name__ == "__main__":
    unittest.main()
