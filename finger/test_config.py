"""Unit tests for finger/config.py. Run: .venv/bin/python -m unittest finger.test_config"""

import copy
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import config as C  # noqa: E402


def example():
    with open(C.EXAMPLE_CONFIG_PATH, encoding="utf-8") as fh:
        return json.load(fh)


class ParseTests(unittest.TestCase):
    def test_example_parses_but_is_not_armable(self):
        cfg = C.parse(example())
        self.assertEqual(cfg.sensors, [63, 1, 65, 54, 69])
        self.assertEqual(cfg.regulators, [18, 17, 16, 19, 20])
        self.assertEqual(cfg.role_of_sensor(54), "DI")
        self.assertIsNone(cfg.role_of_sensor(2))
        self.assertEqual(len(cfg.arm_problems()), 10)
        with self.assertRaises(C.ConfigError):
            cfg.require_armable()
        with self.assertRaises(C.ConfigError):
            cfg.contraction_mm("ED", 0)

    def test_example_matches_sensor_mapping(self):
        C.check_against_mapping(C.parse(example()))

    def test_cap_matches_valve_layer(self):
        from valve_controller import MAX_INPUT_KPA
        self.assertEqual(C.APP_CAP_KPA, MAX_INPUT_KPA)

    def test_calibrated_config_arms_and_converts(self):
        raw = example()
        for r in C.ROLES:
            raw["muscles"][r]["sign"] = -1
            raw["muscles"][r]["reference_counts"] = 1000
        cfg = C.parse(raw).require_armable()
        # -1 sign: counts rising means the muscle paid out (negative contraction)
        self.assertAlmostEqual(cfg.contraction_mm("ED", 1000 + cfg.counts_per_turn),
                               -3.141592653589793 * 14.0)

    def rejects(self, mutate, needle):
        raw = example()
        mutate(raw)
        with self.assertRaisesRegex(C.ConfigError, needle):
            C.parse(raw)

    def test_rejections(self):
        self.rejects(lambda r: r.update(version=2), "version")
        self.rejects(lambda r: r["muscles"].pop("PI"), "missing roles")
        self.rejects(lambda r: r["muscles"].update(FCU={}), "unknown roles")
        self.rejects(lambda r: r["muscles"]["PI"].update(regulator=19), "duplicate regulator")
        self.rejects(lambda r: r["muscles"]["PI"].update(sensor=63), "duplicate sensor")
        self.rejects(lambda r: r["muscles"]["ED"].update(regulator=32), "outside")
        self.rejects(lambda r: r["muscles"]["ED"].update(sensor=0), "outside")
        self.rejects(lambda r: r["muscles"]["ED"].update(ceiling_kpa=150), "outside")
        self.rejects(lambda r: r["muscles"]["ED"].update(ceiling_kpa="80"), "finite number")
        self.rejects(lambda r: r["muscles"]["ED"].update(sign=2), "sign")
        self.rejects(lambda r: r["muscles"]["ED"].update(reference_counts=1.5), "integer or null")
        self.rejects(lambda r: r["ports"].update(rs485=r["ports"]["valve"]), "must differ")
        self.rejects(lambda r: r["ports"].update(valve=""), "non-empty")

    def test_mapping_mismatch_is_refused(self):
        cfg = C.parse(example())
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "sensor_mapping.json")
            with open(p, "w") as fh:
                json.dump({"muscles": [{"muscle": r, "regulator": cfg.muscles[r].regulator,
                                        "sensor": cfg.muscles[r].sensor} for r in C.ROLES]}, fh)
            C.check_against_mapping(cfg, p)
            with open(p, "w") as fh:
                json.dump({"muscles": [{"muscle": "DI", "regulator": 19, "sensor": 57}]}, fh)
            with self.assertRaisesRegex(C.ConfigError, "not present|says"):
                C.check_against_mapping(cfg, p)

    def test_load_save_roundtrip_and_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "finger.local.json")
            with self.assertRaisesRegex(C.ConfigError, "no local config"):
                C.load(p, mapping_path=None)
            with open(p, "w") as fh:
                json.dump(example(), fh)
            cfg = C.load(p, mapping_path=None)
            cfg.muscles["FDP"].sign = 1
            cfg.muscles["FDP"].reference_counts = 123456
            C.save(cfg)
            again = C.load(p, mapping_path=None)
            self.assertEqual(again.muscles["FDP"].sign, 1)
            self.assertEqual(again.muscles["FDP"].reference_counts, 123456)
            self.assertIsNone(again.muscles["ED"].sign)
            with open(p, "w") as fh:
                fh.write("{ not json")
            with self.assertRaisesRegex(C.ConfigError, "not valid JSON"):
                C.load(p, mapping_path=None)


if __name__ == "__main__":
    unittest.main()
