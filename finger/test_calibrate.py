"""Hardware-free tests for finger/calibrate.py. Run: .venv/bin/python -m unittest finger.test_calibrate"""

import json
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import calibrate as K  # noqa: E402
from finger import config as C  # noqa: E402
from finger.rs485 import Reading  # noqa: E402


class FakeBus:
    """Returns scripted counts per slave; `offline` slaves time out."""

    def __init__(self, counts, offline=(), noise=0):
        self.counts = dict(counts)
        self.offline = set(offline)
        self.noise = noise
        self._i = 0

    def read(self, slave):
        if slave in self.offline:
            return Reading(slave, "timeout")
        self._i += 1
        jitter = (self._i % 3 - 1) * self.noise
        return Reading(slave, "ok", counts=self.counts[slave] + jitter, status=0)

    def close(self):
        pass


def make_cfg(tmpdir):
    with open(C.EXAMPLE_CONFIG_PATH, encoding="utf-8") as fh:
        raw = json.load(fh)
    p = os.path.join(tmpdir, "finger.local.json")
    with open(p, "w") as fh:
        json.dump(raw, fh)
    return C.load(p, mapping_path=None)


class HelperTests(unittest.TestCase):
    def test_decide_sign(self):
        self.assertEqual(K.decide_sign(+5000, 1000), 1)
        self.assertEqual(K.decide_sign(-5000, 1000), -1)
        self.assertIsNone(K.decide_sign(+500, 1000))
        self.assertIsNone(K.decide_sign(0, 1))

    def test_summarize(self):
        s = K.summarize([10, 12, 11])
        self.assertEqual((s["mean"], s["span"], s["n"]), (11, 2, 3))


class CaptureTests(unittest.TestCase):
    def test_capture_rejects_offline_and_moving(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = make_cfg(d)
            bus = FakeBus({63: 100, 1: 200, 65: 300, 54: 400, 69: 500}, offline={54})
            with self.assertRaisesRegex(RuntimeError, r"DI \(id 54\).*good reads"):
                K.capture(bus, cfg, list(C.ROLES), 0.05, 100, "test")
            noisy = FakeBus({63: 100}, noise=5000)  # ±5000 counts ≈ ±0.1 mm
            with self.assertRaisesRegex(RuntimeError, "ED: moved"):
                K.capture(noisy, cfg, ["ED"], 0.05, 100, "test")
            quiet = FakeBus({63: 100}, noise=10)
            st = K.capture(quiet, cfg, ["ED"], 0.05, 100, "test")
            self.assertAlmostEqual(st["ED"]["mean"], 100, delta=10)


class ModeTests(unittest.TestCase):
    def test_reference_saves_means(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = make_cfg(d)
            bus = FakeBus({63: 1000, 1: 2000, 65: 3000, 54: 4000, 69: 5000})
            rc = K.mode_reference(bus, cfg, list(C.ROLES), 0.05, 0.05, yes=True)
            self.assertEqual(rc, 0)
            again = C.load(cfg.path, mapping_path=None)
            self.assertEqual([again.muscles[r].reference_counts for r in C.ROLES],
                             [1000, 2000, 3000, 4000, 5000])
            self.assertTrue(all(again.muscles[r].sign is None for r in C.ROLES))

    def test_sign_records_direction_and_rejects_small_moves(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = make_cfg(d)
            bus = FakeBus({63: 100000, 54: 100000})
            per_mm = 1 / cfg.mm_per_count
            # ED: shortening makes counts fall by 2 mm  -> sign -1
            # DI: shortening makes counts rise by 0.1 mm -> too small, not recorded
            moves = iter([("ED", -2), ("DI", +0.1)])

            def fake_input(prompt):
                if "SHORTEN" in prompt:
                    role, mm = next(moves)
                    bus.counts[cfg.muscles[role].sensor] += int(mm * per_mm)
                return ""

            with mock.patch("builtins.input", side_effect=fake_input):
                rc = K.mode_sign(bus, cfg, ["ED", "DI"], 0.05, 0.05, 0.5)
            self.assertEqual(rc, 0)
            again = C.load(cfg.path, mapping_path=None)
            self.assertEqual(again.muscles["ED"].sign, -1)
            self.assertIsNone(again.muscles["DI"].sign)
            # contraction is positive for a shortening move with the stored sign
            again.muscles["ED"].reference_counts = 100000
            self.assertGreater(again.contraction_mm("ED", bus.counts[63]), 0)

    def test_parse_roles(self):
        self.assertEqual(K.parse_roles(["di", "PI", "di"]), ["DI", "PI"])
        with self.assertRaises(SystemExit):
            K.parse_roles(["FCU"])


if __name__ == "__main__":
    unittest.main()
