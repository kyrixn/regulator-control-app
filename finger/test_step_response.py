"""Hardware-free tests for finger/step_response.py.
Run: .venv/bin/python -m unittest finger.test_step_response"""

import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from finger import config as C  # noqa: E402
from finger import step_response as S  # noqa: E402

WIRING = {"ED": (18, 63), "FDS": (17, 72), "FDP": (16, 73), "DI": (19, 70), "PI": (20, 71)}


def cfg():
    with open(C.EXAMPLE_CONFIG_PATH, encoding="utf-8") as fh:
        raw = json.load(fh)
    for r in C.ROLES:
        raw["muscles"][r]["sign"] = -1
        raw["muscles"][r]["reference_counts"] = 0
    return C.parse(raw, {r: {"regulator": a, "sensor": b} for r, (a, b) in WIRING.items()})


def synthetic(role="FDP", idle=10.0, levels=(20.0, 40.0), gain=0.05, tau=0.4, hz=40.0,
              hold=5.0, rest=5.0, settle=3.0, residual_frac=0.1):
    """First-order response with a small unloading residual; others static."""
    rows, t, dt = [], 0.0, 1.0 / hz
    y0 = 0.0

    def row(phase, level, cmd, y):
        mm = {r: 0.0 for r in C.ROLES}
        mm[role] = y
        return S.Row(t, phase, level, {r: (cmd if r == role else idle) for r in C.ROLES},
                     mm, {r: 0 for r in C.ROLES}, {r: True for r in C.ROLES})

    while t < settle:
        rows.append(row("idle", None, idle, y0)); t += dt
    for lv in levels:
        target = y0 + gain * (lv - idle)
        y, t_step = y0, t
        while t < t_step + hold:
            y = target + (y0 - target) * math.exp(-(t - t_step) / tau)
            rows.append(row(f"step_c0_{lv:g}", lv, lv, y)); t += dt
        y_hi, t_rest = y, t
        back = y0 + residual_frac * (y_hi - y0)
        while t < t_rest + rest:
            y = back + (y_hi - back) * math.exp(-(t - t_rest) / tau)
            rows.append(row(f"rest_c0_{lv:g}", lv, idle, y)); t += dt
        y0 = back
    return rows


class PlanTests(unittest.TestCase):
    def test_default_levels_and_duration(self):
        p = S.make_plan(cfg(), "FDP", None, 10, 5, 5, 4, 0, 1)
        self.assertEqual(p.levels, [20.0, 40.0, 60.0, 80.0])
        self.assertEqual(p.duration(), 4 + 4 * 10)
        p = S.make_plan(cfg(), "DI", None, 10, 5, 5, 4, 0, 2)
        self.assertEqual(p.levels, [31.2, 62.5, 93.8, 125.0])

    def test_rejections(self):
        c = cfg()
        with self.assertRaisesRegex(ValueError, "unknown role"):
            S.make_plan(c, "FCU", None, 10, 5, 5, 4, 0, 1)
        with self.assertRaisesRegex(ValueError, "ceiling"):
            S.make_plan(c, "FDP", [50, 90], 10, 5, 5, 4, 0, 1)
        with self.assertRaisesRegex(ValueError, "idle 90 kPa exceeds ED"):
            S.make_plan(c, "DI", [100], 90, 5, 5, 4, 0, 1)
        with self.assertRaisesRegex(ValueError, "hold"):
            S.make_plan(c, "FDP", None, 10, 0.1, 5, 4, 0, 1)
        with self.assertRaisesRegex(ValueError, "cycles"):
            S.make_plan(c, "FDP", None, 10, 5, 5, 4, 0, 0)


class AnalysisTests(unittest.TestCase):
    def test_recovers_gain_tau_and_residual(self):
        rows = synthetic(gain=0.05, tau=0.4)
        res = S.analyze(rows, "FDP", 10.0)
        self.assertEqual([r.level for r in res], [20.0, 40.0])
        for r in res:
            self.assertAlmostEqual(r.gain_mm_per_kpa, 0.05, places=3)
            self.assertAlmostEqual(r.t63_s, 0.4, delta=0.06)
            self.assertAlmostEqual(r.residual_mm, 0.1 * r.delta_mm, places=3)
            self.assertTrue(all(v == 0 for v in r.coupling_mm.values()))
        self.assertGreater(res[1].delta_mm, res[0].delta_mm)

    def test_csv_roundtrip_columns(self):
        rows = synthetic(levels=(20.0,))
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.csv")
            S.write_csv(rows, p)
            with open(p) as fh:
                header = fh.readline().strip().split(",")
                n = sum(1 for _ in fh)
        self.assertEqual(n, len(rows))
        self.assertIn("len_FDP_mm", header)
        self.assertIn("cmd_FDP_kpa", header)
        self.assertEqual(len(header), 3 + 4 * 5)

    def test_incomplete_rows_do_not_crash(self):
        self.assertEqual(S.analyze([], "FDP", 10.0), [])
        rows = synthetic(levels=(20.0,))[:5]  # only idle rows
        self.assertEqual(S.analyze(rows, "FDP", 10.0), [])


if __name__ == "__main__":
    unittest.main()
