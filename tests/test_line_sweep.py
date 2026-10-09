"""Line mode of pair_sweep (a muscle group along one coordinate s) and its
analysis, on synthetic data."""

import contextlib
import io
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiments import analyze_sweep as A  # noqa: E402
from experiments import pair_sweep as P  # noqa: E402

WRIST = {"FCR": (20.0, 60.0), "FCU": (10.0, 50.0), "ECR": (30.0, 10.0), "ECU": (20.0, 10.0)}


class LineProtocolTests(unittest.TestCase):
    def test_pressures_follow_the_line(self):
        self.assertEqual(P.line_pressures(WRIST, -1), {"FCR": 20, "FCU": 10, "ECR": 30, "ECU": 20})
        self.assertEqual(P.line_pressures(WRIST, 1), {"FCR": 60, "FCU": 50, "ECR": 10, "ECU": 10})
        self.assertEqual(P.line_pressures(WRIST, 0), {"FCR": 40, "FCU": 30, "ECR": 20, "ECU": 15})

    def test_schedule_turns(self):
        self.assertEqual(P.schedule_turns([(0.6, 2), (0.4, 1)], centre=0.1),
                         [(0.1 - 0.6, 0.7), (0.1 - 0.6, 0.7), (0.1 - 0.4, 0.5)])

    def test_segments(self):
        segs = P.line_sweep_segments(WRIST, [(0.6, 2), (0.4, 1)], 1.0, {"Sup": 60})
        self.assertEqual(len(segs), 2 + 3 * 8 + 1)
        by = {s.label: s.targets for s in segs}
        self.assertEqual(by["inflate"], {**P.line_pressures(WRIST, 0), "Sup": 60})
        self.assertEqual(by["c1 hold +"], {**P.line_pressures(WRIST, 0.6), "Sup": 60})
        self.assertEqual(by["c3 hold -"], {**P.line_pressures(WRIST, -0.4), "Sup": 60})
        self.assertEqual(by["c2 zero from +"], by["inflate"])
        for s in segs[:-1]:
            self.assertEqual(s.targets["Sup"], 60)
        self.assertEqual(set(segs[-1].targets.values()), {0.0})

    def test_centre_offset(self):
        segs = P.line_sweep_segments(WRIST, [(0.5, 1)], 1.0, {}, centre=0.2)
        by = {s.label: s.targets for s in segs}
        self.assertEqual(by["c1 zero from -"], P.line_pressures(WRIST, 0.2))
        self.assertEqual(by["c1 hold +"], P.line_pressures(WRIST, 0.7))

    def test_rejects(self):
        for kwargs in ({"schedule": []}, {"schedule": [(0, 2)]}, {"schedule": [(0.5, 0)]},
                       {"schedule": [(3.5, 1)]},              # ECU below 0 at s = +3.5
                       {"holds": {"FCR": 30}}):
            args = {"line": WRIST, "schedule": [(0.5, 1)], "rate": 1.0, "holds": {}, **kwargs}
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                P.line_sweep_segments(**args)

    def test_parsers_and_tag(self):
        self.assertEqual(P.parse_line("FCR=20:60, ECR=30:10"),
                         {"FCR": (20.0, 60.0), "ECR": (30.0, 10.0)})
        self.assertEqual(P.parse_schedule("0.6:6,0.4:3"), [(0.6, 6), (0.4, 3)])
        for bad in ("FCR=20", "FCR=20:60", "=20:60,ECR=1:2", "FCR=a:60,ECR=1:2"):
            with self.subTest(bad=bad), self.assertRaises(Exception):
                P.parse_line(bad)
        with self.assertRaises(Exception):
            P.parse_schedule("0.6")
        self.assertEqual(P.run_tag({"mass_g": 0.0, "station": None, "session": "d1",
                                    "mount": "m 1"}), "_0g_d1_m-1")
        self.assertEqual(P.run_tag({"mass_g": None}), "")

    def test_plan_only_opens_nothing(self):
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out:
            rc = P.main(["--line", "FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10",
                         "--cycles-at", "0.6:2", "--hold", "Sup=60", "--max-kpa", "70",
                         "--mass-g", "0", "--plan", "--out", d])
            self.assertEqual(os.listdir(d), [])
        self.assertEqual(rc, 0)
        self.assertIn("PLAN ONLY", out.getvalue())

    def test_unmapped_regulators_can_be_held(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = P.main(["--line", "FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10",
                         "--cycles-at", "0.6:1", "--hold", "Sup=60,V28=60,V30=60,V31=60",
                         "--max-kpa", "70", "--plan"])
        self.assertEqual(rc, 0)
        for reg in ("V28", "V30", "V31"):
            self.assertIn(f"{reg:4} {reg}", out.getvalue())

    def test_a_regulator_cannot_be_driven_twice(self):
        for hold in ("V4=10", "V40=10", "XYZ=10"):          # V4 is FCR; no V40; no XYZ
            with self.subTest(hold=hold), contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                P.main(["--line", "FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10",
                        "--cycles-at", "0.6:1", "--hold", hold, "--max-kpa", "70", "--plan"])

    def test_plan_respects_the_cap(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            P.main(["--line", "FCR=20:60,FCU=10:50,ECR=30:10,ECU=20:10",
                    "--cycles-at", "0.8:1", "--plan"])           # FCR 56 > default 50


SLOPES = {"FCR": -6.0, "FCU": -5.0, "ECR": 8.0, "ECU": 4.0}      # mm per unit of s


def simulate_line_run(path, half_width=0.1, stop_hi=0.55, stop_lo=-0.9,
                      schedule=((0.5, 3), (0.3, 2), (0.8, 2)), drift_mm=0.6, decay=0.3):
    """A whole line-mode wrist run through the real Recorder: a play operator of
    `half_width` in s (the joint stays put for 2 * half_width after a turn), the
    joint clamped to [stop_lo, stop_hi], each length SLOPES[n] * joint plus a
    settling offset that shrinks by `decay` each cycle."""
    from station import Recorder

    regs = {"FCR": (4, 68), "FCU": (3, 67), "ECR": (6, 70), "ECU": (5, 69), "Sup": (2, 66)}
    muscles = {n: {"regulator": r, "sensor": sn, "region": "forearm"} for n, (r, sn) in regs.items()}
    rate = 2.0
    segs = P.line_sweep_segments(WRIST, list(schedule), rate, {"Sup": 60.0})
    rec = Recorder(path + ".csv")
    t, cmd, y, cycle = 0.0, {n: 0.0 for n in regs}, 0.0, 0

    def s_now():
        lo, hi = WRIST["FCR"]
        return 2.0 * (cmd["FCR"] - lo) / (hi - lo) - 1.0

    def sample(dt):
        nonlocal t, y
        for _ in range(int(round(dt / 0.05))):              # sensors at 20 Hz
            t += 0.05
            s = s_now()
            y = min(max(min(max(y, s - half_width), s + half_width), stop_lo), stop_hi)
            offset = drift_mm * (decay ** cycle)
            for n, k in SLOPES.items():
                rec.row(t, "len", regs[n][1], round((k * y + offset) / A.MM_PER_COUNT))

    y = -1.0 - half_width                                    # deflated: extended side
    for seg in segs:
        rec.row(t, "mark", note=seg.label)
        if seg.label.endswith("to +"):
            cycle += 1
        if seg.kind == "ramp":
            delta = max(abs(v - cmd[n]) for n, v in seg.targets.items())
            steps = max(1, math.ceil(delta / seg.rate / 0.2))
            start = dict(cmd)
            for k in range(1, steps + 1):
                for n, v in seg.targets.items():
                    cmd[n] = start[n] + (v - start[n]) * k / steps
                    rec.row(t, "cmd", regs[n][0], cmd[n])
                sample(0.2)
        else:
            sample(seg.seconds)
    rec.row(t, "mark", note="done")
    rec.close()
    with open(path + ".json", "w") as fh:
        json.dump({"mode": "line", "line": {n: list(v) for n, v in WRIST.items()},
                   "centre": 0.0, "schedule": [list(b) for b in schedule],
                   "turns": [list(x) for x in P.schedule_turns(list(schedule))],
                   "rate": rate, "cycles": sum(n for _, n in schedule), "hold": {"Sup": 60.0},
                   "pre_hold": 0.0, "dither": None,
                   "condition": {"mass_g": 0.0, "station": None, "session": "d1", "mount": None},
                   "muscles": muscles}, fh)


class LineAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.tmp.name, "run")
        simulate_line_run(cls.path)
        cls.res = A.compute(cls.path + ".csv", slip_mm=0.5, tol_mm=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_coordinate_and_amplitudes(self):
        self.assertEqual(self.res["lead"], "FCR")
        self.assertEqual([c["amp"] for c in self.res["cycles"]], [0.5] * 3 + [0.3] * 2 + [0.8] * 2)

    def test_dead_band_in_s(self):
        # 2 * half_width plus 0.5 mm of travel at each muscle's slope, within a command step
        for c in self.res["cycles"][1:5]:
            for name, sn in (("FCR", 68), ("FCU", 67), ("ECR", 70), ("ECU", 69)):
                expected = 0.2 + 0.5 / abs(SLOPES[name])
                self.assertAlmostEqual(c["dead_plus"][sn], expected, delta=0.03)
                self.assertAlmostEqual(c["dead_minus"][sn], expected, delta=0.03)

    def test_the_stop_is_flagged_only_where_it_is_hit(self):
        for c in self.res["cycles"]:
            hit_plus = [sn for sn, v in c["clip_plus"].items() if v]
            hit_minus = [sn for sn, v in c["clip_minus"].items() if v]
            self.assertEqual(hit_minus, [])
            if c["amp"] == 0.8:
                self.assertEqual(sorted(hit_plus), [67, 68, 69, 70])
            else:
                self.assertEqual(hit_plus, [])

    def test_slope_per_unit_s_skips_the_stop(self):
        # Within one command step (0.02 in s) over the 0.4 a slope window; a
        # ramp into the stop would pull the a = 0.8 median far towards 0.
        for name, sn in (("FCR", 68), ("ECR", 70)):
            self.assertAlmostEqual(self.res["slope"][sn] / SLOPES[name], 1.0, delta=0.15)
            self.assertAlmostEqual(self.res["slope_by_amp"][0.8][sn] / SLOPES[name], 1.0,
                                   delta=0.1)

    def test_settling_over_the_first_block(self):
        self.assertEqual(self.res["settled"], 3)

    def test_report_and_plots(self):
        text = A.analyze(self.path + ".csv", plots=True)
        self.assertIn("1 unit of s = 20 kPa of FCR", text)
        self.assertIn("a 0.8", text)
        self.assertIn("clipped 2/4 turns", text)
        self.assertIn("settled at cycle 3 (first block)", text)
        for suffix in ("_report.txt", "_loops.png", "_settling.png"):
            self.assertTrue(os.path.exists(self.path + suffix))


if __name__ == "__main__":
    unittest.main()
