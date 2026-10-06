"""The pair-sweep protocol and its analysis, on synthetic data."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiments import analyze_sweep as A  # noqa: E402
from experiments import pair_sweep as P  # noqa: E402


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.segs = P.pair_sweep_segments("FCR", "ECR", pretension=30, amplitude=15, rate=1.0,
                                          cycles=2, holds={"FCU": 30, "ECU": 30})

    def test_shape(self):
        labels = [s.label for s in self.segs]
        self.assertEqual(labels[:2], ["inflate", "settle"])
        self.assertEqual(labels[2:10], ["c1 to +", "c1 hold +", "c1 + to 0", "c1 zero from +",
                                        "c1 to -", "c1 hold -", "c1 - to 0", "c1 zero from -"])
        self.assertEqual(labels[-1], "vent")
        self.assertEqual(len(self.segs), 2 + 2 * 8 + 1)

    def test_pair_sum_is_constant_and_holds_stay(self):
        for s in self.segs[:-1]:
            self.assertEqual(s.targets["FCR"] + s.targets["ECR"], 60)
            self.assertEqual((s.targets["FCU"], s.targets["ECU"]), (30, 30))
        self.assertEqual(self.segs[2].targets, {"FCR": 45, "ECR": 15, "FCU": 30, "ECU": 30})
        self.assertEqual(set(self.segs[-1].targets.values()), {0.0})

    def test_explicit_turns(self):
        segs = P.pair_sweep_segments("FCR", "ECR", 30, 20, 1.0, 1, {"FCU": 15},
                                     turn_plus=(60, 10), turn_minus=(10, 50))
        turns = {s.label: (s.targets["FCR"], s.targets["ECR"]) for s in segs
                 if s.label in ("c1 hold +", "c1 hold -", "c1 zero from +")}
        self.assertEqual(turns, {"c1 hold +": (60, 10), "c1 hold -": (10, 50),
                                 "c1 zero from +": (30, 30)})
        _, ranges = P.plan_summary(segs, {})
        self.assertEqual((ranges["FCR"][1], ranges["ECR"][1]), (60, 50))
        with self.assertRaises(ValueError):
            P.pair_sweep_segments("FCR", "ECR", 30, 20, 1.0, 1, {}, turn_plus=(60, -5))

    def test_pre_hold(self):
        segs = P.pair_sweep_segments("FCR", "ECR", 35, 25, 1.0, 1, {}, pre_hold_s=600)
        self.assertEqual([s.label for s in segs[:3]], ["inflate", "settle", "pre-hold"])
        self.assertEqual((segs[2].kind, segs[2].seconds), ("hold", 600))

    def test_dither_widens_the_planned_range(self):
        segs = P.pair_sweep_segments("FCR", "ECR", 35, 21, 1.0, 1, {"FCU": 15})
        _, ranges = P.plan_summary(segs, {}, dither=(4.0, ("FCR", "ECR")))
        self.assertEqual(ranges["FCR"][1], 60)
        self.assertEqual(P.cycle_min(segs, ("FCR", "ECR")) - 4.0, 10)
        self.assertEqual(ranges["FCU"], (0.0, 15))

    def test_amplitude_must_fit_the_pretension(self):
        with self.assertRaises(ValueError):
            P.pair_sweep_segments("FCR", "ECR", 10, 15, 1.0, 1, {})

    def test_plan_summary(self):
        total, ranges = P.plan_summary(self.segs, start={})
        # inflate 30/5=6 s, settle 5 s, per cycle 4x15 s + 2+2+2+3 s, vent 6 s
        self.assertAlmostEqual(total, 6 + 5 + 2 * (60 + 9) + 6)
        self.assertEqual(ranges["FCR"], (0.0, 45))
        self.assertEqual(ranges["ECR"], (0.0, 45))


def stick_slip(half_width):
    """A friction dead band (play operator): the position only follows d once d
    has moved 2 * half_width past the last reversal. 0.2 mm per kPa."""
    t, d, length = [], [], []
    y = 0.0
    for i in range(0, 401):                       # d: 0 -> 10 -> -30 (0.1 kPa steps)
        di = i * 0.1 if i <= 100 else 10 - (i - 100) * 0.1
        y = min(max(y, di - half_width), di + half_width)
        t.append(i * 0.1)
        d.append(di)
        length.append(0.2 * y)
    return t, d, length


class AnalysisTests(unittest.TestCase):
    def test_breakaway(self):
        t, d, length = stick_slip(half_width=1.5)
        # reversal at d = +10 (t = 10 s): the band is 3 kPa, and 0.05 mm more
        # takes another 0.25 kPa, so motion is detected 3.3 kPa after the reversal
        b = A.breakaway(t, length, t, d, t0=10.0, t1=40.0, thr_mm=0.05)
        self.assertAlmostEqual(b, 3.3, delta=0.11)
        self.assertIsNone(A.breakaway(t, [0.0] * len(t), t, d, 10.0, 40.0, 0.05))

    def test_hold_mean_uses_the_end_of_the_hold(self):
        t = [0, 1, 2, 3, 4]
        y = [9, 9, 1, 1, 1]
        self.assertEqual(A.hold_mean(t, y, 0, 4, last_s=2.0), 1)

    def test_settled_cycle(self):
        refs = [{68: 0.0, 70: 0.0}, {68: 0.5, 70: -0.3}, {68: 0.55, 70: -0.32},
                {68: 0.56, 70: -0.31}]
        self.assertEqual(A.settled_cycle(refs, tol_mm=0.1), 3)
        self.assertIsNone(A.settled_cycle(refs[:2], tol_mm=0.1))

    def test_differential_is_the_half_difference_from_the_base(self):
        t = [0.0, 1.0, 2.0]
        d = A.differential(t, [30, 60, 10], t, [30, 10, 50], base=(30, 30))
        self.assertEqual(d, [0.0, 25.0, -20.0])
        # with a constant sum it equals p(a) - P, as in the first runs
        self.assertEqual(A.differential(t, [30, 45, 15], t, [30, 15, 45], base=(30, 30)),
                         [0.0, 15.0, -15.0])

    def test_smooth_removes_the_dither(self):
        import math
        t = [i * 0.01 for i in range(1000)]                    # 10 s at 100 Hz
        y = [0.1 * ti + 0.5 * math.sin(2 * math.pi * 2.0 * ti) for ti in t]
        ys = A.smooth(t, y, window_s=0.5)                      # one 2 Hz period
        mid = [abs(v - 0.1 * ti) for ti, v in zip(t, ys) if 1 < ti < 9]
        self.assertLess(max(mid), 0.02)

    def test_drift_rate(self):
        t = [i * 1.0 for i in range(601)]                      # 10 min at 1 Hz
        y = [0.002 * ti for ti in t]                           # 0.12 mm/min
        total, rate = A.drift(t, y, 0.0, 600.0, last_s=60.0)
        self.assertAlmostEqual(total, 1.2)
        self.assertAlmostEqual(rate, 0.12)

    def test_readback_fit(self):
        kpa = [0, 10, 20, 30]
        volts = [0.5 + 0.02 * k for k in kpa]
        m, c, r2 = A.line_fit(kpa, volts)
        self.assertAlmostEqual(m, 0.02)
        self.assertAlmostEqual(c, 0.5)
        self.assertAlmostEqual(r2, 1.0)


def simulate_run(path, half_width=6.0, slope=0.35, drift_mm=0.6, decay=0.3):
    """A whole FCR/ECR run through the real Recorder: a friction play operator of
    `half_width` kPa on d (so after a turn the wrist stays put for 2*half_width,
    past zero, like the real wrist), `slope` mm per kPa once moving, and a
    settling offset on both sensors that shrinks by `decay` each cycle.
    Returns (band kPa, slope)."""
    import json
    import math
    from station import Recorder

    muscles = {"FCR": {"regulator": 4, "sensor": 68, "region": "forearm"},
               "ECR": {"regulator": 6, "sensor": 70, "region": "forearm"}}
    P, A, cycles = 30.0, 10.0, 4
    segs = P_mod.pair_sweep_segments("FCR", "ECR", P, A, rate=2.0, cycles=cycles, holds={})
    rec = Recorder(path + ".csv")
    t, cmd, y, cycle = 0.0, {"FCR": 0.0, "ECR": 0.0}, 0.0, 0

    def sample(dt):
        nonlocal t, y
        for _ in range(int(round(dt / 0.05))):              # sensors at 20 Hz
            t += 0.05
            d = cmd["FCR"] - P
            y = min(max(y, d - half_width), d + half_width)
            offset = drift_mm * (decay ** cycle)
            rec.row(t, "len", 68, round((slope * y + offset) / A_mod.MM_PER_COUNT))
            rec.row(t, "len", 70, round((-slope * y + offset) / A_mod.MM_PER_COUNT))

    for s in segs:
        rec.row(t, "mark", note=s.label)
        if s.label.endswith("to +"):
            cycle += 1
        if s.kind == "ramp":
            delta = max(abs(v - cmd[n]) for n, v in s.targets.items())
            steps = max(1, math.ceil(delta / s.rate / 0.2))
            start = dict(cmd)
            for k in range(1, steps + 1):
                for n, v in s.targets.items():
                    cmd[n] = start[n] + (v - start[n]) * k / steps
                    rec.row(t, "cmd", muscles[n]["regulator"], cmd[n])
                sample(0.2)
        else:
            sample(s.seconds)
    rec.row(t, "mark", note="done")
    rec.close()
    with open(path + ".json", "w") as fh:
        json.dump({"pair": ["FCR", "ECR"], "pretension": P, "amplitude": A, "rate": 2.0,
                   "cycles": cycles, "hold": {}, "muscles": muscles}, fh)
    return 2 * half_width, slope


P_mod, A_mod = P, A


class EndToEndTests(unittest.TestCase):
    def test_compute_recovers_the_simulated_friction_and_settling(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "run")
            band, slope = simulate_run(path)
            res = A.compute(path + ".csv", slip_mm=0.5, tol_mm=0.1)
        cycles = res["cycles"]
        self.assertEqual(len(cycles), 4)
        # band + 0.5 mm of travel at `slope`, within one 0.4 kPa command step
        expected = band + 0.5 / slope
        for c in cycles[1:]:                       # cycle 1 starts from the inflated rest
            for s in (68, 70):
                self.assertAlmostEqual(c["dead_plus"][s], expected, delta=0.6)
        for c in cycles[:-1]:                      # the last -A turn is followed by the vent
            for s in (68, 70):
                self.assertAlmostEqual(c["dead_minus"][s], expected, delta=0.6)
        self.assertIsNone(cycles[-1]["dead_minus"][68])
        self.assertAlmostEqual(res["slope"][68], slope, delta=0.05)
        self.assertAlmostEqual(res["slope"][70], -slope, delta=0.05)
        # offsets 0.6*0.3^k: change < 0.1 mm first between cycles 2 and 3
        self.assertEqual(res["settled"], 3)

    def test_report_renders(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "run")
            simulate_run(path)
            text = A.analyze(path + ".csv", plots=False)
        self.assertIn("dead band", text)
        self.assertIn("settled at cycle 3", text)


if __name__ == "__main__":
    unittest.main()
