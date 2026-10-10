"""imu_check.py's verdict on a captured stream (no hardware)."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import imu_check as C  # noqa: E402
from valve_controller import ImuFrame  # noqa: E402


def stream(n, period_ms=22.222, seq0=0, ms0=1000, skip=(), quat=(1.0, 0.0, 0.0, 0.0)):
    """n frames at the given period; seq numbers in `skip` are missing."""
    out = []
    for k in range(n):
        if k in skip:
            continue
        f = ImuFrame(seq=(seq0 + k) & 0xFFFF, giga_ms=int(ms0 + k * period_ms) & 0xFFFFFFFF,
                     quat=quat, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 0.0), calib=(3, 3, 3, 3))
        out.append((100.0 + k * period_ms / 1000, f))
    return out


class SummaryTests(unittest.TestCase):
    def test_clean_45_hz_stream(self):
        s = C.summarize(stream(451))
        self.assertEqual(s.frames, 451)
        self.assertAlmostEqual(s.rate_hz, 45.0, delta=0.1)
        self.assertEqual(s.missing, 0)
        self.assertEqual(C.problems(s), [])

    def test_counts_missing_frames_across_the_seq_wrap(self):
        s = C.summarize(stream(100, seq0=0xFFF0, skip=(20, 21, 50)))
        self.assertEqual(s.missing, 3)
        self.assertIn("3 frame(s) missing", C.problems(s)[0])

    def test_rate_outside_the_band(self):
        s = C.summarize(stream(100, period_ms=30.0))          # 33 Hz
        self.assertTrue(any("outside 40-50 Hz" in p for p in C.problems(s)))

    def test_zero_quaternion_means_no_fusion(self):
        s = C.summarize(stream(50, quat=(0.0, 0.0, 0.0, 0.0)))
        self.assertTrue(any("fusion" in p for p in C.problems(s)))

    def test_too_few_frames(self):
        self.assertEqual(C.problems(C.summarize(stream(1))),
                         ["only 1 frame(s) received"])


if __name__ == "__main__":
    unittest.main()
