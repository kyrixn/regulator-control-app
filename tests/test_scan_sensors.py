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
        samples = [Reading(50, "ok", counts=100, status=0), Reading(50, "ok", counts=103, status=2),
                   Reading(50, "timeout"), Reading(51, "error")]
        st = S.sensor_stats(samples, mm_per_count=0.5)
        self.assertEqual((st[50]["ok"], st[50]["timeout"], st[50]["error"]), (2, 1, 0))
        self.assertEqual((st[50]["jitter_counts"], st[50]["jitter_mm"]), (3, 1.5))
        self.assertEqual(st[50]["status"], [0, 2])
        self.assertIsNone(st[51]["jitter_counts"])
        self.assertEqual(st[51]["status"], [])

    def test_quality_problems(self):
        stats = {50: {"ok": 100, "timeout": 0, "error": 0},
                 51: {"ok": 98, "timeout": 1, "error": 1},
                 52: {"ok": 0, "timeout": 0, "error": 0}}
        self.assertEqual(S.quality_problems(stats, 0.0),
                         ["sensor 51: 2/100 reads failed (1 timeout, 1 CRC/protocol)"])
        self.assertEqual(S.quality_problems(stats, 0.05), [])


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
