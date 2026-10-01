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
