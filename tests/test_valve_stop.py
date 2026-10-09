"""STOP and the ramp ticker share ramp_lock, so no ramp setpoint follows an 's'."""

import os
import sys
import threading
import time
import unittest
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from valve_controller import ValveController  # noqa: E402


def bare():
    """A ValveController with no serial port; send() records whether ramp_lock is held."""
    vc = ValveController.__new__(ValveController)
    vc.max_kpa = 140.0
    vc.message_queue = deque()
    vc.valve_data = {}
    vc.display_lock = threading.Lock()
    vc.ramps = {}
    vc.ramp_lock = threading.Lock()
    vc.running = True
    vc.sent = []

    def send(cmd):
        vc.sent.append((cmd, vc.ramp_lock.locked()))
        vc.running = False          # one ticker pass is enough
        return True
    vc.send = send
    return vc


class StopOrderTests(unittest.TestCase):
    def test_stop_sends_under_ramp_lock(self):
        vc = bare()
        vc.ramps[4] = {"start_val": 0, "target": 2000, "off": False,
                       "start_t": time.time(), "end_t": time.time() + 5}
        vc.emergency_stop()
        self.assertEqual(vc.ramps, {})
        self.assertEqual(vc.sent, [("s", True)])

    def test_ramp_ticker_sends_under_ramp_lock(self):
        vc = bare()
        vc.ramps[4] = {"start_val": 0, "target": 2000, "off": False,
                       "start_t": time.time() - 1, "end_t": time.time() - 0.5}
        vc._ramp_loop()
        self.assertEqual(vc.sent, [("4,2000", True)])


if __name__ == "__main__":
    unittest.main()
