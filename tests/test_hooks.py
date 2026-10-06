"""Hooks the experiments rely on: mapping by name, ADC parsing, Giga lookup,
encoder reading listeners."""

import json
import os
import sys
import tempfile
import unittest
from collections import deque
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import encoder_controller as E  # noqa: E402
import sensor_mapping as M  # noqa: E402
import valve_controller as V  # noqa: E402

ADC_LINE = "ADC seq=101256 C0=1608(0.489V) C1=1603(0.487V) C15=6230(1.878V)"


class MusclesByNameTests(unittest.TestCase):
    def test_entries_by_name(self):
        entries = [{"muscle": "FCR", "regulator": 4, "sensor": 68, "region": "forearm"},
                   {"muscle": "FDP4", "regulator": None, "sensor": 53},
                   {"muscle": "FPL", "regulator": 11, "sensor": None}]
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m.json")
            with open(p, "w") as fh:
                json.dump({"muscles": entries}, fh)
            by_name = M.muscles_by_name(p)
        self.assertEqual(by_name["FCR"], {"regulator": 4, "sensor": 68, "region": "forearm"})
        self.assertEqual(by_name["FDP4"], {"regulator": None, "sensor": 53, "region": None})
        self.assertEqual(by_name["FPL"], {"regulator": 11, "sensor": None, "region": None})

    def test_station_wrist(self):
        by_name = M.muscles_by_name()
        self.assertEqual({n: (by_name[n]["regulator"], by_name[n]["sensor"])
                          for n in ("PT", "Sup", "FCU", "FCR", "ECU", "ECR")},
                         {"PT": (1, 65), "Sup": (2, 66), "FCU": (3, 67), "FCR": (4, 68),
                          "ECU": (5, 69), "ECR": (6, 70)})


class AdcTests(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(V.parse_adc_line(ADC_LINE), {0: 0.489, 1: 0.487, 15: 1.878})
        self.assertIsNone(V.parse_adc_line("PONG - Arduino connected"))

    def test_handle_response_stores_and_notifies(self):
        vc = V.ValveController.__new__(V.ValveController)
        vc.display_lock = mock.MagicMock()
        vc.message_queue = deque()
        vc.valve_data = {}
        vc.adc_volts, vc.adc_time, vc.adc_listeners = {}, 0.0, []
        got = []
        vc.adc_listeners.append(lambda volts, t: got.append(volts))
        vc._handle_response(ADC_LINE)
        self.assertEqual(vc.adc_volts[15], 1.878)
        self.assertEqual(got, [{0: 0.489, 1: 0.487, 15: 1.878}])
        self.assertEqual(list(vc.message_queue), [])     # not spammed into the log


class OverlapDetectingSerial:
    """Fails like Windows pyserial does when two threads write at once."""

    def __init__(self):
        import threading
        self._busy = threading.Lock()
        self.overlaps = 0
        self.writes = 0

    def write(self, data):
        import time
        if not self._busy.acquire(blocking=False):
            self.overlaps += 1
            raise OSError("WriteFile failed (overlapping write)")
        try:
            time.sleep(0.0005)
            self.writes += 1
        finally:
            self._busy.release()


class SendLockTests(unittest.TestCase):
    def test_writes_from_two_threads_never_overlap(self):
        import threading
        vc = V.ValveController.__new__(V.ValveController)
        vc.ser = OverlapDetectingSerial()
        vc.message_queue = deque()
        vc._write_lock = threading.Lock()
        results = []

        def spam(cmd):
            for _ in range(100):
                results.append(vc.send(cmd))

        ts = [threading.Thread(target=spam, args=(c,)) for c in ("a", "4,2000")]
        [t.start() for t in ts]
        [t.join() for t in ts]
        self.assertEqual(vc.ser.overlaps, 0)
        self.assertEqual(vc.ser.writes, 200)
        self.assertTrue(all(results))


class FindGigaTests(unittest.TestCase):
    def test_by_vid(self):
        ports = [SimpleNamespace(device="COM20", hwid="USB VID:PID=0000:0000", vid=None),
                 SimpleNamespace(device="COM8", hwid="USB VID:PID=2341:0266 SER=0043", vid=0x2341)]
        with mock.patch("serial.tools.list_ports.comports", return_value=ports):
            self.assertEqual(V.find_giga_port(), "COM8")
        with mock.patch("serial.tools.list_ports.comports", return_value=ports[:1]):
            self.assertIsNone(V.find_giga_port())


class ListenerTests(unittest.TestCase):
    def test_polled_readings_reach_listeners(self):
        c = E.EncoderController(slave_ids=[50, 51], interval=0.0, autostart=False)
        c.running = True
        c._read_encoder = lambda s: E.EncoderReading(slave=s, online=True, absolute_position=s,
                                                    message="ok", timestamp=1.0)
        c._scan()
        got = []
        c.listeners.append(lambda r: got.append(r.slave))
        c._poll_once()
        self.assertEqual(got, [50, 51])


if __name__ == "__main__":
    unittest.main()
