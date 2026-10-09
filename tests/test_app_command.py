"""app.on_command: the command box (kPa), STOP and 'ps <name>'."""

import json
import os
import sys
import tempfile
import unittest
from collections import deque
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app  # noqa: E402
from presets import Step  # noqa: E402


class FakeValves:
    def __init__(self):
        self.max_kpa = 50.0
        self.message_queue = deque()
        self.calls = []
        self.adc_time = 0.0
        self.adc_volts = {}

    def set_multiple_valves(self, pairs, ramp=0.0):
        self.calls.append(("set", pairs, ramp))
        return True

    def send(self, cmd):
        self.calls.append(("send", cmd))
        return True

    def emergency_stop(self):
        self.calls.append(("stop",))
        return True


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.vc = FakeValves()
        self.runner = mock.Mock()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "presets.json")
        with open(path, "w") as fh:
            json.dump({"base": {"4": 30, "6": 30, "t": 5, "p": 0},
                       "over": [{"4": 30}, {"4": 80}]}, fh)
        for name, value in (("valve_controller", self.vc), ("preset_runner", self.runner),
                            ("presets_path", path)):
            patcher = mock.patch.object(app, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def errors(self):
        return [m for m in self.vc.message_queue if m.startswith("[ERR]")]

    def test_set_in_kpa_uses_the_time_box(self):
        app.on_command({"text": "4,30,6,off", "ramp": "2"})
        self.assertEqual(self.vc.calls, [("set", [(4, 30.0), (6, "off")], 2.0)])
        self.assertIn("> 4,30,6,off", self.vc.message_queue)

    def test_over_the_cap_sends_nothing(self):
        app.on_command({"text": "4,60"})
        self.assertEqual(self.vc.calls, [])
        self.assertTrue(self.errors())

    def test_stop_ends_the_preset_then_vents(self):
        order = []
        self.runner.stop.side_effect = lambda: order.append("preset")
        self.vc.emergency_stop = lambda: order.append("vent")
        app.on_command({"text": "s"})
        self.assertEqual(order, ["preset", "vent"])

    def test_giga_command_passes_through(self):
        app.on_command({"text": "?"})
        self.assertEqual(self.vc.calls, [("send", "?")])

    def test_preset_starts(self):
        app.on_command({"text": "ps base"})
        self.runner.start.assert_called_once_with(
            "base", [Step([(4, 30.0), (6, 30.0)], 5.0, 0.0)])

    def test_unknown_preset_lists_the_names(self):
        app.on_command({"text": "ps nope"})
        self.runner.start.assert_not_called()
        self.assertIn("base, over", self.errors()[0])

    def test_preset_over_the_cap_never_starts(self):
        app.on_command({"text": "ps over"})
        self.runner.start.assert_not_called()
        self.assertIn("step 2", self.errors()[0])

    def test_no_valves(self):
        with mock.patch.object(app, "valve_controller", None), \
                mock.patch.object(app.socketio, "emit") as emit:
            app.on_command({"text": "4,30"})
        emit.assert_called_once()


if __name__ == "__main__":
    unittest.main()
