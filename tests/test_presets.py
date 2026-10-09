"""presets.py: the typed command line (kPa), presets and the sequence runner."""

import json
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from presets import (CommandError, PresetRunner, Step, load_presets,  # noqa: E402
                     parse_command, parse_preset)

CAP = 50.0


class ParseCommandTests(unittest.TestCase):
    def test_giga_commands_pass_through(self):
        self.assertEqual(parse_command("?", CAP), ("raw", "?"))
        self.assertEqual(parse_command("p", CAP), ("raw", "p"))
        self.assertEqual(parse_command(" A ", CAP), ("raw", "a"))

    def test_stop(self):
        self.assertEqual(parse_command("s", CAP), ("stop",))
        self.assertEqual(parse_command("S", CAP), ("stop",))

    def test_set_in_kpa(self):
        self.assertEqual(parse_command("4,30", CAP), ("set", [(4, 30.0)]))
        self.assertEqual(parse_command("4, 30.5, 6,off", CAP),
                         ("set", [(4, 30.5), (6, "off")]))

    def test_preset(self):
        self.assertEqual(parse_command("ps base30", CAP), ("preset", "base30"))
        self.assertEqual(parse_command("PS  base 30 ", CAP), ("preset", "base 30"))

    def test_rejects(self):
        for bad in ("", "ps", "4", "4,30,6", "40,10", "-1,10", "4,51", "4,-5",
                    "4,abc", "x,10", "hello", "4;30"):
            with self.subTest(bad=bad), self.assertRaises(CommandError):
                parse_command(bad, CAP)


class ParsePresetTests(unittest.TestCase):
    def test_sequence(self):
        steps = parse_preset([{"4": 30, "6": 30, "t": 5, "p": 2},
                              {"4": "off", "t": 0, "p": 0}], CAP)
        self.assertEqual(steps, [Step([(4, 30.0), (6, 30.0)], 5.0, 2.0),
                                 Step([(4, "off")], 0.0, 0.0)])

    def test_one_line_preset_and_defaults(self):
        self.assertEqual(parse_preset({"4": 30}, CAP), [Step([(4, 30.0)], 0.0, 0.0)])

    def test_wait_only_step(self):
        self.assertEqual(parse_preset([{"p": 3}], CAP), [Step([], 0.0, 3.0)])

    def test_any_bad_step_rejects_the_preset(self):
        for bad in ([{"4": 30}, {"4": 60}],          # above the cap
                    [{"4": 30, "q": 1}],             # unknown key
                    [{"FCR": 30}],                   # names are not accepted
                    [{"32": 10}],                    # no such regulator
                    [{"4": 30, "t": -1}],
                    [{"4": 30, "p": "x"}],
                    [], "4,30", [3]):
            with self.subTest(bad=bad), self.assertRaises(CommandError):
                parse_preset(bad, CAP)


class LoadPresetsTests(unittest.TestCase):
    def test_load(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "presets.json")
            with open(path, "w") as fh:
                json.dump({"base": {"4": 30}}, fh)
            self.assertEqual(load_presets(path), {"base": {"4": 30}})

    def test_missing_or_broken_file(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(CommandError):
                load_presets(os.path.join(d, "none.json"))
            path = os.path.join(d, "bad.json")
            with open(path, "w") as fh:
                fh.write("{not json")
            with self.assertRaises(CommandError):
                load_presets(path)


class FakeValves:
    """Records set_multiple_valves calls; no ramps are ever pending."""

    def __init__(self):
        self.calls = []
        self.ramps = {}
        self.ramp_lock = threading.Lock()

    def set_multiple_valves(self, pairs, ramp=0.0):
        self.calls.append((time.monotonic(), list(pairs), ramp))
        return True


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.vc = FakeValves()
        self.log = []
        self.runner = PresetRunner(self.vc, self.log.append)

    def wait_done(self, timeout=2.0):
        end = time.monotonic() + timeout
        while self.runner.running and time.monotonic() < end:
            time.sleep(0.005)
        self.assertFalse(self.runner.running)

    def test_steps_in_order_with_ramp_and_pause(self):
        steps = [Step([(4, 30.0)], 0.05, 0.1), Step([(4, 20.0), (6, 10.0)], 0.0, 0.0)]
        self.assertTrue(self.runner.start("seq", steps))
        self.wait_done()
        (t1, p1, r1), (t2, p2, r2) = self.vc.calls
        self.assertEqual((p1, r1), ([(4, 30.0)], 0.05))
        self.assertEqual((p2, r2), ([(4, 20.0), (6, 10.0)], 0.0))
        self.assertGreaterEqual(t2 - t1, 0.15)

    def test_wait_only_step_sends_nothing(self):
        self.runner.start("wait", [Step([], 0.0, 0.01)])
        self.wait_done()
        self.assertEqual(self.vc.calls, [])

    def test_refuses_a_second_preset_while_running(self):
        self.runner.start("long", [Step([(4, 30.0)], 0.0, 1.0)])
        self.assertFalse(self.runner.start("other", [Step([(6, 30.0)], 0.0, 0.0)]))
        self.runner.stop()
        self.wait_done()
        self.assertEqual(len(self.vc.calls), 1)

    def test_stop_ends_the_sequence(self):
        self.runner.start("long", [Step([(4, 30.0)], 0.0, 1.0), Step([(4, 40.0)], 0.0, 0.0)])
        time.sleep(0.05)
        self.runner.stop()
        self.wait_done(timeout=0.5)
        time.sleep(0.05)
        self.assertEqual(len(self.vc.calls), 1)
        self.assertTrue(any("stopped" in line for line in self.log))

    def test_failed_send_aborts(self):
        self.vc.set_multiple_valves = lambda pairs, ramp=0.0: False
        self.runner.start("bad", [Step([(4, 30.0)], 0.0, 0.0), Step([(4, 40.0)], 0.0, 0.0)])
        self.wait_done()
        self.assertTrue(any("aborted" in line for line in self.log))


if __name__ == "__main__":
    unittest.main()
