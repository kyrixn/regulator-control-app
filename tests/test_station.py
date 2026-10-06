"""station.py with fake devices: recording, health checks, ramps, aborts."""

import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import station as S  # noqa: E402
from encoder_controller import EncoderReading  # noqa: E402

MUSCLES = {"FCR": {"regulator": 4, "sensor": 68, "region": "forearm"},
           "ECR": {"regulator": 6, "sensor": 70, "region": "forearm"}}


def reading(slave, t, online=True):
    return EncoderReading(slave=slave, online=online, absolute_position=1000 + slave,
                          message="ok" if online else "timeout", timestamp=t)


class FakeValves:
    def __init__(self):
        self.calls = []
        self.adc_listeners = []
        self.stopped = False
        self.closed = False

    def set_multiple_valves(self, pairs, ramp=0.0):
        self.calls.append(list(pairs))
        return True

    def emergency_stop(self):
        self.stopped = True
        return True

    def close(self):
        self.closed = True


class FakeHub:
    def __init__(self):
        self.listeners = []
        self.closed = False

    def add_listener(self, fn):
        self.listeners.append(fn)

    def close(self):
        self.closed = True


def make(armed=True, fresh=True):
    d = tempfile.mkdtemp()
    rec = S.Recorder(os.path.join(d, "run.csv"))
    st = S.Station(rec, MUSCLES, armed=armed, max_kpa=50, hub=FakeHub(), valves=FakeValves(),
                   stale_s=0.5)
    st.sleep = lambda seconds: None          # no real waiting in tests
    if fresh:
        st.clock = lambda: 10.0
        for s in (68, 70):
            st._on_reading(reading(s, 10.0))
    return st, rec


def rows(rec):
    rec.close()
    with open(rec.path, newline="") as fh:
        return list(csv.DictReader(fh))


class HealthTests(unittest.TestCase):
    def test_problems(self):
        latest = {68: reading(68, 9.9), 70: reading(70, 9.0)}
        self.assertEqual(S.health_problem(latest, [68, 70], now=10.0, stale_s=0.5),
                         "sensor 70: no fresh reading for 1.00 s")
        self.assertEqual(S.health_problem(latest, [68, 71], now=10.0, stale_s=0.5),
                         "sensor 71: no reading")
        latest[70] = reading(70, 10.0, online=False)
        self.assertEqual(S.health_problem(latest, [70], now=10.0, stale_s=0.5),
                         "sensor 70: offline (timeout)")
        latest[70] = reading(70, 10.0)
        self.assertIsNone(S.health_problem(latest, [68, 70], now=10.0, stale_s=0.5))


class RecorderTests(unittest.TestCase):
    def test_rows(self):
        st, rec = make()
        st.mark("inflate")
        out = rows(rec)
        self.assertEqual(list(out[0].keys()), ["t", "kind", "id", "value", "note"])
        self.assertEqual([(r["kind"], r["id"], r["value"]) for r in out[:2]],
                         [("len", "68", "1068"), ("len", "70", "1070")])
        self.assertEqual((out[2]["kind"], out[2]["note"]), ("mark", "inflate"))


class SendTests(unittest.TestCase):
    def test_armed_send_logs_and_commands(self):
        st, rec = make()
        st.send({"FCR": 30, "ECR": 20})
        self.assertEqual(st.valves.calls, [[(4, 30.0), (6, 20.0)]])
        cmds = [(r["id"], r["value"], r["note"]) for r in rows(rec) if r["kind"] == "cmd"]
        self.assertEqual(cmds, [("4", "30.0", ""), ("6", "20.0", "")])

    def test_dry_send_logs_but_sends_nothing(self):
        st, rec = make(armed=False)
        st.send({"FCR": 30})
        self.assertEqual(st.valves.calls, [])
        self.assertEqual([r["note"] for r in rows(rec) if r["kind"] == "cmd"], ["dry"])

    def test_out_of_range_is_refused(self):
        st, _ = make()
        for bad in ({"FCR": 51}, {"FCR": -1}, {"PT": 10}):
            with self.assertRaises(ValueError):
                st.send(bad)
        self.assertEqual(st.valves.calls, [])


class RampTests(unittest.TestCase):
    def test_ramp_keeps_the_pair_sum_and_ends_on_target(self):
        st, _ = make()
        st.commanded = {4: 30.0, 6: 30.0}
        st.ramp({"FCR": 40, "ECR": 20}, rate=100.0, tick=0.01)
        calls = st.valves.calls
        self.assertEqual(calls[-1], [(4, 40.0), (6, 20.0)])
        for c in calls:
            self.assertAlmostEqual(dict(c)[4] + dict(c)[6], 60.0, places=6)

    def test_unhealthy_sensor_aborts_and_stops(self):
        st, _ = make(fresh=False)
        st.clock = lambda: 10.0
        st._on_reading(reading(68, 10.0))       # 70 never reads
        with self.assertRaises(S.Abort):
            st.ramp({"FCR": 40, "ECR": 20}, rate=100.0, tick=0.01)
        self.assertTrue(st.valves.stopped)
        self.assertEqual(st.valves.calls, [])

    def test_refused_command_aborts_with_the_valve_layer_reason(self):
        st, _ = make()
        st.valves.set_multiple_valves = lambda pairs, ramp=0.0: False
        st.valves.message_queue = ["[ERR] Send error: WriteFile failed"]
        with self.assertRaises(S.Abort) as ctx:
            st.send({"FCR": 30})
        self.assertIn("WriteFile failed", str(ctx.exception))
        self.assertTrue(st.valves.stopped)

    def _ticking(self, st):
        t = [10.0]
        st.clock = lambda: t[0]
        st.sleep = lambda s: t.__setitem__(0, t[0] + s)
        st.stale_s = 1e9                       # fake sensors never refresh
        return t

    def test_dither_keeps_the_pair_sum_and_logs_the_base(self):
        st, rec = make()
        st.commanded = {4: 30.0, 6: 30.0}
        self._ticking(st)
        st.set_dither(4.0, 2.0, {"FCR": +1, "ECR": -1})
        st.hold(0.5)
        calls = [dict(c) for c in st.valves.calls]
        self.assertGreaterEqual(len(calls), 9)               # streams at 20 Hz while dithering
        for c in calls:
            self.assertAlmostEqual(c[4] + c[6], 60.0)
            self.assertLessEqual(abs(c[4] - 30.0), 4.0 + 1e-9)
        self.assertGreater(max(abs(c[4] - 30.0) for c in calls), 3.0)
        out = rows(rec)
        bases = {(r["id"], r["value"]) for r in out if r["kind"] == "base"}
        self.assertEqual(bases, {("4", "30.0"), ("6", "30.0")})
        self.assertEqual(st.commanded, {4: 30.0, 6: 30.0})   # ramps continue from the base

    def test_dither_beyond_the_cap_aborts(self):
        st, _ = make()
        st.commanded = {4: 48.0, 6: 30.0}
        self._ticking(st)
        st.set_dither(4.0, 2.0, {"FCR": +1})
        with self.assertRaises(S.Abort):
            st.hold(0.3)
        self.assertTrue(st.valves.stopped)

    def test_clear_dither_returns_to_the_base(self):
        st, _ = make()
        st.commanded = {4: 30.0, 6: 30.0}
        t = self._ticking(st)
        st.set_dither(4.0, 2.0, {"FCR": +1, "ECR": -1})
        t[0] += 0.125                                          # a quarter period: +4 kPa
        st.hold(0.01)
        st.clear_dither()
        self.assertEqual(dict(st.valves.calls[-1]), {4: 30.0, 6: 30.0})

    def test_ping(self):
        st, _ = make()
        st.valves.message_queue = []
        st.valves.send = lambda cmd: st.valves.message_queue.append("PONG - Arduino connected")
        self.assertTrue(st.ping(timeout=0.2))
        st.valves.send = lambda cmd: True       # standalone mode: serial ignored, no reply
        self.assertFalse(st.ping(timeout=0.2))

    def test_close_after_error_stops_the_valves(self):
        st, _ = make()
        st.commanded = {4: 30.0}
        st.close(error=True)
        self.assertTrue(st.valves.stopped)
        self.assertTrue(st.hub.closed and st.valves.closed)


if __name__ == "__main__":
    unittest.main()
