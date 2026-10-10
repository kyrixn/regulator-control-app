"""The Giga's BNO055 stream: frame parsing, the reader, start/stop, the command
box and the station log."""

import csv
import os
import sys
import tempfile
import threading
import unittest
from collections import deque
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import presets as P  # noqa: E402
import station as S  # noqa: E402
import valve_controller as V  # noqa: E402

# seq 42, giga 123456 ms, q = (1, 0, -0.5, 0), gyro (1, -2, 0) deg/s,
# linear accel (1, -2.5, 9.81) m/s^2, calibration sys 3 gyr 2 acc 1 mag 0.
FRAME = ("I" "002A" "0001E240"
         "4000" "0000" "E000" "0000"
         "0010" "FFE0" "0000"
         "0064" "FF06" "03D5"
         "E4")


def bare():
    """A ValveController with no serial port; send() records the commands."""
    vc = V.ValveController.__new__(V.ValveController)
    vc.message_queue = deque()
    vc.valve_data = {}
    vc.display_lock = threading.Lock()
    vc.adc_volts, vc.adc_time, vc.adc_listeners = {}, 0.0, []
    vc.imu, vc.imu_time, vc.imu_listeners = None, 0.0, []
    vc.imu_streaming = False
    vc.ramps, vc.ramp_lock = {}, threading.Lock()
    vc.read_thread = vc.ramp_thread = None
    vc.sent = []
    vc.send = lambda cmd: vc.sent.append(cmd) or True
    return vc


class ParseTests(unittest.TestCase):
    def test_frame_fits_one_usb_packet(self):
        self.assertLessEqual(len(FRAME) + 1, 64)          # + "\n"

    def test_decodes_and_scales(self):
        f = V.parse_imu_line(FRAME)
        self.assertEqual((f.seq, f.giga_ms), (42, 123456))
        self.assertEqual(f.quat, (1.0, 0.0, -0.5, 0.0))
        self.assertEqual(f.gyro, (1.0, -2.0, 0.0))
        self.assertEqual(f.accel, (1.0, -2.5, 9.81))
        self.assertEqual(f.calib, (3, 2, 1, 0))

    def test_other_lines_are_not_frames(self):
        for line in ("IMU: stream on, 45 Hz", FRAME[:-1], FRAME + "0",
                     FRAME.lower(), "PONG - Arduino connected", ""):
            self.assertIsNone(V.parse_imu_line(line), line)


class ReaderTests(unittest.TestCase):
    def test_frame_reaches_listeners_not_the_log(self):
        vc = bare()
        got = []
        vc.imu_listeners.append(lambda frame, t: got.append(frame.seq))
        vc._handle_response(FRAME)
        self.assertEqual(vc.imu.seq, 42)
        self.assertGreater(vc.imu_time, 0.0)
        self.assertEqual(got, [42])
        self.assertEqual(list(vc.message_queue), [])

    def test_imu_status_lines_are_logged(self):
        vc = bare()
        vc._handle_response("IMU: stream on, 45 Hz")
        vc._handle_response("IMU: q=(1.0000,0.0000,0.0000,0.0000)")
        self.assertEqual(list(vc.message_queue),
                         ["IMU: stream on, 45 Hz", "IMU: q=(1.0000,0.0000,0.0000,0.0000)"])
        self.assertIsNone(vc.imu)

    def test_waiting_lines_are_read_without_sleeping_between_them(self):
        vc = bare()
        lines = [FRAME.encode() + b"\n"] * 5
        vc.ser = mock.Mock()
        type(vc.ser).in_waiting = mock.PropertyMock(side_effect=lambda: sum(map(len, lines)))
        vc.ser.readline.side_effect = lambda: lines.pop(0)
        vc.running = True
        sleeps = []

        def sleep(seconds):
            sleeps.append(len(lines))
            vc.running = False                    # idle: one pass is enough
        with mock.patch.object(V.time, "sleep", sleep):
            vc._read_loop()
        self.assertEqual(sleeps, [0])             # only once the input was drained
        self.assertEqual(vc.imu.seq, 42)


class StreamControlTests(unittest.TestCase):
    def test_start_and_stop(self):
        vc = bare()
        vc.imu_stream(True)
        self.assertTrue(vc.imu_streaming)
        vc.imu_stream(False)
        self.assertFalse(vc.imu_streaming)
        self.assertEqual(vc.sent, ["i1", "i0"])

    def test_close_stops_a_running_stream(self):
        vc = bare()
        vc.ser = mock.Mock()
        vc.imu_stream(True)
        vc.close()
        self.assertEqual(vc.sent, ["i1", "i0"])
        vc.ser.close.assert_called_once()

    def test_close_without_a_stream_sends_nothing(self):
        vc = bare()
        vc.ser = mock.Mock()
        vc.close()
        self.assertEqual(vc.sent, [])


class CommandBoxTests(unittest.TestCase):
    def test_imu_commands_pass_through(self):
        for text in ("i", "i1", "i0", "I1"):
            self.assertEqual(P.parse_command(text, 50.0), ("raw", text.lower()))


class FakeValves:
    def __init__(self):
        self.adc_listeners, self.imu_listeners = [], []
        self.streams = []
        self.closed = False

    def imu_stream(self, on):
        self.streams.append(on)
        return True

    def close(self):
        self.closed = True


class FakeHub:
    def add_listener(self, fn):
        pass

    def close(self):
        pass


MUSCLES = {"FCR": {"regulator": 4, "sensor": 68, "region": "forearm"}}


class StationTests(unittest.TestCase):
    def make(self, imu):
        d = tempfile.mkdtemp()
        rec = S.Recorder(os.path.join(d, "run.csv"))
        valves = FakeValves()
        st = S.Station(rec, MUSCLES, armed=False, hub=FakeHub(), valves=valves, imu=imu)
        return st, rec, valves

    def test_off_by_default(self):
        st, rec, valves = self.make(imu=False)
        st.close()
        self.assertEqual((valves.streams, valves.imu_listeners), ([], []))

    def test_frames_are_logged(self):
        st, rec, valves = self.make(imu=True)
        self.assertEqual(valves.streams, [True])
        valves.imu_listeners[0](V.parse_imu_line(FRAME), 5.0)
        st.close()
        rec.close()
        with open(rec.path, newline="") as fh:
            rows = [r for r in csv.DictReader(fh) if r["kind"] == "imu"]
        got = {r["id"]: r["value"] for r in rows}
        self.assertEqual(got, {"qw": "1.0", "qx": "0.0", "qy": "-0.5", "qz": "0.0",
                               "gx": "1.0", "gy": "-2.0", "gz": "0.0",
                               "ax": "1.0", "ay": "-2.5", "az": "9.81", "giga_ms": "123456"})
        self.assertEqual(rows[-1]["note"], "seq=42 cal=S3G2A1M0")
        self.assertTrue(all(r["t"] == "5.0000" for r in rows))


if __name__ == "__main__":
    unittest.main()
