"""EncoderController scan/rescan logic, with the serial read faked out."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import encoder_controller as E  # noqa: E402


def make(answering, ids):
    c = E.EncoderController(slave_ids=ids, interval=0.0, autostart=False)
    c.running = True
    c.reads = []

    def fake_read(slave):
        c.reads.append(slave)
        if slave not in answering:
            raise E.SerialTimeoutError("timeout")
        return E.EncoderReading(slave=slave, online=True, absolute_position=slave,
                                message="ok", timestamp=0.0)

    c._read_encoder = fake_read
    return c


class ScanTests(unittest.TestCase):
    def test_autostart_false_opens_nothing(self):
        c = E.EncoderController(slave_ids=[1], autostart=False)
        self.assertIsNone(c.ser)
        self.assertFalse(c.connected)

    def test_scan_keeps_answering_ids(self):
        c = make({50, 52}, [50, 51, 52])
        c._scan()
        self.assertEqual(c.active_slaves, [50, 52])
        self.assertEqual(sorted(c.readings), [50, 52])
        self.assertIn("Scan complete: 2", c.drain_messages()[-1])

    def test_rescan_drops_ids_that_stopped_answering(self):
        answering = {50, 51}
        c = make(answering, [50, 51])
        c._scan()
        answering.discard(51)
        c.request_rescan()
        c._poll_once()
        self.assertEqual(c.active_slaves, [50])
        self.assertEqual(sorted(c.readings), [50])

    def test_new_id_list_applies_on_the_poll_thread(self):
        c = make({173}, [50])
        c._scan()
        c.request_rescan([173])
        self.assertEqual(c.slave_ids, [50])      # not yet: the poll thread swaps it
        c._poll_once()
        self.assertEqual(c.slave_ids, [173])
        self.assertEqual(c.active_slaves, [173])

    def test_empty_port_idles_without_reading(self):
        c = make(set(), [50, 51])
        c._scan()
        c.reads.clear()
        c._poll_once()
        self.assertEqual(c.reads, [])
        self.assertIn("idle until RESCAN", c.drain_messages()[-1])

    def test_polls_active_ids(self):
        c = make({50}, [50, 51])
        c._scan()
        c.reads.clear()
        c._poll_once()
        self.assertEqual(c.reads, [50])

    def test_empty_id_list(self):
        c = make(set(), [])
        c._scan()
        self.assertIsNone(c.get_state()["scanned_range"])


if __name__ == "__main__":
    unittest.main()
