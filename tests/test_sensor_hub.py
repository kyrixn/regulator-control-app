"""SensorHub merging, duplicate detection and routing, with fake controllers."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensor_hub import SensorHub  # noqa: E402


class FakeController:
    def __init__(self, port, slave_ids, answering):
        self.port_name = port
        self.slave_ids = list(slave_ids)
        self.active_slaves = [s for s in slave_ids if s in answering]
        self.connected = True
        self.calls = []
        self.messages = [f"[OK] Connected to {port}"]

    def get_state(self):
        enc = [{"slave": s, "online": True, "position_mm": float(s), "zeroed": False,
                "message": "ok"} for s in self.active_slaves]
        return {"connected": self.connected, "encoders": enc, "online": len(enc)}

    def zero(self, slave):
        self.calls.append(("zero", slave))

    def zero_all(self):
        self.calls.append(("zero_all",))

    def clear_zero(self):
        self.calls.append(("clear_zero",))

    def request_rescan(self, slave_ids=None):
        self.calls.append(("rescan", slave_ids))

    def drain_messages(self):
        out, self.messages = self.messages, []
        return out

    def close(self):
        self.calls.append(("close",))


def make_hub(wiring, ids=(50, 51, 52, 173, 174)):
    fakes = {}

    def factory(port, slave_ids):
        if port == "BAD":
            raise OSError("could not open port 'BAD'")
        fakes[port] = FakeController(port, slave_ids, wiring.get(port, ()))
        return fakes[port]

    return SensorHub(list(wiring), ids, factory=factory), fakes


WIRING = {"COM7": {50, 51}, "COM8": {52, 173}, "COM9": set()}


class HubTests(unittest.TestCase):
    def test_merged_state(self):
        hub, _ = make_hub(WIRING)
        st = hub.get_state()
        self.assertTrue(st["connected"])
        self.assertEqual([e["slave"] for e in st["encoders"]], [50, 51, 52, 173])
        self.assertEqual({e["slave"]: e["port"] for e in st["encoders"]},
                         {50: "COM7", 51: "COM7", 52: "COM8", 173: "COM8"})
        self.assertEqual(st["online"], 4)
        self.assertEqual([(p["port"], p["found"]) for p in st["ports"]],
                         [("COM7", 2), ("COM8", 2), ("COM9", 0)])
        self.assertEqual(st["duplicates"], [])

    def test_duplicate_id_keeps_first_port_and_warns_once(self):
        hub, _ = make_hub({"COM7": {50}, "COM8": {50, 51}})
        st = hub.get_state()
        self.assertEqual(st["duplicates"], [50])
        self.assertEqual(st["encoders"][0]["port"], "COM7")
        warnings = [m for m in hub.drain_messages() if "[WARN]" in m]
        self.assertEqual(warnings, ["[WARN] sensor 50 answers on COM7 and COM8; showing COM7"])
        self.assertEqual([m for m in hub.drain_messages() if "[WARN]" in m], [])

    def test_zero_goes_to_the_owning_port(self):
        hub, fakes = make_hub(WIRING)
        hub.zero(173)
        self.assertEqual(fakes["COM8"].calls, [("zero", 173)])
        self.assertEqual(fakes["COM7"].calls, [])

    def test_zero_unknown_sensor_warns(self):
        hub, fakes = make_hub(WIRING)
        hub.drain_messages()
        hub.zero(99)
        self.assertEqual(hub.drain_messages(),
                         ["[WARN] cannot zero sensor 99: not found on any port"])
        self.assertTrue(all(f.calls == [] for f in fakes.values()))

    def test_fan_out(self):
        hub, fakes = make_hub(WIRING)
        hub.zero_all()
        hub.clear_zero()
        hub.request_rescan([1, 2])
        hub.close()
        for f in fakes.values():
            self.assertEqual(f.calls, [("zero_all",), ("clear_zero",), ("rescan", [1, 2]),
                                       ("close",)])

    def test_messages_are_prefixed_with_the_port(self):
        hub, _ = make_hub({"COM7": {50}})
        self.assertEqual(hub.drain_messages(), ["COM7: [OK] Connected to COM7"])

    def test_a_port_that_fails_to_open_is_skipped(self):
        hub, _ = make_hub({"COM7": {50}, "BAD": set()})
        self.assertEqual(hub.ports, ["COM7"])
        self.assertIn("[ERR] BAD: could not open port 'BAD'", hub.drain_messages())

    def test_no_ports_is_not_connected(self):
        hub, _ = make_hub({})
        self.assertFalse(hub.connected)
        self.assertEqual(hub.get_state()["encoders"], [])


if __name__ == "__main__":
    unittest.main()
