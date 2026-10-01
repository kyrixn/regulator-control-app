"""app.resolve_ports on Windows and the mapping-derived default ids."""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app  # noqa: E402
from tests.test_ports import BLUETOOTH, GIGA, WINDOWS  # noqa: E402


def args(**kw):
    base = dict(valve_port=None, rs485_ports=None, no_valves=False, no_encoders=False)
    base.update(kw)
    return SimpleNamespace(**base)


class ResolvePortsTests(unittest.TestCase):
    def ports(self, listing):
        mock.patch("serial.tools.list_ports.comports", return_value=listing).start()
        mock.patch("encoder_controller.glob.glob", return_value=[]).start()
        self.addCleanup(mock.patch.stopall)

    def test_giga_and_every_channel(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args()),
                         ("COM5", ["COM7", "COM8", "COM10", "COM14"]))

    def test_explicit_rs485_ports(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args(rs485_ports=["COM7", "COM8"])),
                         ("COM5", ["COM7", "COM8"]))

    def test_no_adapter_never_falls_back_to_giga_or_bluetooth(self):
        self.ports([GIGA, BLUETOOTH])
        self.assertEqual(app.resolve_ports(args(no_valves=True)), (None, []))

    def test_same_port_twice_is_refused(self):
        self.ports(WINDOWS)
        with self.assertRaises(SystemExit):
            app.resolve_ports(args(valve_port="COM7", rs485_ports=["COM7"]))

    def test_no_encoders(self):
        self.ports(WINDOWS)
        self.assertEqual(app.resolve_ports(args(no_encoders=True)), ("COM5", []))


class DefaultIdsTests(unittest.TestCase):
    def test_ids_come_from_the_mapping(self):
        self.assertEqual(app.default_slave_ids({175: {}, 50: {}, 53: {}}), [50, 53, 175])

    def test_empty_mapping_falls_back(self):
        from encoder_controller import DEFAULT_SLAVE_IDS
        self.assertEqual(app.default_slave_ids({}), list(DEFAULT_SLAVE_IDS))

    def test_port_list(self):
        self.assertEqual(app.parse_port_list(" COM7, COM8 ,"), ["COM7", "COM8"])


if __name__ == "__main__":
    unittest.main()
