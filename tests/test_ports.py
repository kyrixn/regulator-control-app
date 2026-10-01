"""Port discovery: Windows WCH COM ports by name, Linux /dev nodes by glob."""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import encoder_controller as E  # noqa: E402


def com(dev, desc, manu, hwid="USB VID:PID=0000:0000"):
    return SimpleNamespace(device=dev, description=desc, manufacturer=manu, hwid=hwid)


# As pyserial listed them on the station PC on 2026-09-29, plus a Giga.
GIGA = com("COM5", "USB Serial Device (COM5)", "Microsoft", "USB VID:PID=2341:0266 SER=0043")
BLUETOOTH = com("COM3", "Standard Serial over Bluetooth link (COM3)", "Microsoft",
                "BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}")
WINDOWS = [
    com("COM10", "WCH USB-SERIAL Ch D (COM10)", "wch.cn"),
    BLUETOOTH,
    com("COM7", "WCH USB-SERIAL Ch A (COM7)", "wch.cn"),
    com("COM14", "WCH USB-SERIAL Ch H (COM14)", "wch.cn"),
    com("COM8", "WCH USB-SERIAL Ch B (COM8)", "wch.cn"),
    GIGA,
]


class WindowsDiscoveryTests(unittest.TestCase):
    def setUp(self):
        mock.patch("serial.tools.list_ports.comports", return_value=WINDOWS).start()
        mock.patch.object(E.glob, "glob", return_value=[]).start()
        self.addCleanup(mock.patch.stopall)

    def test_channels_sorted_by_letter(self):
        self.assertEqual(E.list_ch9344_ports(), ["COM7", "COM8", "COM10", "COM14"])

    def test_channel_letter(self):
        self.assertEqual(E.wch_channel(WINDOWS[0]), "D")
        self.assertIsNone(E.wch_channel(BLUETOOTH))
        self.assertIsNone(E.wch_channel(GIGA))

    def test_is_ch9344_port(self):
        self.assertTrue(E.is_ch9344_port("COM7"))
        self.assertFalse(E.is_ch9344_port("COM5"))
        self.assertFalse(E.is_ch9344_port("COM3"))
        self.assertFalse(E.is_ch9344_port(None))

    def test_single_port_wch_chip_is_not_a_channel(self):
        ch340 = com("COM20", "USB-SERIAL CH340 (COM20)", "wch.cn")
        with mock.patch("serial.tools.list_ports.comports", return_value=[ch340]):
            self.assertEqual(E.list_ch9344_ports(), [])


class LinuxDiscoveryTests(unittest.TestCase):
    def test_dev_nodes_sorted_by_number(self):
        nodes = {"/dev/ttyCH9344USB*": ["/dev/ttyCH9344USB10", "/dev/ttyCH9344USB2",
                                        "/dev/ttyCH9344USB0"],
                 "/dev/ttyCH343USB*": []}
        with mock.patch("serial.tools.list_ports.comports", return_value=[]), \
                mock.patch.object(E.glob, "glob", side_effect=lambda pat: nodes[pat]):
            self.assertEqual(E.list_ch9344_ports(), ["/dev/ttyCH9344USB0",
                                                     "/dev/ttyCH9344USB2",
                                                     "/dev/ttyCH9344USB10"])
            self.assertTrue(E.is_ch9344_port("/dev/ttyCH9344USB3"))


if __name__ == "__main__":
    unittest.main()
