"""app._muscles_state: one entry per regulator, named only when mapped."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app  # noqa: E402

MAP = {8: {"muscle": "FDP1", "sensor": 50, "region": "forearm"},
       11: {"muscle": "FPL", "sensor": None, "region": "forearm"},
       20: {"muscle": "DI1", "sensor": 75, "region": "palm"}}
ENC = {"encoders": [{"slave": 50, "position_mm": -1.25, "online": True}]}


class MusclesStateTests(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(app, "sensor_map", MAP)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_mapped_regulator_with_sensor(self):
        m = app._muscles_state(ENC)[8]
        self.assertEqual((m["muscle"], m["sensor"], m["position_mm"], m["sensor_online"]),
                         ("FDP1", 50, -1.25, True))

    def test_mapped_regulator_without_sensor(self):
        m = app._muscles_state(ENC)[11]
        self.assertEqual((m["muscle"], m["sensor"], m["position_mm"]), ("FPL", None, None))

    def test_unmapped_regulator_has_no_name(self):
        m = app._muscles_state(ENC)[0]
        self.assertIsNone(m["muscle"])
        self.assertEqual(len(app._muscles_state(None)), app.NUM_VALVES)

    def test_bar_band_follows_the_region(self):
        muscles = app._muscles_state(ENC)
        band = lambda reg: (muscles[reg]["bar_min_mm"], muscles[reg]["bar_max_mm"])  # noqa: E731
        self.assertEqual(band(20), (-4.0, 4.0))      # palmar
        self.assertEqual(band(8), (-10.0, 10.0))     # forearm
        self.assertEqual(band(0), (-10.0, 10.0))     # unmapped: default band


if __name__ == "__main__":
    unittest.main()
