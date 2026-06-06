import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.MissionPlannerService import MissionPlannerService
from Application.configuration.config_loader import LoiterConfig


class TestMissionPlannerService(unittest.TestCase):
    def setUp(self):
        self.planner = MissionPlannerService()
        self.loiter = LoiterConfig(time=15, alt=60, radius=50)

    def test_order_targets_nearest(self):
        targets = [
            {"lat": 50.0, "lon": 19.0, "count": 1},
            {"lat": 50.001, "lon": 19.0, "count": 1},
            {"lat": 50.01, "lon": 19.01, "count": 1},
        ]
        ordered = self.planner.order_targets_nearest(50.0, 19.0, targets)
        self.assertEqual(len(ordered), 3)
        self.assertAlmostEqual(ordered[0]["lat"], 50.001, places=4)

    def test_build_loiter_waypoints(self):
        targets = [{"lat": 50.0, "lon": 19.0, "count": 1}]
        wps = self.planner.build_loiter_waypoints(targets, self.loiter)
        self.assertEqual(len(wps), 1)
        self.assertEqual(wps[0]["command"], "NAV_LOITER_TIME")
        self.assertEqual(wps[0]["time"], 15)
        self.assertEqual(wps[0]["alt"], 60)

    def test_build_landing_sites_filters_by_confidence(self):
        targets = [
            {"lat": 50.0, "lon": 19.0},
            {"lat": 50.1, "lon": 19.1},
        ]
        ook_results = [
            {"freq": 5.0, "confidence": 5.0},
            {"freq": None, "confidence": 1.0},
        ]
        sites = self.planner.build_landing_sites(targets, ook_results, min_confidence=4.0)
        self.assertEqual(len(sites), 1)
        self.assertEqual(sites[0], (50.0, 19.0))


if __name__ == "__main__":
    unittest.main()
