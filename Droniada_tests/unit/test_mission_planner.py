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
        self.assertAlmostEqual(ordered[0]["lat"], 50.0, places=4)   # najbliższy startowi
        self.assertAlmostEqual(ordered[1]["lat"], 50.001, places=4) # drugi
        self.assertAlmostEqual(ordered[2]["lat"], 50.01, places=4)  # najdalszy
    def test_build_loiter_waypoints(self):
        targets = [{"lat": 50.0, "lon": 19.0, "count": 1}]
        wps = self.planner.build_loiter_waypoints(targets, self.loiter)
        self.assertEqual(len(wps), 1)
        self.assertEqual(wps[0]["command"], "NAV_LOITER_TIME")
        self.assertEqual(wps[0]["time"], 15)
        self.assertEqual(wps[0]["alt"], 60)

    def test_build_approach_and_loiter_waypoints(self):
        targets = [
            {"lat": 50.0, "lon": 19.0},
            {"lat": 50.1, "lon": 19.1},
        ]
        wps = self.planner.build_approach_and_loiter_waypoints(targets, self.loiter)
        self.assertEqual(len(wps), 4)
        self.assertEqual(wps[0]["command"], "WAYPOINT")
        self.assertEqual(wps[1]["command"], "NAV_LOITER_TIME")
        self.assertEqual(wps[1]["time"], 15)
        self.assertEqual(wps[3]["command"], "NAV_LOITER_TIME")

    def test_loiter_wp_indices(self):
        indices = self.planner.loiter_wp_indices(first_approach_wp=10, site_count=3)
        self.assertEqual(indices, [11, 13, 15])

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

    def test_select_desired_landing_sites_picks_by_freq_and_confidence(self):
        targets = [
            {"lat": 50.0, "lon": 19.0},
            {"lat": 50.1, "lon": 19.1},
            {"lat": 50.2, "lon": 19.2},
            {"lat": 50.3, "lon": 19.3},
        ]
        ook_results = [
            {"freq": 4.0, "confidence": 5.0},
            {"freq": 8.0, "confidence": 6.0},
            {"freq": 10.0, "confidence": 4.5},
            {"freq": 12.0, "confidence": 7.0},
        ]
        sites = self.planner.select_desired_landing_sites(
            targets, ook_results, desired=(4.0, 8.0, 10.0, 12.0), min_confidence=4.0
        )
        self.assertEqual(
            sites,
            [(50.0, 19.0), (50.1, 19.1), (50.2, 19.2), (50.3, 19.3)],
        )

    def test_select_desired_landing_sites_duplicate_freq_picks_top_two(self):
        targets = [
            {"lat": 50.0, "lon": 19.0},
            {"lat": 50.1, "lon": 19.1},
            {"lat": 50.2, "lon": 19.2},
        ]
        ook_results = [
            {"freq": 4.0, "confidence": 5.0},
            {"freq": 4.0, "confidence": 8.0},
            {"freq": 4.0, "confidence": 6.0},
        ]
        sites = self.planner.select_desired_landing_sites(
            targets, ook_results, desired=(4.0, 4.0), min_confidence=4.0
        )
        self.assertEqual(sites, [(50.1, 19.1), (50.2, 19.2)])

    def test_select_desired_landing_sites_skips_missing_freq(self):
        targets = [
            {"lat": 50.0, "lon": 19.0},
            {"lat": 50.1, "lon": 19.1},
        ]
        ook_results = [
            {"freq": 4.0, "confidence": 5.0},
            {"freq": None, "confidence": 0.0},
        ]
        sites = self.planner.select_desired_landing_sites(
            targets, ook_results, desired=(4.0, 8.0), min_confidence=4.0
        )
        self.assertEqual(sites, [(50.0, 19.0)])


if __name__ == "__main__":
    unittest.main()
