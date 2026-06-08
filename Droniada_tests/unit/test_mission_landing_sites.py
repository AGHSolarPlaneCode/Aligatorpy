import os
import sys
import unittest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.MissionService import MissionService


class TestProcessLandingSites(unittest.TestCase):
    def test_process_landing_sites_appends_waypoints(self):
        drone = MagicMock()
        drone.get_attitude.return_value = (0.0, 0.0, 1.57)
        drone.append_waypoints.return_value = True

        mission = MissionService(drone)
        mission.calc_drop_coords = MagicMock(
            return_value={"lat": 50.0, "lon": 19.0, "isBottle": False}
        )
        mission.calc_drop_waypoints = MagicMock(
            side_effect=lambda dp, yaw, container: container.append({"command": "WAYPOINT"})
        )

        ok = mission.process_landing_sites([(50.0, 19.0), (50.1, 19.1)])
        self.assertTrue(ok)
        self.assertEqual(mission.calc_drop_waypoints.call_count, 2)
        drone.append_waypoints.assert_called_once()

    def test_process_landing_sites_empty_returns_false(self):
        drone = MagicMock()
        mission = MissionService(drone)
        self.assertFalse(mission.process_landing_sites([]))


if __name__ == "__main__":
    unittest.main()
