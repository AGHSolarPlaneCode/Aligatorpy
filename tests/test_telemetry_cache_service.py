import importlib.util
import sys
import time
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "Application"
    / "Services"
    / "TelemetryCacheService.py"
)
SPEC = importlib.util.spec_from_file_location("telemetry_cache_service", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
TelemetryCacheService = MODULE.TelemetryCacheService


class TelemetryCacheServiceTest(unittest.TestCase):
    def test_updates_snapshot_at_limited_rate(self):
        class Drone:
            position_calls = 0
            attitude_calls = 0

            def get_current_position_data(self, timeout):
                self.position_calls += 1
                return (50.0, 20.0, 60.0), 5.5

            def get_attitude(self, timeout):
                self.attitude_calls += 1
                return 0.1, 0.2, 0.3

        drone = Drone()
        cache = TelemetryCacheService(drone, rate_hz=10)

        cache.start()
        time.sleep(0.26)
        cache.stop()

        snapshot = cache.get_latest()
        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.coordinates, (50.0, 20.0, 60.0))
        self.assertEqual(snapshot.attitude, (0.1, 0.2, 0.3))
        self.assertEqual(snapshot.ground_speed_mps, 5.5)
        self.assertLessEqual(drone.position_calls, 3)
        self.assertEqual(drone.position_calls, drone.attitude_calls)

    def test_rejects_rate_above_ten_hz(self):
        with self.assertRaises(ValueError):
            TelemetryCacheService(object(), rate_hz=11)


if __name__ == "__main__":
    unittest.main()
