import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "Application"
    / "Services"
    / "LedFrequencyDetectionService.py"
)
SPEC = importlib.util.spec_from_file_location("led_frequency_detection_service", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
LedFrequencyDetectionService = MODULE.LedFrequencyDetectionService
LedDetection = MODULE.LedDetection


class LedFrequencyDetectionServiceTest(unittest.TestCase):
    def setUp(self):
        self.fps = 60
        self.width = 320
        self.height = 200
        self.service = LedFrequencyDetectionService(
            led_frequencies=[2, 6, 10, 14],
            fps=self.fps,
            camera_resolution=(self.width, self.height),
            drone_speed_mps=5.0,
            field_width_m=70.0,
            brightness_threshold=128,
            analysis_duration_s=2.0,
            analysis_square_size_m=6.0,
            min_blob_area_px=1,
            max_blob_area_px=25,
        )

    def _frame(self, timestamp, leds):
        frame = np.full((self.height, self.width), 25, dtype=np.uint8)
        velocity = self.service.image_velocity_y_px_s
        for x, initial_y, frequency in leds:
            y = int(round(initial_y + velocity * timestamp))
            is_on = (timestamp * frequency) % 1.0 < 0.5
            if is_on and 0 <= y < self.height:
                frame[max(0, y - 2) : min(self.height, y + 3), x - 2 : x + 3] = 220
        return frame.reshape(-1)

    def test_rejects_non_flat_frame(self):
        frame = np.zeros((self.height, self.width), dtype=np.uint8)

        with self.assertRaises(ValueError):
            self.service.process_frame(frame, 0.0)

    def test_detects_requested_frequencies_on_moving_leds(self):
        leds = [
            (50, 40, 2),
            (105, 60, 6),
            (160, 80, 10),
            (215, 100, 14),
        ]
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            detections.extend(self.service.process_frame(self._frame(timestamp, leds), timestamp))

        self.assertEqual({d.frequency_hz for d in detections}, {2.0, 6.0, 10.0, 14.0})

    def test_ignores_unrequested_frequency(self):
        leds = [(100, 50, 7)]
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            detections.extend(self.service.process_frame(self._frame(timestamp, leds), timestamp))

        self.assertEqual(detections, [])

    def test_matches_small_frequency_error_with_tolerance(self):
        leds = [(100, 50, 6.2)]
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            detections.extend(self.service.process_frame(self._frame(timestamp, leds), timestamp))

        self.assertEqual([d.frequency_hz for d in detections], [6.0])

    def test_ignores_repeated_camera_frame_timestamp(self):
        frame = self._frame(0.0, [(100, 50, 6)])
        self.service.process_frame(frame, 0.0)
        self.service.process_frame(frame, 0.0)

        self.assertEqual(len(self.service._tracks[0].samples), 1)

    def test_rejects_square_that_can_contain_two_sites(self):
        with self.assertRaises(ValueError):
            LedFrequencyDetectionService(
                led_frequencies=[6],
                analysis_square_size_m=8.0,
                minimum_site_distance_m=10.0,
            )

    def test_projects_detection_with_full_drone_attitude(self):
        class Drone:
            def get_current_coordinates(self):
                return 50.0, 20.0, 60.0

            def get_attitude(self):
                return 0.1, 0.2, 0.3

        class Mission:
            received = None

            def project_target_cords(self, *args):
                self.received = args
                return 50.1, 20.1

        mission = Mission()
        detection = LedDetection(1, 6.0, (100, 120), 0.3, 10.0)

        projected = self.service.project_detection(detection, Drone(), mission)

        self.assertEqual(
            mission.received,
            ((100, 120), 50.0, 20.0, 60.0, 0.1, 0.2, 0.3),
        )
        self.assertEqual(projected.coordinates, (50.1, 20.1))

    def test_allows_detection_retry_after_failed_projection(self):
        frame = self._frame(0.0, [(100, 50, 6)])
        self.service.process_frame(frame, 0.0)
        track = self.service._tracks[0]
        track.detected_frequency = 6.0

        self.service._allow_detection_retry(track.track_id)

        self.assertIsNone(track.detected_frequency)


if __name__ == "__main__":
    unittest.main()
