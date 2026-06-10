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


class LedFrequencyDetectionServiceTest(unittest.TestCase):
    def setUp(self):
        self.fps = 60
        self.width = 640
        self.height = 400
        self.service = LedFrequencyDetectionService(
            led_frequencies=[4, 8, 12, 16],
            fps=self.fps,
            camera_resolution=(self.width, self.height),
            drone_speed_mps=5.0,
            field_width_m=70.0,
            analysis_duration_s=2.0,
            roi_size_m=6.0,
            min_blob_area_px=1,
            max_blob_area_px=25,
        )

    def _frame(self, timestamp, leds, wind_px=0.0, drone_speed_mps=None):
        frame = np.full((self.height, self.width), 25, dtype=np.uint8)
        speed = self.service.drone_speed_mps if drone_speed_mps is None else drone_speed_mps
        velocity = (
            self.service.image_motion_direction
            * speed
            / self.service.meters_per_pixel
        )
        for x, initial_y, frequency in leds:
            led_x = int(round(x + wind_px))
            led_y = int(round(initial_y + velocity * timestamp))
            if (timestamp * frequency) % 1.0 < 0.5 and 0 <= led_y < self.height:
                frame[led_y, led_x] = 255
        return (frame.astype(np.uint16) << 8).reshape(-1)

    def test_detects_only_requested_frequencies(self):
        leds = [
            (60, 40, 2),
            (170, 60, 4),
            (280, 80, 8),
            (390, 100, 12),
            (500, 120, 16),
        ]
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            detections.extend(self.service.process_frame(self._frame(timestamp, leds), timestamp))

        self.assertEqual({item.frequency_hz for item in detections}, {4, 8, 12, 16})

    def test_tracks_led_moving_inside_roi_due_to_wind(self):
        detections = []
        leds = [(250, 60, 8)]

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            wind_px = 15 * np.sin(2 * np.pi * timestamp / 1.4)
            detections.extend(
                self.service.process_frame(
                    self._frame(timestamp, leds, wind_px=wind_px),
                    timestamp,
                )
            )

        self.assertEqual([item.frequency_hz for item in detections], [8])

    def test_rejects_random_flashes(self):
        rng = np.random.default_rng(seed=7)
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            frame = np.full((self.height, self.width), 25 << 8, dtype=np.uint16)
            if rng.random() < 0.5:
                frame[100, 100] = 255 << 8
            detections.extend(self.service.process_frame(frame.reshape(-1), timestamp))

        self.assertEqual(detections, [])

    def test_tracks_led_using_changing_drone_speed(self):
        frequency = 8
        initial_x = 250
        initial_y = 60
        traveled_m = 0.0
        detections = []

        for index in range(int(self.fps * 2.2)):
            timestamp = index / self.fps
            current_speed_mps = 5.0 + 2.0 * np.sin(2 * np.pi * timestamp / 0.8)
            if index > 0:
                traveled_m += current_speed_mps / self.fps

            frame = np.full((self.height, self.width), 25, dtype=np.uint8)
            led_y = int(round(initial_y + traveled_m / self.service.meters_per_pixel))
            if (timestamp * frequency) % 1.0 < 0.5 and led_y < self.height:
                frame[led_y, initial_x] = 255

            detections.extend(
                self.service.process_frame(
                    (frame.astype(np.uint16) << 8).reshape(-1),
                    timestamp,
                    drone_speed_mps=current_speed_mps,
                )
            )

        self.assertEqual([item.frequency_hz for item in detections], [frequency])
        self.assertEqual(self.service._next_track_id, 1)


if __name__ == "__main__":
    unittest.main()
