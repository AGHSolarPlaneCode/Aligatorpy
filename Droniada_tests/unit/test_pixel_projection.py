"""Testy projekcji piksel → ziemia przez process_one_frame / process_target."""

from __future__ import annotations

import math
import os
import sys
import unittest
from contextlib import contextmanager
from unittest.mock import patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Droniada_tests.projection_test_utils import (
    huge_geofence,
    latlon_to_meters,
    make_detection_pipeline,
    make_mock_drone,
    project_grid_via_process_one_frame,
    project_via_process_one_frame,
    scale_pixel,
)
from Application.Services.gi_camera_handler import HEIGHT, WIDTH


DEFAULT_LAT = 52.2297
DEFAULT_LON = 21.0122
DEFAULT_ALT = 50.0


@contextmanager
def suppress_projection_prints():
    with patch("builtins.print"):
        yield


class TestPixelProjection(unittest.TestCase):
    def setUp(self):
        self.drone = make_mock_drone(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT)
        self.pipeline = make_detection_pipeline(self.drone)
        self.geofence = huge_geofence(DEFAULT_LAT, DEFAULT_LON)

    def test_scale_pixel_matches_process_one_frame(self):
        cam_x, cam_y = WIDTH // 2, HEIGHT // 2
        expected = scale_pixel(self.pipeline.mission, self.pipeline.camera, cam_x, cam_y)

        with suppress_projection_prints():
            with patch.object(self.pipeline.led_detector, "process_frame") as mock_detect:
                mock_detect.return_value = [
                    {"id": 0, "x": cam_x, "y": cam_y, "frames_unseen": 0}
                ]
                with patch.object(self.pipeline.mission, "process_target", return_value=True) as mock_pt:
                    self.pipeline.process_one_frame(is_bottle=True)

        mock_pt.assert_called_once()
        self.assertEqual(mock_pt.call_args[0][0], expected)

    def test_center_pixel_near_nadir_at_zero_tilt(self):
        cam_x, cam_y = WIDTH // 2, HEIGHT // 2
        with suppress_projection_prints():
            lat, lon = project_via_process_one_frame(
                self.pipeline, cam_x, cam_y, self.geofence
            )

        self.assertIsNotNone(lat)
        self.assertIsNotNone(lon)
        east, north = latlon_to_meters(lat, lon, DEFAULT_LAT, DEFAULT_LON)
        self.assertLess(abs(east), 5.0, "środek kadru powinien trafić blisko nadiru (E)")
        self.assertLess(abs(north), 5.0, "środek kadru powinien trafić blisko nadiru (N)")

    def test_grid_footprint_has_expected_span_at_50m(self):
        n_cols, n_rows = 10, 10
        with suppress_projection_prints():
            grid = project_grid_via_process_one_frame(
                self.pipeline, n_cols, n_rows, self.geofence
            )

        valid = [(p.lat, p.lon) for p in grid if p.lat is not None and p.lon is not None]
        self.assertEqual(len(valid), n_cols * n_rows)

        easts, norths = zip(
            *(latlon_to_meters(lat, lon, DEFAULT_LAT, DEFAULT_LON) for lat, lon in valid)
        )
        span_e = max(easts) - min(easts)
        span_n = max(norths) - min(norths)

        fx = float(self.pipeline.mission.K[0, 0])
        img_w = self.pipeline.mission.image_width
        expected_span = img_w * DEFAULT_ALT / fx

        self.assertGreater(span_e, expected_span * 0.5)
        self.assertLess(span_e, expected_span * 1.5)
        self.assertGreater(span_n, expected_span * 0.3)
        self.assertLess(span_n, expected_span * 1.5)

    def test_zero_tilt_grid_is_roughly_symmetric_around_nadir(self):
        with suppress_projection_prints():
            grid = project_grid_via_process_one_frame(self.pipeline, 10, 10, self.geofence)

        easts, norths = [], []
        for p in grid:
            if p.lat is None:
                continue
            e, n = latlon_to_meters(p.lat, p.lon, DEFAULT_LAT, DEFAULT_LON)
            easts.append(e)
            norths.append(n)

        self.assertAlmostEqual(sum(easts) / len(easts), 0.0, delta=8.0)
        self.assertAlmostEqual(sum(norths) / len(norths), 0.0, delta=8.0)

    def test_roll_tilts_footprint(self):
        drone_level = make_mock_drone(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, roll_deg=0.0)
        drone_rolled = make_mock_drone(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, roll_deg=15.0)

        pipe_level = make_detection_pipeline(drone_level)
        pipe_rolled = make_detection_pipeline(drone_rolled)
        geofence = huge_geofence(DEFAULT_LAT, DEFAULT_LON)

        corner_x = int(WIDTH * 0.9)
        corner_y = HEIGHT // 2

        with suppress_projection_prints():
            lat0, lon0 = project_via_process_one_frame(pipe_level, corner_x, corner_y, geofence)
            lat1, lon1 = project_via_process_one_frame(pipe_rolled, corner_x, corner_y, geofence)

        self.assertIsNotNone(lat0)
        self.assertIsNotNone(lat1)
        e0, n0 = latlon_to_meters(lat0, lon0, DEFAULT_LAT, DEFAULT_LON)
        e1, n1 = latlon_to_meters(lat1, lon1, DEFAULT_LAT, DEFAULT_LON)
        shift = math.hypot(e1 - e0, n1 - n0)
        self.assertGreater(shift, 1.0, "roll=15° powinien przesunąć projekcję bocznego piksela")

    def test_pitch_shifts_footprint_forward(self):
        drone_level = make_mock_drone(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, pitch_deg=0.0)
        drone_pitched = make_mock_drone(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, pitch_deg=10.0)

        pipe_level = make_detection_pipeline(drone_level)
        pipe_pitched = make_detection_pipeline(drone_pitched)
        geofence = huge_geofence(DEFAULT_LAT, DEFAULT_LON)

        cam_x, cam_y = WIDTH // 2, int(HEIGHT * 0.25)

        with suppress_projection_prints():
            lat0, lon0 = project_via_process_one_frame(pipe_level, cam_x, cam_y, geofence)
            lat1, lon1 = project_via_process_one_frame(pipe_pitched, cam_x, cam_y, geofence)

        self.assertIsNotNone(lat0)
        self.assertIsNotNone(lat1)
        _, n0 = latlon_to_meters(lat0, lon0, DEFAULT_LAT, DEFAULT_LON)
        _, n1 = latlon_to_meters(lat1, lon1, DEFAULT_LAT, DEFAULT_LON)
        self.assertNotAlmostEqual(n0, n1, delta=0.5)


if __name__ == "__main__":
    unittest.main()
