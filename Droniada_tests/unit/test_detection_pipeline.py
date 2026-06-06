import os
import sys
import threading
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.DetectionPipelineService import DetectionPipelineService


class TestDetectionPipelineService(unittest.TestCase):
    def _make_pipeline(self):
        drone = MagicMock()
        camera = MagicMock()
        camera.WIDTH = 1280
        camera.HEIGHT = 800
        mission = MagicMock()
        mission.image_width = 2304
        mission.image_height = 1296
        mission.TRG_CANDIDATES = []
        mission.GEOFENCE = []
        mission.process_target.return_value = True

        pipeline = DetectionPipelineService(drone=drone, camera=camera, mission=mission, fps=10)
        return pipeline, drone, camera, mission

    def test_scale_pixel(self):
        pipeline, _, camera, _ = self._make_pipeline()
        x, y = pipeline._scale_pixel(640, 400)
        self.assertEqual(x, 1152)
        self.assertEqual(y, 648)

    def test_process_frame_calls_process_target_for_new_detections(self):
        pipeline, _, _, mission = self._make_pipeline()
        frame = np.zeros((800, 1280), dtype=np.uint8)

        with patch.object(pipeline.led_detector, "process_frame") as mock_detect:
            mock_detect.return_value = [
                {"id": 0, "x": 100, "y": 200, "frames_unseen": 0},
                {"id": 1, "x": 300, "y": 400, "frames_unseen": 1},
            ]
            accepted = pipeline._process_frame(frame, is_bottle=True)

        self.assertEqual(accepted, 1)
        mission.process_target.assert_called_once()

    def test_run_stops_on_max_frames(self):
        pipeline, drone, camera, mission = self._make_pipeline()
        stop = threading.Event()

        frame = np.zeros((800, 1280), dtype=np.uint8)
        camera.get_frame.return_value = (frame, 0.0)

        with patch.object(pipeline.led_detector, "process_frame", return_value=[]):
            result = pipeline.run(stop_event=stop, geofence=[], max_frames=3)

        self.assertEqual(result, [])
        camera.start.assert_called_once()
        camera.set_10fps_mode.assert_called_once()
        drone.set_telemetry_rate.assert_not_called()
        drone.set_mission_current_rate.assert_not_called()


if __name__ == "__main__":
    unittest.main()
