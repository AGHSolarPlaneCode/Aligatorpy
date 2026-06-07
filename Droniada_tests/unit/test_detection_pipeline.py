import os
import sys
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

    def test_scale_pixel_via_process_one_frame(self):
        pipeline, _, camera, mission = self._make_pipeline()
        frame = np.zeros((800, 1280), dtype=np.uint8)

        with patch.object(pipeline.led_detector, "process_frame") as mock_detect:
            mock_detect.return_value = [{"id": 0, "x": 640, "y": 400, "frames_unseen": 0}]
            camera.get_image.return_value = (frame, 0.0, None)
            accepted = pipeline.process_one_frame(is_bottle=True)

        self.assertEqual(accepted, 1)
        mission.process_target.assert_called_once()
        args = mission.process_target.call_args[0]
        self.assertEqual(args[0], (1152, 648))

    def test_process_one_frame_skips_when_no_frame(self):
        pipeline, _, camera, mission = self._make_pipeline()
        camera.get_image.return_value = (None, None, "No active branch")
        accepted = pipeline.process_one_frame()
        self.assertEqual(accepted, 0)
        mission.process_target.assert_not_called()

    def test_run_uses_should_stop_callback(self):
        pipeline, _, camera, mission = self._make_pipeline()
        frame = np.zeros((800, 1280), dtype=np.uint8)
        camera.get_image.return_value = (frame, 0.0, None)

        calls = {"n": 0}

        def should_stop():
            calls["n"] += 1
            return calls["n"] >= 3

        with patch.object(pipeline.led_detector, "process_frame", return_value=[]):
            pipeline.run(should_stop=should_stop, geofence=[], start_camera=False)

        self.assertEqual(calls["n"], 3)


if __name__ == "__main__":
    unittest.main()
