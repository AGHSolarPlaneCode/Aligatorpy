import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.OokDetectionService import OokDetectionService
from Application.configuration.config_loader import OokConfig


class TestOokDetectionService(unittest.TestCase):
    def test_extract_center_roi_from_grayscale(self):
        frame = np.arange(100 * 100, dtype=np.uint8).reshape(100, 100)
        roi = OokDetectionService.extract_center_roi(frame, roi_size=20)
        self.assertEqual(roi.shape, (20, 20))

    @patch("Application.Services.OokDetectionService.Process")
    def test_detect_modulation_returns_result(self, mock_process_cls):
        camera = MagicMock()
        camera.get_image.return_value = (np.full((80, 128), 255, dtype=np.uint8), 0.0, None)

        mock_worker = MagicMock()
        mock_process_cls.return_value = mock_worker

        result_queue = MagicMock()
        result_queue.empty.return_value = False
        result_queue.get.return_value = {"freq": 5.0, "confidence": 6.0, "samples": 10}

        ook_cfg = OokConfig(
            duration_s=0.01,
            candidates=(5.0, 7.0),
            min_confidence=4.0,
            roi_size=20,
            brightness_threshold=220,
        )

        with patch("Application.Services.OokDetectionService.Queue", side_effect=[MagicMock(), result_queue]):
            service = OokDetectionService(camera, ook_cfg)
            result = service.detect_modulation()

        self.assertEqual(result["freq"], 5.0)
        camera.set_120fps_active.assert_called_once_with(True)
        camera.set_10fps_active.assert_called_once_with(True)


if __name__ == "__main__":
    unittest.main()
