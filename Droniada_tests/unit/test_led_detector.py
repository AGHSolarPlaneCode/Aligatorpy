import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.led_detector import LedDetector


def _blank_frame(w: int, h: int) -> np.ndarray:
    return np.zeros((h, w), dtype=np.uint8)


def _add_bright_spot(frame: np.ndarray, cx: int, cy: int, radius: int = 3) -> None:
    y, x = np.ogrid[: frame.shape[0], : frame.shape[1]]
    mask = (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2
    frame[mask] = 255


class TestLedDetector(unittest.TestCase):
    def setUp(self):
        self.detector = LedDetector(128, 80)

    def test_detects_bright_spot(self):
        frame = _blank_frame(128, 80)
        _add_bright_spot(frame, 40, 30, radius=4)

        targets = self.detector.process_frame(frame)
        self.assertGreater(len(targets), 0)
        self.assertEqual(targets[-1]["frames_unseen"], 0)

    def test_merge_same_spot_across_frames(self):
        frame1 = _blank_frame(128, 80)
        _add_bright_spot(frame1, 50, 40, radius=4)
        t1 = self.detector.process_frame(frame1)
        first_id = t1[-1]["id"]

        frame2 = _blank_frame(128, 80)
        _add_bright_spot(frame2, 52, 41, radius=4)
        t2 = self.detector.process_frame(frame2)

        ids = {t["id"] for t in t2 if t["frames_unseen"] == 0}
        self.assertIn(first_id, ids)

    def test_no_detection_on_empty_frame(self):
        frame = _blank_frame(128, 80)
        targets = self.detector.process_frame(frame)
        visible = [t for t in targets if t["frames_unseen"] == 0]
        self.assertEqual(len(visible), 0)

    def test_reset_clears_state(self):
        frame = _blank_frame(128, 80)
        _add_bright_spot(frame, 10, 10, radius=4)
        self.detector.process_frame(frame)
        self.detector.reset()
        self.assertEqual(self.detector._detected_targets, [])


if __name__ == "__main__":
    unittest.main()
