import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.OOK_detection.ook import classify_ook, ook_brightness


class TestOok(unittest.TestCase):
    def test_brightness_counts_hot_pixels(self):
        roi = np.zeros((20, 20), dtype=np.uint8)
        roi[5:10, 5:10] = 255
        self.assertEqual(ook_brightness(roi, thr=220), 25)

    def test_classify_detects_5hz_signal(self):
        candidates = [5.0, 7.0, 9.0]
        t = np.linspace(0, 2, 240)
        signal = (np.sin(2 * np.pi * 5.0 * t) > 0).astype(float) * 100
        freq, conf = classify_ook(signal, t, candidates, min_confidence=2.0)
        self.assertEqual(freq, 5.0)
        self.assertGreater(conf, 2.0)

    def test_classify_rejects_flat_signal(self):
        t = np.linspace(0, 1, 50)
        signal = np.full(50, 50.0)
        freq, conf = classify_ook(signal, t, [5.0, 7.0], min_confidence=4.0)
        self.assertIsNone(freq)

    def test_classify_rejects_too_few_samples(self):
        freq, conf = classify_ook([1, 2], [0, 1], [5.0], min_confidence=4.0)
        self.assertIsNone(freq)
        self.assertEqual(conf, 0.0)


if __name__ == "__main__":
    unittest.main()
