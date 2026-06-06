import os
import sys
import unittest
from multiprocessing import Queue

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.OOK_detection.ook_worker import run_ook_worker


class TestOokWorker(unittest.TestCase):
    def test_worker_classifies_samples_from_queue(self):
        sample_queue = Queue()
        result_queue = Queue()

        t = np.linspace(0, 1, 100)
        signal = (np.sin(2 * np.pi * 7.0 * t) > 0).astype(float) * 80

        for brightness, ts in zip(signal, t):
            sample_queue.put((float(brightness), float(ts)))
        sample_queue.put(None)

        run_ook_worker(
            sample_queue,
            result_queue,
            duration_s=1,
            candidates=[5.0, 7.0, 9.0],
            min_confidence=2.0,
        )

        result = result_queue.get(timeout=2)
        self.assertEqual(result["freq"], 7.0)
        self.assertGreater(result["confidence"], 2.0)
        self.assertEqual(result["samples"], 100)


if __name__ == "__main__":
    unittest.main()
