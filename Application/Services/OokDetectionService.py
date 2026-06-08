from __future__ import annotations

import time
from multiprocessing import Process, Queue
from typing import Any, Dict, Optional

import numpy as np

from Application.Logger.log_module import get_logger
from Application.OOK_detection.ook import ook_brightness
from Application.OOK_detection.ook_worker import run_ook_worker
from Application.Services.gi_camera_handler import CameraPipeline
from Application.configuration.config_loader import OokConfig, cfg


class OokDetectionService:
    def __init__(self, camera: CameraPipeline, ook_config: Optional[OokConfig] = None):
        self.logger = get_logger(__name__)
        self.camera = camera
        self.config = ook_config or cfg.mission.ook

    @staticmethod
    def extract_center_roi(frame: np.ndarray, roi_size: int) -> np.ndarray:
        if frame.ndim == 1:
            side = int(np.sqrt(frame.size))
            frame = frame.reshape(side, side)

        h, w = frame.shape[:2]
        half = roi_size // 2
        cx, cy = w // 2, h // 2
        y1 = max(0, cy - half)
        y2 = min(h, cy + half)
        x1 = max(0, cx - half)
        x2 = min(w, cx + half)
        return frame[y1:y2, x1:x2]

    def detect_modulation(self) -> Dict[str, Any]:
        self.camera.set_120fps_active(True)

        sample_queue = Queue()
        result_queue = Queue()
        worker = Process(
            target=run_ook_worker,
            args=(
                sample_queue,
                result_queue,
                self.config.duration_s,
                list(self.config.candidates),
                self.config.min_confidence,
            ),
            daemon=True,
        )
        worker.start()

        deadline = time.monotonic() + self.config.duration_s
        samples_sent = 0
        while time.monotonic() < deadline:
            frame, ts, _ = self.camera.get_image()
            if frame is not None and ts is not None:
                roi = self.extract_center_roi(frame, self.config.roi_size)
                brightness = ook_brightness(roi, self.config.brightness_threshold)
                sample_queue.put((brightness, ts))
                samples_sent += 1

        sample_queue.put(None)
        worker.join(timeout=5)

        self.camera.set_10fps_active(True)

        if result_queue.empty():
            self.logger.warning("OOK worker returned no result")
            return {"freq": None, "confidence": 0.0, "samples": samples_sent}

        result = result_queue.get()
        self.logger.info(
            f"OOK result: freq={result['freq']}Hz, "
            f"confidence={result['confidence']:.2f}, samples={result['samples']}"
        )
        return result
