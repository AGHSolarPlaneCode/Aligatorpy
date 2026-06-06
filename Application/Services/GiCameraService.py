from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import numpy as np

from Application.Logger.log_module import get_logger
from Application.Services.gi_camera_handler import CameraPipeline, HEIGHT, WIDTH


class GiCameraService:
    """Thread-safe adapter over GStreamer CameraPipeline."""

    def __init__(self):
        self.logger = get_logger(__name__)
        self._pipeline: Optional[CameraPipeline] = None
        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._pipeline = CameraPipeline()
        self._started.clear()

        def _run():
            self._started.set()
            self._pipeline.start()

        self._thread = threading.Thread(target=_run, daemon=True, name="GiCameraPipeline")
        self._thread.start()

        if not self._started.wait(timeout=10):
            raise RuntimeError("Camera pipeline failed to start within 10s")

        # Give pipeline time to reach PLAYING and receive first frame
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            frame, _, err = self._pipeline.get_image()
            if frame is not None:
                self.logger.info("Camera pipeline ready")
                return
            time.sleep(0.1)

        self.logger.warning("Camera pipeline started but no frame yet")

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._pipeline = None
        self._thread = None

    def set_10fps_mode(self) -> None:
        self._ensure_running()
        self._pipeline.set_10fps_active(True)

    def set_120fps_mode(self) -> None:
        self._ensure_running()
        self._pipeline.set_120fps_active(True)

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[float]]:
        self._ensure_running()
        frame, ts, err = self._pipeline.get_image()
        if err:
            self.logger.debug(f"get_frame: {err}")
        return frame, ts

    def _ensure_running(self) -> None:
        if self._pipeline is None or self._thread is None or not self._thread.is_alive():
            raise RuntimeError("Camera pipeline is not running — call start() first")
