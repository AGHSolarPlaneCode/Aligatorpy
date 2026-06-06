from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from Application.Logger.log_module import get_logger
from Application.Services.GiCameraService import GiCameraService
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.Services.led_detector import LedDetector
from Application.configuration.config_loader import cfg


class DetectionPipelineService:
    """
    Pipeline detekcji diod LED z telemetrią drona @ 10 Hz.

    Używa:
    - GiCameraService (GStreamer, 10 fps branch)
    - LedDetector
    - MissionService.process_target() — GPS/attitude pobierane w process_target()
    """

    DEFAULT_FPS = 10

    def __init__(
        self,
        drone: MatekService,
        camera: Optional[GiCameraService] = None,
        mission: Optional[MissionService] = None,
        fps: int = DEFAULT_FPS,
    ):
        self.logger = get_logger(__name__)
        self.drone = drone
        self.camera = camera or GiCameraService()
        self.mission = mission or MissionService(drone)
        self.led_detector = LedDetector(self.camera.WIDTH, self.camera.HEIGHT)
        self.fps = fps

    @staticmethod
    def load_search_zone() -> List[Tuple[float, float]]:
        zone_path = cfg.dirs.zones_dir / cfg.zones.search_zone_path
        return MissionService.load_Poly(zone_path)

    def _scale_pixel(self, x: int, y: int) -> Tuple[int, int]:
        sx = self.mission.image_width / self.camera.WIDTH
        sy = self.mission.image_height / self.camera.HEIGHT
        return int(x * sx), int(y * sy)

    def _process_frame(self, frame, is_bottle: bool) -> int:
        targets = self.led_detector.process_frame(frame)
        accepted = 0

        for target in targets:
            if target["frames_unseen"] != 0:
                continue

            pixel = self._scale_pixel(target["x"], target["y"])
            result = self.mission.process_target(pixel, is_bottle, self.mission.GEOFENCE)
            if result:
                accepted += 1

        return accepted

    @staticmethod
    def _sleep_until_next_frame(loop_start: float, interval: float) -> None:
        elapsed = time.monotonic() - loop_start
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)

    def run(
        self,
        stop_event=None,
        is_bottle: bool = True,
        geofence: Optional[List[Tuple[float, float]]] = None,
        max_frames: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        if stop_event is None:
            import threading
            stop_event = threading.Event()

        if geofence is None:
            geofence = self.load_search_zone()

        self.mission.GEOFENCE = geofence
        self.mission.TRG_CANDIDATES = []

        self.camera.start()
        self.camera.set_10fps_mode()
        self.led_detector.reset()

        interval = 1.0 / self.fps
        frame_count = 0
        self.logger.info(
            f"Starting LED detection pipeline @ {self.fps}Hz (is_bottle={is_bottle})"
        )

        while not stop_event.is_set():
            loop_start = time.monotonic()

            frame, _ = self.camera.get_frame()
            if frame is not None:
                accepted = self._process_frame(frame, is_bottle)
                frame_count += 1

                if accepted:
                    self.logger.info(
                        f"Frame {frame_count}: accepted {accepted} detection(s), "
                        f"candidates={len(self.mission.TRG_CANDIDATES)}"
                    )

            if max_frames is not None and frame_count >= max_frames:
                break

            self._sleep_until_next_frame(loop_start, interval)

        targets = list(self.mission.TRG_CANDIDATES)
        self.logger.info(f"Pipeline finished with {len(targets)} target candidate(s)")
        return targets


def run_led_detection_pipeline(
    stop_event,
    is_bottle: bool = True,
    fps: int = DetectionPipelineService.DEFAULT_FPS,
    device: Optional[str] = None,
    baud: Optional[int] = None,
    geofence: Optional[List[Tuple[float, float]]] = None,
    max_frames: Optional[int] = None,
    result_queue=None,
    camera: Optional[GiCameraService] = None,
) -> List[Dict[str, Any]]:
    """
    Entry point for running detection in a thread (shares camera with main process).
    """
    pipeline_device = device or cfg.mav.device2
    drone = MatekService(device=pipeline_device, baud=baud or cfg.mav.baud)
    pipeline = DetectionPipelineService(drone=drone, camera=camera, fps=fps)
    try:
        targets = pipeline.run(
            stop_event=stop_event,
            is_bottle=is_bottle,
            geofence=geofence,
            max_frames=max_frames,
        )
        if result_queue is not None:
            result_queue.put(targets)
        return targets
    finally:
        drone.close()
