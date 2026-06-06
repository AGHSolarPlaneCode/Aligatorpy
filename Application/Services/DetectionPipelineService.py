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
    Pipeline detekcji diod LED @ 10 Hz.

    Wymaga jednego połączenia MAVLink — wywołuj process_one_frame() wyłącznie
    z głównego wątku (process_target() czyta GPS/attitude z MatekService).

    Kamera: GiCameraService (gi_camera_handler) — współdzielona z resztą misji.
    """

    DEFAULT_FPS = 10

    def __init__(
        self,
        drone: MatekService,
        camera: GiCameraService,
        mission: Optional[MissionService] = None,
        fps: int = DEFAULT_FPS,
    ):
        self.logger = get_logger(__name__)
        self.drone = drone
        self.camera = camera
        self.mission = mission or MissionService(drone)
        self.led_detector = LedDetector(self.camera.WIDTH, self.camera.HEIGHT)
        self.fps = fps

    @staticmethod
    def load_search_zone() -> List[Tuple[float, float]]:
        zone_path = cfg.dirs.zones_dir / cfg.zones.search_zone_path
        return MissionService.load_Poly(zone_path)

    def prepare(
        self,
        is_bottle: bool = True,
        geofence: Optional[List[Tuple[float, float]]] = None,
        start_camera: bool = False,
    ) -> None:
        """Inicjalizacja przed pętlą detekcji (bez blokowania — jedna klatka)."""
        if geofence is None:
            geofence = self.load_search_zone()

        self.mission.GEOFENCE = geofence
        self.mission.TRG_CANDIDATES = []

        if start_camera:
            self.camera.start()

        self.camera.set_10fps_mode()
        self.led_detector.reset()
        self.logger.info(
            f"Detection pipeline prepared @ {self.fps}Hz (is_bottle={is_bottle})"
        )

    def _scale_pixel(self, x: int, y: int) -> Tuple[int, int]:
        sx = self.mission.image_width / self.camera.WIDTH
        sy = self.mission.image_height / self.camera.HEIGHT
        return int(x * sx), int(y * sy)

    def process_one_frame(self, is_bottle: bool = True) -> int:
        """
        Jedna iteracja: klatka z GiCameraService + detekcja + process_target().
        Wywoływać tylko z głównego wątku (MAVLink).
        """
        frame, _ = self.camera.get_frame()
        if frame is None:
            return 0

        targets = self.led_detector.process_frame(frame)
        accepted = 0

        for target in targets:
            if target["frames_unseen"] != 0:
                continue

            pixel = self._scale_pixel(target["x"], target["y"])
            result = self.mission.process_target(pixel, is_bottle, self.mission.GEOFENCE)
            if result:
                accepted += 1
                self.logger.info(
                    f"Target registered at pixel {pixel}, "
                    f"candidates={len(self.mission.TRG_CANDIDATES)}"
                )

        return accepted

    @staticmethod
    def _sleep_until_next_frame(loop_start: float, interval: float) -> None:
        elapsed = time.monotonic() - loop_start
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)

    def run(
        self,
        should_stop,
        is_bottle: bool = True,
        geofence: Optional[List[Tuple[float, float]]] = None,
        max_frames: Optional[int] = None,
        start_camera: bool = False,
    ) -> List[Dict[str, Any]]:
        """
        Pętla detekcji w głównym wątku.

        Args:
            should_stop: callable() -> bool — np. lambda: curr_wp >= stop_wp
        """
        self.prepare(is_bottle=is_bottle, geofence=geofence, start_camera=start_camera)

        interval = 1.0 / self.fps
        frame_count = 0

        while not should_stop():
            loop_start = time.monotonic()

            accepted = self.process_one_frame(is_bottle)
            if accepted:
                frame_count += 1

            if max_frames is not None and frame_count >= max_frames:
                break

            self._sleep_until_next_frame(loop_start, interval)

        targets = list(self.mission.TRG_CANDIDATES)
        self.logger.info(f"Pipeline finished with {len(targets)} target candidate(s)")
        return targets
