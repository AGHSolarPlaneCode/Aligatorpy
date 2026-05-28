from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from Application.Logger.log_module import get_logger
from Application.Services.CameraService import CameraService
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import cfg


class DetectionPipelineService:
    """
    Pipeline detekcji diod LED z telemetrią drona @ 10 Hz.

    Używa istniejących metod:
    - CameraService.process_led_frame()
    - MissionService.process_target()  (zrzutowanie + agregacja celów)
    - MatekService.set_telemetry_rate()  (nowa metoda, GPS + ATTITUDE)
    """

    DEFAULT_FPS = 10

    def __init__(
        self,
        drone: MatekService,
        camera: Optional[CameraService] = None,
        mission: Optional[MissionService] = None,
        fps: int = DEFAULT_FPS,
    ):
        self.logger = get_logger(__name__)
        self.drone = drone
        self.camera = camera or CameraService(drone=drone)
        self.mission = mission or MissionService(drone)
        self.fps = fps

    @staticmethod
    def load_search_zone() -> List[Tuple[float, float]]:
        zone_path = cfg.dirs.zones_dir / cfg.zones.search_zone_path
        return MissionService.load_Poly(zone_path)

    def _scale_pixel(self, x: int, y: int) -> Tuple[int, int]:
        """Skaluje piksel z rozdzielczości detekcji do rozdzielczości kalibracji kamery."""
        sx = self.mission.image_width / self.camera.RESOLUTION_W
        sy = self.mission.image_height / self.camera.RESOLUTION_H
        return int(x * sx), int(y * sy)

    def _process_frame(self, frame, is_bottle: bool) -> int:
        """
        Wykrywa diody w klatce i rejestruje je przez MissionService.process_target().
        Zwraca liczbę pomyślnie przetworzonych detekcji w tej klatce.
        """
        _, targets = self.camera.process_led_frame(frame)
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
        configure_camera: bool = True,
    ) -> List[Dict[str, Any]]:
        """
        Główna pętla pipeline detekcji.

        Args:
            stop_event: threading.Event / multiprocessing.Event — zatrzymuje pętlę
            is_bottle: typ celu przekazywany do process_target()
            geofence: lista (lat, lon) — domyślnie z cfg.zones
            max_frames: opcjonalny limit klatek (testy)
            configure_camera: przełącza kamerę w tryb wideo 10 fps

        Returns:
            Lista wykrytych celów: [{"lat", "lon", "count", "isBottle"}, ...]
        """
        if stop_event is None:
            stop_event = self.camera.stop_event

        if geofence is None:
            geofence = self.load_search_zone()

        self.mission.GEOFENCE = geofence
        self.mission.TRG_CANDIDATES = []

        self.drone.set_telemetry_rate(self.fps)
        if configure_camera:
            detection_size = (self.camera.RESOLUTION_W, self.camera.RESOLUTION_H)
            self.camera.configure_for_streaming(size=detection_size, fps=self.fps)

        self.camera.reset_led_detector()

        interval = 1.0 / self.fps
        frame_count = 0
        self.logger.info(
            f"Starting LED detection pipeline @ {self.fps}Hz (is_bottle={is_bottle})"
        )

        while not stop_event.is_set():
            loop_start = time.monotonic()

            frame = self.camera.capture_frame()
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
) -> List[Dict[str, Any]]:
    """
    Entry point do uruchomienia pipeline w osobnym procesie/wątku.

    Przykład (multiprocessing):
        from multiprocessing import Process, Event, Queue
        stop = Event()
        results = Queue()
        p = Process(
            target=run_led_detection_pipeline,
            args=(stop,),
            kwargs={"result_queue": results},
        )
        p.start()
        ...
        stop.set()
        p.join()
        targets = results.get()
    """
    pipeline_device = device or cfg.mav.device2
    drone = MatekService(device=pipeline_device, baud=baud or cfg.mav.baud)
    pipeline = DetectionPipelineService(drone=drone, fps=fps)
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
