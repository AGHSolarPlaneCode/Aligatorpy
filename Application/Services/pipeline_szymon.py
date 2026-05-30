from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from Application.Logger.log_module import get_logger
from Application.Services.CameraService import CameraService
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import cfg


class DetectionPipelineService:
    """
    Pełna misja detekcji diod LED z modulacją OOK.

    Fazy:
      1. SEARCH  – lot AUTO z WP_A do WP_B z obniżoną prędkością; detekcja diod
                   @10 Hz, rzutowanie pikseli na GPS i agregacja celów.
      2. VISIT   – po osiągnięciu WP_B dokładamy każdą diodę jako NAV_LOITER_TIME
                   w kolejności najbliższego sąsiada (greedy nearest-neighbour).
      3. OOK     – kamera przełącza się na wyższe fps / inną rozdzielczość;
                   nad każdą diodą (hover) zbieramy szereg czasowy jasności i
                   dekodujemy częstotliwość OOK (patrz hook `decode_ook`).

    Wykorzystuje istniejące metody:
      - CameraService.process_led_frame()        (detekcja blobów)
      - CameraService.configure_for_streaming()   (faza SEARCH)
      - CameraService.configure_for_ook()         (NOWE, faza OOK)
      - MissionService.process_target()           (zrzut + agregacja)
      - MissionService.get_distance_meters()      (poprawione, statyczne)
      - MatekService.set_telemetry_rate()
      - MatekService.set_speed()                  (NOWE)
      - MatekService.set_waypoints / start_mission / append_waypoints
    """

    DEFAULT_FPS = 10

    # ---- Faza OOK: ustawienia kamery (Phase 3) ----
    # fps musi spełniać Nyquista względem max częstotliwości OOK (fps >= 2*f_max).
    # Wartości DOMYŚLNE — podmień pod swój sprzęt / cfg.
    OOK_FPS = 120
    OOK_RESOLUTION = (1280, 800)

    REACH_RADIUS_M = 2.0

    REACH_CHECK_EVERY = 1

    def __init__(
        self,
        drone: MatekService,
        camera: Optional[CameraService] = None,
        mission: Optional[MissionService] = None,
        fps: int = DEFAULT_FPS,
        search_speed: float = 5.0,      # m/s podczas przeszukiwania  (-> wire to cfg)
        search_alt: float = 30.0,       # m AGL trasy SEARCH           (-> wire to cfg)
        hover_time: float = 8.0,        # s loiteru nad każdą diodą    (-> wire to cfg)
        expected_leds: int = 10,        # ile diod się spodziewamy
    ):
        self.logger = get_logger(__name__)
        self.drone = drone
        self.camera = camera or CameraService(drone=drone)
        self.mission = mission or MissionService(drone)
        self.fps = fps

        self.search_speed = search_speed
        self.search_alt = search_alt
        self.hover_time = hover_time
        self.expected_leds = expected_leds

        # wyniki fazy OOK: [{"lat","lon","isBottle","frequency_hz"}, ...]
        self.ook_results: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------ #
    #  Pomocnicze (geometria / setup)                                     #
    # ------------------------------------------------------------------ #
    @staticmethod
    def load_search_zone() -> List[Tuple[float, float]]:
        zone_path = cfg.dirs.zones_dir / cfg.zones.search_zone_path
        return MissionService.load_Poly(zone_path)

    def _scale_pixel(self, x: int, y: int) -> Tuple[int, int]:  
        """Skaluje piksel z rozdzielczości detekcji do rozdzielczości kalibracji."""
        sx = self.mission.image_width / self.camera.RESOLUTION_W
        sy = self.mission.image_height / self.camera.RESOLUTION_H
        return int(x * sx), int(y * sy)

    @staticmethod
    def _distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        return MissionService.get_distance_meters(lat1, lon1, lat2, lon2)

    def _reached(self, lat: float, lon: float, radius: float = None) -> bool: #osiagniety wp czy nie
        radius = self.REACH_RADIUS_M if radius is None else radius
        pos = self.drone.get_current_coordinates()
        if pos is None:
            return False
        cur_lat, cur_lon, _ = pos
        return self._distance(cur_lat, cur_lon, lat, lon) <= radius

    def _wait_until_reached(self, lat, lon, stop_event, timeout: float = 120.0) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if stop_event.is_set():
                return False
            if self._reached(lat, lon):
                return True
            time.sleep(0.5)
        self.logger.warning(f"Timeout dojścia do ({lat:.6f},{lon:.6f})")
        return False

    @staticmethod
    def _sleep_until_next_frame(loop_start: float, interval: float) -> None:
        remaining = interval - (time.monotonic() - loop_start)
        if remaining > 0:
            time.sleep(remaining)

    # ------------------------------------------------------------------ #
    #  Faza 1: detekcja                                                   #
    # ------------------------------------------------------------------ #
    def _process_frame(self, frame, is_bottle: bool) -> int:
        """Wykrywa diody w klatce i rejestruje je przez MissionService.process_target()."""
        _, targets = self.camera.process_led_frame(frame)
        accepted = 0
        for target in targets:
            if target["frames_unseen"] != 0:
                continue
            pixel = self._scale_pixel(target["x"], target["y"])
            # process_target po poprawce używa przekazanej strefy (nie self.GEOFENCE)
            if self.mission.process_target(pixel, is_bottle, self.mission.GEOFENCE):
                accepted += 1
        return accepted

    def _detection_loop(
        self,
        stop_event,
        is_bottle: bool,
        done_check=None,
        max_frames: Optional[int] = None,
    ) -> int:
        """Wspólna pętla detekcji. Kończy się przez stop_event / done_check / max_frames."""
        interval = 1.0 / self.fps
        frame_count = 0
        while not stop_event.is_set():
            loop_start = time.monotonic()

            frame = self.camera.capture_frame()
            accepted = self._process_frame(frame, is_bottle)
            frame_count += 1

            if accepted:
                self.logger.info(
                    f"Frame {frame_count}: +{accepted} det., "
                    f"candidates={len(self.mission.TRG_CANDIDATES)}"
                )

            # sprawdzamy warunek zakończenia co kilka klatek (oszczędza MAVLink), imo useless
            if done_check is not None and frame_count % self.REACH_CHECK_EVERY == 0:
                if done_check():
                    self.logger.info("Warunek zakończenia fazy SEARCH spełniony.")
                    break

            if max_frames is not None and frame_count >= max_frames:
                break

            self._sleep_until_next_frame(loop_start, interval)
        return frame_count

    # ------------------------------------------------------------------ #
    #  Faza 2: kolejność najbliższego sąsiada + loiter waypoints          #
    # ------------------------------------------------------------------ #
    def _nearest_neighbor_order(
        self, start_lat: float, start_lon: float, targets: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """zawsze najbliższa jeszcze nieodwiedzona dioda."""
        remaining = list(targets)
        ordered: List[Dict[str, Any]] = []
        cur_lat, cur_lon = start_lat, start_lon
        while remaining:
            nxt = min(
                remaining,
                key=lambda t: self._distance(cur_lat, cur_lon, t["lat"], t["lon"]),
            )
            ordered.append(nxt)
            remaining.remove(nxt)
            cur_lat, cur_lon = nxt["lat"], nxt["lon"]
        return ordered

    def _build_loiter_waypoints(
        self, ordered: List[Dict[str, Any]], alt: float
    ) -> List[Dict[str, Any]]:
        wps = []
        for t in ordered:
            wps.append(
                {
                    "command": "NAV_LOITER_TIME",
                    "lat": t["lat"],
                    "lon": t["lon"],
                    "alt": alt,
                    "time": self.hover_time,
                    "radius": 0,   # dla drona to po prostu zawisnij idealnie nad
                    "yaw": 0,
                }
            )
        return wps

    # ------------------------------------------------------------------ #
    #  Faza 3: OOK                                                        #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _roi_intensity(frame, box: int = 60) -> float: #roi dla diody nad ktora jestesmy
        """Średnia jasność centralnego ROI (dioda powinna być ~w centrum podczas hoveru)."""
        h, w = frame.shape[0], frame.shape[1]
        cy, cx = h // 2, w // 2
        roi = frame[max(0, cy - box):cy + box, max(0, cx - box):cx + box]
        if roi.ndim == 3:
            roi = roi.mean(axis=2) #dla bezpiecznejsta bo kamera i tak jest mono
        return float(roi.mean())

    def _capture_ook_window(self, duration: float) -> Tuple[List[float], List[float]]:
        """Zbiera szereg czasowy jasności ROI przez `duration` sekund."""
        series: List[float] = [] #jasnosci srednie
        times: List[float] = []  # momenty dla tych jasnosci 
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            frame = self.camera.capture_frame()
            times.append(time.monotonic())
            series.append(self._roi_intensity(frame))
        return series, times

    def decode_ook(self, series: List[float], times: List[float]) -> Optional[float]:
        """
        HOOK / referencyjna implementacja dekodowania OOK.

        Szereg jasności -> usunięcie składowej stałej -> FFT -> szczyt widma.
        Zwraca dominującą częstotliwość [Hz] albo None.

        UWAGA: to wersja referencyjna. Dostrój pod swoje diody:
          - upewnij się, że OOK_FPS >= 2 * f_max (Nyquist),
          - rozważ okno (Hanning), filtrację, próg SNR, dopasowanie do siatki
            znanych częstotliwości diod, jeśli takie masz.
        """
        n = len(series)
        if n < 8 or times[-1] <= times[0]:
            self.logger.warning("Za mało próbek do dekodowania OOK.")
            return None

        fs = (n - 1) / (times[-1] - times[0])   # zmierzone fps
        sig = np.asarray(series, dtype=np.float64)
        sig -= sig.mean()                        # usuń DC
        if np.allclose(sig, 0):
            return None

        window = np.hanning(n)
        spectrum = np.abs(np.fft.rfft(sig * window))
        freqs = np.fft.rfftfreq(n, d=1.0 / fs)

        spectrum[0] = 0.0                        # ignoruj DC
        peak_idx = int(np.argmax(spectrum))
        peak_freq = float(freqs[peak_idx])
        self.logger.info(f"OOK: fs~{fs:.1f}Hz, peak={peak_freq:.2f}Hz")
        return peak_freq

    def _visit_and_decode(self, ordered: List[Dict[str, Any]], stop_event) -> None:
        for i, t in enumerate(ordered, start=1):
            if stop_event.is_set():
                break
            self.logger.info(f"-> dioda {i}/{len(ordered)} ({t['lat']:.6f},{t['lon']:.6f})")
            if not self._wait_until_reached(t["lat"], t["lon"], stop_event):
                self.logger.warning("Nie dotarto do diody, pomijam dekodowanie.")
                continue

            # zostawiamy margines względem hover_time na samo zbieranie próbek
            series, times = self._capture_ook_window(max(1.0, self.hover_time * 0.9))
            freq = self.decode_ook(series, times)
            self.ook_results.append(
                {
                    "lat": t["lat"],
                    "lon": t["lon"],
                    "isBottle": t.get("isBottle"),
                    "frequency_hz": freq,
                }
            )
    
    # ------------------------------------------------------------------ #
    #  Główny orchestrator                                                #
    # ------------------------------------------------------------------ #
    def run_full_mission(
        self,
        wp_a: Dict[str, float],
        wp_b: Dict[str, float],
        stop_event=None,
        is_bottle: bool = True,
        geofence: Optional[List[Tuple[float, float]]] = None,
        alt: Optional[float] = None,
        takeoff_alt: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Args:
            wp_a, wp_b: {"lat","lon"} — punkty trasy przeszukiwania
            stop_event: threading/multiprocessing Event
            is_bottle:  przekazywane do process_target (diody traktowane jednolicie)
            geofence:   lista (lat,lon); domyślnie z cfg.zones
            alt:        wysokość trasy [m AGL]; domyślnie self.search_alt
            takeoff_alt: jeśli podane, dokładamy NAV_TAKEOFF na początku misji
        Returns:
            self.ook_results
        """
        stop_event = stop_event or self.camera.stop_event
        geofence = geofence or self.load_search_zone()
        alt = self.search_alt if alt is None else alt

        self.mission.GEOFENCE = geofence
        self.mission.TRG_CANDIDATES = []
        self.ook_results = []

        # --- 1. zbuduj i wgraj misję SEARCH (A -> B) ---
        nav: List[Dict[str, Any]] = []
        if takeoff_alt is not None:
            nav.append({"command": "TAKEOFF", "alt": takeoff_alt})
        nav.append({"command": "WAYPOINT", "lat": wp_a["lat"], "lon": wp_a["lon"], "alt": alt, "acr": 10})
        nav.append({"command": "WAYPOINT", "lat": wp_b["lat"], "lon": wp_b["lon"], "alt": alt, "acr": 10})

        if not self.drone.set_waypoints(nav):
            self.logger.error("Nie udało się wgrać misji SEARCH.")
            return []

        # --- 2. zwolnij na czas przeszukiwania, --- 3. start AUTO ---
        self.drone.set_speed(self.search_speed)
        if not self.drone.start_mission():
            self.logger.error("Nie udało się wystartować misji.")
            return []

        # --- 4. kamera + telemetria do detekcji @ fps ---
        self.drone.set_telemetry_rate(self.fps)
        self.camera.configure_for_streaming(
            size=(self.camera.RESOLUTION_W, self.camera.RESOLUTION_H), fps=self.fps
        )
        self.camera.reset_led_detector()

        # --- 5. SEARCH: detekcja aż do osiągnięcia WP_B ---
        self.logger.info(
            f"SEARCH @ {self.fps}Hz, speed={self.search_speed}m/s "
            f"(spodziewane diody={self.expected_leds})"
        )
        self._detection_loop(
            stop_event,
            is_bottle=is_bottle,
            done_check=lambda: self._reached(wp_b["lat"], wp_b["lon"]),
        )

        targets = list(self.mission.TRG_CANDIDATES)
        self.logger.info(f"SEARCH zakończony: znaleziono {len(targets)} diod.")
        if not targets:
            return []
        if len(targets) < self.expected_leds:
            self.logger.warning(
                f"Znaleziono {len(targets)}/{self.expected_leds} diod — kontynuuję mimo to."
            )

        # --- 6. kolejność najbliższego sąsiada (start = WP_B) ---
        ordered = self._nearest_neighbor_order(wp_b["lat"], wp_b["lon"], targets)

        # --- 7. dołóż loiter waypoints do misji ---
        loiter_wps = self._build_loiter_waypoints(ordered, alt)
        if not self.drone.append_waypoints(loiter_wps):
            self.logger.error("Nie udało się dołożyć waypointów loiter.")
            return []

        # --- 8. przełącz kamerę na tryb OOK ---
        self.camera.configure_for_ook(size=self.OOK_RESOLUTION, fps=self.OOK_FPS)

        # --- 9. VISIT + dekodowanie OOK nad każdą diodą ---
        self._visit_and_decode(ordered, stop_event)

        self.logger.info(f"Misja zakończona. Wyniki OOK: {self.ook_results}")
        return self.ook_results

    # ------------------------------------------------------------------ #
    #  Tryb tylko-detekcja (do testów, bez sterowania lotem)              #
    # ------------------------------------------------------------------ #
    def run(
        self,
        stop_event=None,
        is_bottle: bool = True,
        geofence: Optional[List[Tuple[float, float]]] = None,
        max_frames: Optional[int] = None,
        configure_camera: bool = True,
    ) -> List[Dict[str, Any]]:
        """Pętla tylko detekcji (bez nawigacji) — zachowana do testów."""
        if stop_event is None:
            stop_event = self.camera.stop_event
        if geofence is None:
            geofence = self.load_search_zone()

        self.mission.GEOFENCE = geofence
        self.mission.TRG_CANDIDATES = []

        self.drone.set_telemetry_rate(self.fps)
        if configure_camera:
            self.camera.configure_for_streaming(
                size=(self.camera.RESOLUTION_W, self.camera.RESOLUTION_H), fps=self.fps
            )
        self.camera.reset_led_detector()

        self.logger.info(f"Detection-only loop @ {self.fps}Hz (is_bottle={is_bottle})")
        self._detection_loop(stop_event, is_bottle=is_bottle, max_frames=max_frames)

        targets = list(self.mission.TRG_CANDIDATES)
        self.logger.info(f"Detection-only finished: {len(targets)} candidate(s).")
        return targets


# ---------------------------------------------------------------------- #
#  Entry pointy do uruchomienia w osobnym procesie / wątku                #
# ---------------------------------------------------------------------- #
def run_full_mission(
    stop_event,
    wp_a: Dict[str, float],
    wp_b: Dict[str, float],
    is_bottle: bool = True,
    fps: int = DetectionPipelineService.DEFAULT_FPS,
    device: Optional[str] = None,
    baud: Optional[int] = None,
    geofence: Optional[List[Tuple[float, float]]] = None,
    takeoff_alt: Optional[float] = None,
    result_queue=None,
) -> List[Dict[str, Any]]:
    """Pełna misja (SEARCH -> VISIT -> OOK) w osobnym procesie."""
    pipeline_device = device or cfg.mav.device2
    drone = MatekService(device=pipeline_device, baud=baud or cfg.mav.baud)
    pipeline = DetectionPipelineService(drone=drone, fps=fps)
    try:
        results = pipeline.run_full_mission(
            wp_a=wp_a,
            wp_b=wp_b,
            stop_event=stop_event,
            is_bottle=is_bottle,
            geofence=geofence,
            takeoff_alt=takeoff_alt,
        )
        if result_queue is not None:
            result_queue.put(results)
        return results
    finally:
        drone.close()


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
    """Tryb tylko-detekcja w osobnym procesie (zachowany dla kompatybilności)."""
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