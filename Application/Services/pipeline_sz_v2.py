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
    Pełna misja detekcji diod IR (850 nm) z modulacją OOK.
 
    Scenariusz operacyjny (wg ustaleń):
      - lot nocą (~21:00), wysokość ~50 m AGL,
      - kamera mono OV9281 (global shutter) + filtr bandpass 830-870 nm,
      - diody migają OOK, wypełnienie 50%, częstotliwość = JEDNA ze znanych:
        {2,4,6,8,10,12,14,16,18,20} Hz, i to ona jest identyfikatorem diody,
      - diody rozmieszczone co ~10 m.
 
    Fazy:
      1. SEARCH – lot AUTO WP_A -> WP_B ze zmniejszoną prędkością; detekcja blobów
                  @ ~10 Hz, rzutowanie na GPS, agregacja celów (MissionService).
      2. VISIT  – po dotarciu do WP_B dokładamy każdą diodę jako NAV_LOITER_TIME
                  w kolejności najbliższego sąsiada (greedy).
      3. OOK    – kamera przechodzi w tryb wysokich fps; nad każdą diodą (zawis)
                  zbieramy szereg czasowy jasności z WĄSKIEGO centralnego ROI i
                  KLASYFIKUJEMY częstotliwość do jednej ze znanych wartości
                  (matched periodogram / bank korelatorów).
 
    Uwaga geometryczna (dlaczego wąskie ROI):
      Przy 50 m i FOV 110°H/90°V ślad kamery na ziemi to ~143 x 100 m, więc przy
      rozstawie 10 m w kadrze jest kilkanaście diod naraz. Pełnoklatkowa jasność
      byłaby sumą wielu częstotliwości. Dlatego w fazie OOK używamy tylko małego
      kwadratu wokół środka obrazu (dioda w nadirze), zob. _ook_signal().
    """
 
    DEFAULT_FPS = 10
 
    # ------- znane częstotliwości OOK (identyfikatory diod) -------
    CANDIDATE_FREQS: Tuple[int, ...] = (2, 4, 6, 8, 10, 12, 14, 16, 18, 20)  # Hz
 
    # ------- Faza OOK: ustawienia kamery / klasyfikacji -------
    # fps >> 2*f_max (Nyquist dla 20 Hz to 40 Hz; bierzemy duży zapas na czyste zbocza)
    OOK_FPS = 100
    OOK_RESOLUTION = (640, 480)
    OOK_EXPOSURE_US = 2000          # krótka ekspozycja: czysty stan on/off (<< pół okresu)
    # Półszerokość centralnego ROI [px] dla OOK_RESOLUTION przy ~50 m.
    #   sąsiad z 10 m  -> atan(10/50) = 11.3° od nadiru
    #   skala (pion)   ~ H/VFOV = 480/90 = 5.33 px/°  -> sąsiad ~60 px od środka
    #   ROI ±35 px  ~ ±6.6° ~ ±5.8 m na ziemi: łapie nadir, toleruje ~3 m dryfu
    #                 zawisu (±18 px) i odcina sąsiada z 10 m (~60 px).
    OOK_ROI_HALF_PX = 35
    MIN_CONFIDENCE = 3.0            # P_best / P_2nd poniżej tego = wynik niepewny
 
    # promień uznania waypointa za osiągnięty [m]
    REACH_RADIUS_M = 8.0
    # co ile klatek sprawdzamy pozycję w fazie SEARCH (oszczędza pasmo MAVLink)
    REACH_CHECK_EVERY = 1 #co ktora klate lokalizujemy, 1 bo raczej na kazdej z 10
 
    def __init__(
        self,
        drone: MatekService,
        camera: Optional[CameraService] = None,
        mission: Optional[MissionService] = None,
        fps: int = DEFAULT_FPS,
        search_speed: float = 5.0,      # m/s podczas przeszukiwania       (-> cfg)
        search_alt: float = 50.0,       # m AGL trasy SEARCH/VISIT          (-> cfg)
        hover_time: float = 8.0,        # s loiteru nad diodą (do ~20 s OK) (-> cfg)
        expected_leds: int = 10,
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
 
        # wyniki OOK: [{"lat","lon","isBottle","frequency_hz","confidence"}, ...]
        self.ook_results: List[Dict[str, Any]] = []
 
    # ------------------------------------------------------------------ #
    #  Pomocnicze (setup / geometria)                                     #
    # ------------------------------------------------------------------ #
    @staticmethod
    def load_search_zone() -> List[Tuple[float, float]]:
        zone_path = cfg.dirs.zones_dir / cfg.zones.search_zone_path
        return MissionService.load_Poly(zone_path)
 
    @classmethod
    def recommended_roi_half_px(
        cls, alt_m: float, ground_radius_m: float, vfov_deg: float, img_h: int
    ) -> int:
        """
        Pomocniczo: półszerokość ROI [px], by objąć promień `ground_radius_m`
        na ziemi z wysokości `alt_m` (przybliżenie liniowe blisko środka kadru).
        Ustaw ground_radius wyraźnie poniżej połowy rozstawu diod (np. 5 m przy 10 m).
        """
        import math
        angle_deg = math.degrees(math.atan2(ground_radius_m, alt_m))
        px_per_deg = img_h / vfov_deg
        return max(8, int(round(angle_deg * px_per_deg)))
 
    def _scale_pixel(self, x: int, y: int) -> Tuple[int, int]:
        """Skaluje piksel z rozdzielczości detekcji do rozdzielczości kalibracji."""
        sx = self.mission.image_width / self.camera.RESOLUTION_W
        sy = self.mission.image_height / self.camera.RESOLUTION_H
        return int(x * sx), int(y * sy)
 
    @staticmethod
    def _distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        return MissionService.get_distance_meters(lat1, lon1, lat2, lon2)
 
    def _reached(self, lat: float, lon: float, radius: float = None) -> bool: #sprawdza czy juz jestesmy sprawdzamy cyklicznie zeby nie zgubic info
        radius = self.REACH_RADIUS_M if radius is None else radius
        pos = self.drone.get_current_coordinates()
        if pos is None:
            return False
        cur_lat, cur_lon, _ = pos
        return self._distance(cur_lat, cur_lon, lat, lon) <= radius
 
    def _wait_until_reached(self, lat, lon, stop_event, timeout: float = 120.0) -> bool: # czekamy az dolecimy do diody max 120s(do ustalenia)
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
    def _sleep_until_next_frame(loop_start: float, interval: float) -> None: #dokaldny czas do kolejnej klatki 
        remaining = interval - (time.monotonic() - loop_start)
        if remaining > 0:
            time.sleep(remaining)
 

    #  Faza 1: detekcja  

    def _process_frame(self, frame, is_bottle: bool) -> int:
        """Wykrywa diody w klatce i rejestruje(sprawdzac czy jest w prostokacie) je przez MissionService.process_target()."""
        _, targets = self.camera.process_led_frame(frame)
        accepted = 0
        for target in targets:
            if target["frames_unseen"] != 0:
                continue
            pixel = self._scale_pixel(target["x"], target["y"])
            if self.mission.process_target(pixel, is_bottle, self.mission.GEOFENCE):
                accepted += 1
        return accepted
 
    def _detection_loop(  # zarzadzanie lokalizowaniem diod, warunki zakonczenia lokalizowania
        self,
        stop_event,
        is_bottle: bool,
        done_check=None,
        max_frames: Optional[int] = None,
    ) -> int:
        """Wspólna pętla detekcji. Kończy ją stop_event / done_check / max_frames."""
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
    def _nearest_neighbor_order( # zwraca poukladane lokalizacje diod
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
 
    def _build_loiter_waypoints( #robi waypointy loitery dla dron z ordered
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
                    "radius": 0,    # 0 = loiter w miejscu (min. promień platformy)
                    "yaw": 0,
                }
            )
        return wps
 
    # ------------------------------------------------------------------ #
    #  Faza 3: OOK – akwizycja + klasyfikacja częstotliwości              #
    # ------------------------------------------------------------------ #
    def _ook_signal(self, frame, thr=None) -> int:#ile pixeli w roi jest ponad progiem 
        """
        Proxy jasności: liczba jasnych pikseli w WĄSKIM centralnym ROI.
        ROI ma izolować diodę w nadirze i odcinać sąsiadów .
        """
        thr = self.camera.THRESHOLD_VALUE if thr is None else thr
        img = frame.max(axis=2) if frame.ndim == 3 else frame   # OV9281 i tak mono
        h, w = img.shape
        cy, cx = h // 2, w // 2
        r = self.OOK_ROI_HALF_PX
        roi = img[max(0, cy - r):cy + r, max(0, cx - r):cx + r]
        return int((roi > thr).sum())
 
    def _capture_ook_window(self, duration: float) -> Tuple[List[int], List[float]]: #daje listy ile pikseli ponad progiem i kiedy w klatkach(dane do analziy)
        """Zbiera szereg czasowy jasności ROI przez `duration` sekund."""
        counts: List[int] = []
        times: List[float] = []
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            frame = self.camera.capture_frame()
            times.append(time.monotonic())
            counts.append(self._ook_signal(frame))
        return counts, times
 
    def classify_ook(
        self,
        counts: List[int],
        times: List[float],
        candidates: Tuple[int, ...] = None,
        min_cycles: int = 3,
    ) -> Optional[Dict[str, Any]]:
        """
        Klasyfikuje sygnał do JEDNEJ ze znanych częstotliwości.
        robi matematyczne fikumiku i porownuje fazy naszego sygnnalu ze wszystkimi z listy
        i zwraca sile danej czestotliwosci w sygnale sortujemy odzczytujemy zwyciezce, potem porownujemy z 2 miejscem
        jesli roznica jest duza to pewnosc jest duza:), jesli nie to nie:(
        """
        candidates = candidates or self.CANDIDATE_FREQS #bazowe czestotliwsci
        n = len(counts)
        if n < 16:
            self.logger.warning("Za mało próbek")
            return None
 
        x = np.asarray(counts, dtype=float)
        t = np.asarray(times, dtype=float) - times[0] 
        duration = float(t[-1])
        if duration <= 0: #blad zegara
            return None
        fs = (n - 1) / duration #faktyczne fps
 
        # tylko czestotliwosci poniżej Nyquista zmierzonego fps mamy minimum 120fps wiec dla basic bedzie git
        valid = [f for f in candidates if f < fs / 2.0]
        if not valid:
            self.logger.warning(f"faktyczne fps={fs:.1f}Hz za niskie (Nyquist) ")
            return None
        if duration * min(valid) < min_cycles: #sprawdzamy czy jest wystarczajaco cykli(3) , bedziemy miec duzo wiec chill
            self.logger.warning(
                f"Okno {duration:.2f}s = {duration*min(valid):.1f} cykli @ "
                f"{min(valid)}Hz (<{min_cycles}) – wynik mniej pewny."
            )
 
        x -= x.mean()                         # usuń składową stałą, zeby oscylowano wokol zera
        if np.allclose(x, 0):
            self.logger.warning("Brak modulacji w ROI (sygnał stały).")
            return None
        xw = x * np.hanning(n)                # okno tłumi przeciek widmowy, najwieksza wage maja pomiary ze srodka 
 
        powers = []
        for f in valid:
            ph = 2.0 * np.pi * f * t
            re = float(np.dot(xw, np.cos(ph)))
            im = float(np.dot(xw, np.sin(ph)))
            powers.append(re * re + im * im)
        powers = np.asarray(powers)
 
        order = np.argsort(powers)[::-1]
        best_f = valid[int(order[0])]
        p_best = float(powers[int(order[0])])
        p_second = float(powers[int(order[1])]) if len(valid) > 1 else 0.0
        confidence = (p_best / p_second) if p_second > 0 else float("inf")
 
        return {
            "frequency_hz": best_f,
            "confidence": confidence,
            "fs": fs,
            "duration_s": duration,
            "powers": {f: float(p) for f, p in zip(valid, powers)},
        }
 
    def decode_ook(self, counts, times):     # zachowuje stary interfejs/nazwę
        return self.classify_ook(counts, times)
 
    def _visit_and_decode(self, ordered: List[Dict[str, Any]], stop_event) -> None:
        capture_dur = max(2.0, self.hover_time - 1.5)   # bufor na dolot/ustabilizowanie
        for i, t in enumerate(ordered, start=1):
            if stop_event.is_set():
                break
            self.logger.info(
                f"-> dioda {i}/{len(ordered)} ({t['lat']:.6f},{t['lon']:.6f})"
            )
            if not self._wait_until_reached(t["lat"], t["lon"], stop_event):
                self.logger.warning("Nie dotarto do diody – pomijam.")
                self.ook_results.append(
                    {"lat": t["lat"], "lon": t["lon"], "isBottle": t.get("isBottle"),
                     "frequency_hz": None, "confidence": 0.0}
                )
                continue
 
            counts, times = self._capture_ook_window(capture_dur)
            result = self.decode_ook(counts, times)
 
            if result is None:
                freq, conf = None, 0.0
            else:
                freq, conf = result["frequency_hz"], result["confidence"]
 
            if freq is None or conf < self.MIN_CONFIDENCE:
                self.logger.warning(
                    f"Niepewna klasyfikacja diody {i}: f={freq}Hz, conf={conf:.1f} "
                    f"(< {self.MIN_CONFIDENCE}). Rozważ dłuższy zawis / mniejsze ROI."
                )
            else:
                self.logger.info(f"Dioda {i}: f={freq}Hz (conf={conf:.1f}).")
 
            self.ook_results.append(
                {"lat": t["lat"], "lon": t["lon"], "isBottle": t.get("isBottle"),
                 "frequency_hz": freq, "confidence": conf}
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
            wp_a, wp_b: {"lat","lon"} – trasa przeszukiwania
            stop_event: threading/multiprocessing Event
            is_bottle:  przekazywane do process_target (diody traktowane jednolicie)
            geofence:   lista (lat,lon); domyślnie z cfg.zones
            alt:        wysokość trasy [m AGL]; domyślnie self.search_alt (50 m)
            takeoff_alt: jeśli podane, dokładamy NAV_TAKEOFF na początku
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
 
        # --- 2. zwolnij na czas przeszukiwania, 3. start AUTO ---
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
            f"SEARCH @ {self.fps}Hz, speed={self.search_speed}m/s, alt={alt}m "
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
                f"Znaleziono {len(targets)}/{self.expected_leds} diod – kontynuuję."
            )
 
        # --- 6. kolejność najbliższego sąsiada (start = WP_B) ---
        ordered = self._nearest_neighbor_order(wp_b["lat"], wp_b["lon"], targets)
 
        # --- 7. dołóż loiter waypoints do misji ---
        loiter_wps = self._build_loiter_waypoints(ordered, alt)
        if not self.drone.append_waypoints(loiter_wps):
            self.logger.error("Nie udało się dołożyć waypointów loiter.")
            return []
 
        # --- 8. przełącz kamerę w tryb OOK (wyższe fps, krótka ekspozycja) ---
        self.camera.configure_for_ook(
            size=self.OOK_RESOLUTION, fps=self.OOK_FPS, exposure_us=self.OOK_EXPOSURE_US
        )
 
        # --- 9. VISIT + klasyfikacja OOK nad każdą diodą ---
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
        """Pętla tylko detekcji (bez nawigacji) – zachowana do testów."""
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
            wp_a=wp_a, wp_b=wp_b, stop_event=stop_event,
            is_bottle=is_bottle, geofence=geofence, takeoff_alt=takeoff_alt,
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
    """Tryb tylko-detekcja w osobnym procesie (kompatybilność)."""
    pipeline_device = device or cfg.mav.device2
    drone = MatekService(device=pipeline_device, baud=baud or cfg.mav.baud)
    pipeline = DetectionPipelineService(drone=drone, fps=fps)
    try:
        targets = pipeline.run(
            stop_event=stop_event, is_bottle=is_bottle,
            geofence=geofence, max_frames=max_frames,
        )
        if result_queue is not None:
            result_queue.put(targets)
        return targets
    finally:
        drone.close()