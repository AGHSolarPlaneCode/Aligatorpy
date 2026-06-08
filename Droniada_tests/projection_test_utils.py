"""Wspólne narzędzia do testów projekcji piksel → GPS (process_one_frame → process_target)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple
from unittest.mock import MagicMock
import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from Application.Services.DetectionPipelineService import DetectionPipelineService
from Application.Services.MissionService import MissionService
from Application.Services.gi_camera_handler import HEIGHT, WIDTH


@dataclass
class GridPoint:
    col: int
    row: int
    cam_x: int
    cam_y: int
    scaled_x: int
    scaled_y: int
    lat: Optional[float]
    lon: Optional[float]


def make_mock_drone(
    lat: float,
    lon: float,
    alt: float,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> MagicMock:
    drone = MagicMock()
    drone.get_current_coordinates.return_value = (lat, lon, alt)
    drone.get_attitude.return_value = (
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    return drone


def make_mock_camera(width: int = WIDTH, height: int = HEIGHT) -> MagicMock:
    camera = MagicMock()
    camera.WIDTH = width
    camera.HEIGHT = height
    return camera


def make_detection_pipeline(
    drone: MagicMock,
    camera: Optional[MagicMock] = None,
) -> DetectionPipelineService:
    camera = camera or make_mock_camera()
    mission = MissionService(drone)
    return DetectionPipelineService(drone=drone, camera=camera, mission=mission)


def iter_camera_grid(
    n_cols: int,
    n_rows: int,
    cam_w: int = WIDTH,
    cam_h: int = HEIGHT,
) -> Iterator[Tuple[int, int, int, int]]:
    """Środki komórek siatki w układzie kamery (1280×800)."""
    for row in range(n_rows):
        for col in range(n_cols):
            cam_x = int((col + 0.5) * cam_w / n_cols)
            cam_y = int((row + 0.5) * cam_h / n_rows)
            yield col, row, cam_x, cam_y


def scale_pixel(
    mission: MissionService,
    camera: MagicMock,
    cam_x: int,
    cam_y: int,
) -> Tuple[int, int]:
    sx = mission.image_width / camera.WIDTH
    sy = mission.image_height / camera.HEIGHT
    return int(cam_x * sx), int(cam_y * sy)


def huge_geofence(lat: float, lon: float, margin_deg: float = 0.05) -> List[Tuple[float, float]]:
    return [
        (lat - margin_deg, lon - margin_deg),
        (lat + margin_deg, lon - margin_deg),
        (lat + margin_deg, lon + margin_deg),
        (lat - margin_deg, lon + margin_deg),
    ]


def latlon_to_meters(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float]:
    avg_lat = math.radians(origin_lat)
    east = (lon - origin_lon) * 111_320 * math.cos(avg_lat)
    north = (lat - origin_lat) * 110_574
    return east, north


def project_via_process_one_frame(
    pipeline: DetectionPipelineService,
    cam_x: int,
    cam_y: int,
    geofence: List[Tuple[float, float]],
    is_bottle: bool = True,
) -> Tuple[Optional[float], Optional[float]]:
    """
    Jedna projekcja przez ten sam łańcuch co Droniada_mission._run_detection_phase:
    process_one_frame → _scale_pixel → process_target → project_target_cords.
    """
    frame = np.zeros((pipeline.camera.HEIGHT, pipeline.camera.WIDTH), dtype=np.uint8)
    pipeline.camera.get_image.return_value = (frame, 0.0, None)
    pipeline.mission.GEOFENCE = geofence

    captured: dict = {}

    def _capture_insert(lat: float, lon: float, _is_bottle: bool) -> None:
        captured["lat"] = lat
        captured["lon"] = lon

    target = {"id": 0, "x": cam_x, "y": cam_y, "frames_unseen": 0}

    from unittest.mock import patch

    with patch.object(pipeline.led_detector, "process_frame", return_value=[target]):
        with patch.object(pipeline.mission, "isinPolygon", return_value=True):
            with patch.object(pipeline.mission, "is_target", return_value=False):
                with patch.object(pipeline.mission, "insert_target", side_effect=_capture_insert):
                    pipeline.process_one_frame(is_bottle=is_bottle)

    return captured.get("lat"), captured.get("lon")


def project_center_via_process_one_frame(
    pipeline: DetectionPipelineService,
    geofence: List[Tuple[float, float]],
    is_bottle: bool = True,
) -> Tuple[Optional[float], Optional[float]]:
    """Projekcja środka kadru kamery."""
    return project_via_process_one_frame(
        pipeline,
        WIDTH // 2,
        HEIGHT // 2,
        geofence,
        is_bottle=is_bottle,
    )


def sweep_center_pixel_vs_attitude(
    lat: float,
    lon: float,
    alt: float,
    geofence: List[Tuple[float, float]],
    *,
    base_roll_deg: float = 0.0,
    base_pitch_deg: float = 0.0,
    base_yaw_deg: float = 0.0,
    axis: str,
    values_deg: List[float],
    is_bottle: bool = True,
) -> List[Tuple[float, Optional[float], Optional[float]]]:
    """
    Dla każdej wartości kąta (roll/pitch/yaw) zwraca (kąt, east_m, north_m)
    projekcji środka kadru względem pozycji drona.
    """
    results: List[Tuple[float, Optional[float], Optional[float]]] = []
    for angle_deg in values_deg:
        roll = base_roll_deg
        pitch = base_pitch_deg
        yaw = base_yaw_deg
        if axis == "roll":
            roll = angle_deg
        elif axis == "pitch":
            pitch = angle_deg
        elif axis == "yaw":
            yaw = angle_deg
        else:
            raise ValueError(f"Unknown axis: {axis}")

        drone = make_mock_drone(lat, lon, alt, roll, pitch, yaw)
        pipeline = make_detection_pipeline(drone)
        lat_p, lon_p = project_center_via_process_one_frame(
            pipeline, geofence, is_bottle=is_bottle
        )
        if lat_p is None or lon_p is None:
            results.append((angle_deg, None, None))
        else:
            east, north = latlon_to_meters(lat_p, lon_p, lat, lon)
            results.append((angle_deg, east, north))
    return results


def project_grid_via_process_one_frame(
    pipeline: DetectionPipelineService,
    n_cols: int,
    n_rows: int,
    geofence: List[Tuple[float, float]],
    is_bottle: bool = True,
) -> List[GridPoint]:
    points: List[GridPoint] = []
    for col, row, cam_x, cam_y in iter_camera_grid(n_cols, n_rows):
        scaled_x, scaled_y = scale_pixel(pipeline.mission, pipeline.camera, cam_x, cam_y)
        lat, lon = project_via_process_one_frame(
            pipeline, cam_x, cam_y, geofence, is_bottle=is_bottle
        )
        points.append(
            GridPoint(
                col=col,
                row=row,
                cam_x=cam_x,
                cam_y=cam_y,
                scaled_x=scaled_x,
                scaled_y=scaled_y,
                lat=lat,
                lon=lon,
            )
        )
    return points

if __name__ == "__main__":
    print("--- ROZPOCZYNAMY TESTY PROJEKCJI ---")
    
    # 1. Ustawiamy testowe współrzędne drona (np. okolice Rynku w Krakowie) i wysokość
    test_lat = 50.06143
    test_lon = 19.93658
    test_alt = 50.0  # dron wisi 50 metrów nad ziemią
    
    # 2. Tworzymy strefę geofence, żeby system nie odrzucał wyników jako "poza mapą"
    gf = huge_geofence(test_lat, test_lon)
    
    # =========================================================
    # TEST 1: Dron wisi idealnie poziomo, patrzymy w środek kamery
    # =========================================================
    print("\n[TEST 1] Środek kamery (Dron w poziomie: roll=0, pitch=0, yaw=0)")
    
    # Tworzymy naszego sztucznego drona i podpinamy go do potoku
    drone = make_mock_drone(test_lat, test_lon, test_alt)
    pipeline = make_detection_pipeline(drone)
    
    # Odpalamy projekcję dla środka kadru
    lat_p, lon_p = project_center_via_process_one_frame(pipeline, gf)
    
    print(f"Pozycja drona:  lat = {test_lat}, lon = {test_lon}")
    if lat_p is not None and lon_p is not None:
        print(f"Pozycja obiektu: lat = {lat_p:.5f}, lon = {lon_p:.5f}")
        # Przeliczamy na metry, żeby łatwiej było nam ocenić czy wynik ma sens
        east, north = latlon_to_meters(lat_p, lon_p, test_lat, test_lon)
        print(f"-> Obiekt znajduje się: {east:.2f}m na Wschód i {north:.2f}m na Północ od drona.")
    else:
        print("-> BŁĄD: System nie zwrócił żadnych współrzędnych!")

    # =========================================================
    # TEST 2: Testowanie pochylenia drona (Pitch)
    # =========================================================
    print("\n[TEST 2] Zmiana pochylenia drona (Pitch od -15 do +15 stopni)")
    
    # Wywołujemy gotowe narzędzie z Twojego kodu
    angles = [-15.0, -5.0, 0.0, 5.0, 15.0]
    results = sweep_center_pixel_vs_attitude(
        lat=test_lat, lon=test_lon, alt=test_alt, geofence=gf,
        axis="pitch", values_deg=angles
    )
    
    for angle, east, north in results:
        if east is not None and north is not None:
            print(f"Kąt pitch = {angle:5.1f}° -> Przesunięcie na ziemi: Wschód: {east:6.2f}m, Północ: {north:6.2f}m")
        else:
            print(f"Kąt pitch = {angle:5.1f}° -> Brak detekcji")
            
    print("\n--- KONIEC TESTÓW ---")
