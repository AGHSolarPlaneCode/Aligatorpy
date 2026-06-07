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
