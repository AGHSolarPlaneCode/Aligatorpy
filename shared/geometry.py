"""
Geometria rzutowania kamera <-> ziemia. Cała matematyka przestrzenna w jednym
testowalnym miejscu (po stronie Mózgu). MissionService deleguje tutaj.

Konwencja (DO POTWIERDZENIA TESTEM NAZIEMNYM):
  - kamera w nadirze (patrzy pionowo w dół),
  - "góra" obrazu pokrywa się z dziobem drona (yaw),
  - roll/pitch/yaw w radianach, jak z ATTITUDE MAVLink,
  - świat w NED: [north, east, down], down dodatni.

Forward (pixel_to_gps) i reverse (gps_to_pixel) używają tej samej rot_matrix i
spójnych przeliczników GPS<->NED, więc round-trip jest dokładny. Zgodność z
RZECZYWISTOŚCIĄ (czy obrót/znaki pasują do montażu) sprawdza dopiero test naziemny.
"""
from __future__ import annotations

import math
import numpy as np
import cv2

EARTH_R = 6378137.0   # promień Ziemi [m] (przybliżenie sferyczne)


def rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Macierz obrotu body->world. Konwencja zachowana z oryginalnego
    MissionService.project_target_cords (do walidacji testem naziemnym).
    """
    Rx = np.array([[ np.cos(roll), 0, -np.sin(roll)],
                   [ 0,            1,  0           ],
                   [ np.sin(roll), 0,  np.cos(roll)]])
    Ry = np.array([[1, 0,             0            ],
                   [0, np.cos(pitch), np.sin(pitch)],
                   [0, -np.sin(pitch), np.cos(pitch)]])
    Rz = np.array([[ np.cos(yaw), np.sin(yaw), 0],
                   [-np.sin(yaw), np.cos(yaw), 0],
                   [ 0,           0,           1]])
    return Rz @ Ry @ Rx


def ned_offset_to_gps(lat0: float, lon0: float, north: float, east: float) -> tuple:
    """Przesunięcie NED [m] -> nowe (lat, lon). Jawna kolejność north/east."""
    dlat = math.degrees(north / EARTH_R)
    dlon = math.degrees(east / (EARTH_R * math.cos(math.radians(lat0))))
    return lat0 + dlat, lon0 + dlon


def gps_to_ned_offset(lat_t: float, lon_t: float, lat0: float, lon0: float) -> tuple:
    """Odwrotność ned_offset_to_gps: (lat,lon) celu -> (north, east) [m] względem (lat0,lon0)."""
    north = math.radians(lat_t - lat0) * EARTH_R
    east = math.radians(lon_t - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return north, east


def pixel_to_gps(u, v, lat_uav, lon_uav, alt_uav, roll, pitch, yaw,
                 K, dist, img_w, img_h):
    """
    Rzut piksela (u,v) na ziemię -> (lat, lon). Undistort robiony tutaj (w Mózgu).
    Zwraca None gdy piksel poza obrazem albo promień nie trafia w ziemię.
    """
    if not (0 <= u < img_w and 0 <= v < img_h):
        return None

    undist = cv2.undistortPoints(
        np.array([[[float(u), float(v)]]], dtype=np.float64),
        cameraMatrix=K, distCoeffs=dist
    )
    x_u, y_u = undist[0, 0]

    ray_cam = np.array([x_u, -y_u, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)

    ray_world = rot_matrix(roll, pitch, yaw) @ ray_cam
    dz = ray_world[2]
    if abs(dz) < 1e-6 or dz <= 0:
        return None   # promień równoległy do ziemi lub nad horyzont

    t = alt_uav / dz
    north, east = ray_world[0] * t, ray_world[1] * t
    return ned_offset_to_gps(lat_uav, lon_uav, north, east)


def point_in_polygon(lat: float, lon: float, polygon) -> bool:
    """
    Ray casting. polygon: lista (lat, lon). Zwraca True jeśli punkt wewnątrz.
    """
    n = len(polygon)
    inside = False
    for i in range(n):
        j = (i + 1) % n
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if ((yi > lat) != (yj > lat)) and \
           (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
    return inside


def gps_to_pixel(lat_t, lon_t, lat_uav, lon_uav, alt_uav, roll, pitch, yaw,
                 K, dist, img_w, img_h):
    """
    Rzut odwrotny: gdzie dioda o znanym GPS wypada na SUROWEJ matrycy (z dystorsją).
    Używane do HINT w fazie DECODE. Zwraca (u, v) lub None gdy poza kadrem/za dronem.
    """
    north, east = gps_to_ned_offset(lat_t, lon_t, lat_uav, lon_uav)
    # wektor świata dron->cel w NED (cel na ziemi, dron na alt_uav)
    vec_world = np.array([north, east, alt_uav])

    # obrót world->body to transpozycja rot_matrix
    ray_cam = rot_matrix(roll, pitch, yaw).T @ vec_world
    if abs(ray_cam[2]) < 1e-6 or ray_cam[2] <= 0:
        return None

    # odwrotność ray_cam=[x_u,-y_u,1] z forward
    x_u = ray_cam[0] / ray_cam[2]
    y_u = -ray_cam[1] / ray_cam[2]

    # rzut znormalizowanego punktu przez K + dystorsję na piksel
    obj = np.array([[[x_u, y_u, 1.0]]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(
        obj, np.zeros(3), np.zeros(3), K, dist
    )
    u, v = img_pts[0, 0]
    if not (0 <= u < img_w and 0 <= v < img_h):
        return None
    return float(u), float(v)
