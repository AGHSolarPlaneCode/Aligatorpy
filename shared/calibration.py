"""
Kalibracja kamery OV9281 — jedno źródło prawdy dla całego systemu.

Wartości wyznaczone dla rozdzielczości CALIB_W x CALIB_H. Jeśli faza pracuje
w innej rozdzielczości (np. crop), trzeba przeskalować K (dist bez zmian) —
patrz scale_K().
"""
from __future__ import annotations

import numpy as np

CALIB_W = 1280
CALIB_H = 800

K = np.array([
    [692.45913591,   0.0,         635.07827968],
    [  0.0,        692.52565945, 395.26189078],
    [  0.0,          0.0,           1.0       ],
], dtype=np.float64)

DIST = np.array(
    [[-3.42194310e-01,  1.23526508e-01,  4.70056821e-03,
      -1.09821856e-04, -2.11505931e-02]],
    dtype=np.float64,
)

IMG_CENTER = (CALIB_W / 2.0, CALIB_H / 2.0)


def scale_K(new_w: int, new_h: int) -> np.ndarray:
    """Zwraca K przeskalowane do (new_w, new_h). dist nie wymaga skalowania."""
    sx, sy = new_w / CALIB_W, new_h / CALIB_H
    Ks = K.copy()
    Ks[0, 0] *= sx   # fx
    Ks[1, 1] *= sy   # fy
    Ks[0, 2] *= sx   # cx
    Ks[1, 2] *= sy   # cy
    return Ks
