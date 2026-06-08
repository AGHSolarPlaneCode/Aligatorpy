#!/usr/bin/env python3
"""
Wizualizacja projekcji piksel → ziemia (process_one_frame → process_target).

Dla siatki pikseli (domyślnie 10×10) rysuje:
  - lewy panel: siatka na obrazie kamery (1280×800)
  - prawy panel: rzut na mapę lokalną [m] względem pozycji drona

Przykłady:
    python Droniada_tests/live/live_pixel_projection_map.py
    python Droniada_tests/live/live_pixel_projection_map.py --roll 10 --pitch 5
    python Droniada_tests/live/live_pixel_projection_map.py --grid-cols 10 --grid-rows 10 --alt 50
    python Droniada_tests/live/live_pixel_projection_map.py --output media/projection_map.png --no-show
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from contextlib import contextmanager
from unittest.mock import patch

from Droniada_tests.projection_test_utils import (
    huge_geofence,
    latlon_to_meters,
    make_detection_pipeline,
    make_mock_drone,
    project_grid_via_process_one_frame,
)
from Application.Services.gi_camera_handler import HEIGHT, WIDTH


DEFAULT_LAT = 52.2297
DEFAULT_LON = 21.0122


@contextmanager
def suppress_projection_prints():
    with patch("builtins.print"):
        yield


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError as exc:
        raise SystemExit(
            "Brak matplotlib — zainstaluj: pip install matplotlib\n"
            f"({exc})"
        ) from exc
    return plt, Rectangle


def plot_projection_map(
    points,
    *,
    lat_uav: float,
    lon_uav: float,
    alt_uav: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    n_cols: int,
    n_rows: int,
    output: str | None,
    show: bool,
) -> None:
    plt, Rectangle = _require_matplotlib()

    fig, (ax_img, ax_map) = plt.subplots(1, 2, figsize=(14, 6))

    for p in points:
        if p.lat is None or p.lon is None:
            continue
        east, north = latlon_to_meters(p.lat, p.lon, lat_uav, lon_uav)
        ax_img.scatter(p.cam_x, p.cam_y, c="red", s=40, zorder=3)
        ax_img.text(p.cam_x + 8, p.cam_y + 8, f"{p.col},{p.row}", fontsize=7, color="yellow")
        ax_map.scatter(east, north, c="tab:blue", s=40, zorder=3)
        ax_map.text(east + 0.5, north + 0.5, f"{p.col},{p.row}", fontsize=7)

    ax_img.set_xlim(0, WIDTH)
    ax_img.set_ylim(HEIGHT, 0)
    ax_img.set_aspect("equal")
    ax_img.set_title(f"Kamera {WIDTH}×{HEIGHT} — siatka {n_cols}×{n_rows}")
    ax_img.set_xlabel("cam_x [px]")
    ax_img.set_ylabel("cam_y [px]")
    ax_img.add_patch(
        Rectangle((0, 0), WIDTH, HEIGHT, fill=False, edgecolor="white", linewidth=1, linestyle="--")
    )

    ax_map.scatter(0, 0, c="red", marker="^", s=120, label="dron (nadir)", zorder=5)
    ax_map.axhline(0, color="gray", linewidth=0.5)
    ax_map.axvline(0, color="gray", linewidth=0.5)
    ax_map.set_aspect("equal")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_xlabel("East [m]")
    ax_map.set_ylabel("North [m]")
    ax_map.set_title(
        f"Projekcja @ {alt_uav:.0f} m\n"
        f"roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° yaw={yaw_deg:.1f}°"
    )
    ax_map.legend(loc="upper right")

    fig.suptitle(
        f"GPS drona: {lat_uav:.6f}, {lon_uav:.6f}  |  "
        f"process_one_frame → process_target → project_target_cords",
        fontsize=10,
    )
    fig.tight_layout()

    if output:
        out_path = os.path.abspath(output)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        fig.savefig(out_path, dpi=150)
        print(f"Zapisano: {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mapa projekcji pikseli na ziemię (test process_one_frame / process_target)"
    )
    parser.add_argument("--lat", type=float, default=DEFAULT_LAT, help="Szerokość drona [deg]")
    parser.add_argument("--lon", type=float, default=DEFAULT_LON, help="Długość drona [deg]")
    parser.add_argument("--alt", type=float, default=50.0, help="Wysokość drona [m]")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll [deg]")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch [deg]")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw [deg]")
    parser.add_argument("--grid-cols", type=int, default=10, help="Liczba kolumn siatki")
    parser.add_argument("--grid-rows", type=int, default=10, help="Liczba wierszy siatki")
    parser.add_argument("--output", default=None, help="Ścieżka PNG (np. media/projection_map.png)")
    parser.add_argument("--no-show", action="store_true", help="Nie otwieraj okna (tylko zapis --output)")
    args = parser.parse_args()

    drone = make_mock_drone(
        args.lat, args.lon, args.alt,
        roll_deg=args.roll, pitch_deg=args.pitch, yaw_deg=args.yaw,
    )
    pipeline = make_detection_pipeline(drone)
    geofence = huge_geofence(args.lat, args.lon)

    print(
        f"Projekcja siatki {args.grid_cols}×{args.grid_rows} "
        f"@ ({args.lat:.6f}, {args.lon:.6f}), alt={args.alt}m, "
        f"roll={args.roll}° pitch={args.pitch}° yaw={args.yaw}°"
    )

    with suppress_projection_prints():
        points = project_grid_via_process_one_frame(
            pipeline,
            args.grid_cols,
            args.grid_rows,
            geofence,
            is_bottle=True,
        )

    ok = sum(1 for p in points if p.lat is not None)
    print(f"Zprojektowano {ok}/{len(points)} punktów")

    for p in points:
        if p.lat is None:
            print(f"  [{p.col},{p.row}] cam=({p.cam_x},{p.cam_y}) -> BRAK")
            continue
        east, north = latlon_to_meters(p.lat, p.lon, args.lat, args.lon)
        print(
            f"  [{p.col},{p.row}] cam=({p.cam_x:4d},{p.cam_y:4d}) "
            f"scaled=({p.scaled_x:4d},{p.scaled_y:4d}) "
            f"-> E={east:+7.2f}m N={north:+7.2f}m"
        )

    plot_projection_map(
        points,
        lat_uav=args.lat,
        lon_uav=args.lon,
        alt_uav=args.alt,
        roll_deg=args.roll,
        pitch_deg=args.pitch,
        yaw_deg=args.yaw,
        n_cols=args.grid_cols,
        n_rows=args.grid_rows,
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
