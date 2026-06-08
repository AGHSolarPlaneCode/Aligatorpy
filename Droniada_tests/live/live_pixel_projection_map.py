#!/usr/bin/env python3
"""
Wizualizacja projekcji piksel → ziemia (process_one_frame → process_target).

Wykres 1: siatka pikseli @ bazowy roll/pitch/yaw
Wykres 2: ta sama siatka @ zmodyfikowany roll/pitch/yaw
Wykres 3: położenie środka kadru (E/N) vs roll, pitch, yaw

    python Droniada_tests/live/live_pixel_projection_map.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from contextlib import contextmanager
from unittest.mock import patch

import numpy as np

from Droniada_tests.projection_test_utils import (
    GridPoint,
    huge_geofence,
    latlon_to_meters,
    make_detection_pipeline,
    make_mock_drone,
    project_grid_via_process_one_frame,
    sweep_center_pixel_vs_attitude,
)
from Application.Services.gi_camera_handler import HEIGHT, WIDTH

# ── Parametry symulacji (edytuj tutaj) ───────────────────────────────────────
LAT = 52.2297
LON = 21.0122
ALT = 50.0

ROLL = 0.0
PITCH = 0.0
YAW = 0.0

ROLL_MOD = 20
PITCH_MOD = 0
YAW_MOD = 0

GRID_COLS = 10
GRID_ROWS = 10

SWEEP_STEPS = 31
SWEEP_ROLL = 80.0
SWEEP_PITCH = 80.0
SWEEP_YAW = 45.0

OUTPUT = None          # np. "media/projection_map.png"
SHOW = True


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


def _draw_camera_grid(
    ax,
    points: list[GridPoint],
    *,
    n_cols: int,
    n_rows: int,
    title: str,
    Rectangle,
) -> None:
    for p in points:
        if p.lat is None or p.lon is None:
            continue
        ax.scatter(p.cam_x, p.cam_y, c="red", s=40, zorder=3)
        ax.text(p.cam_x + 8, p.cam_y + 8, f"{p.col},{p.row}", fontsize=7, color="yellow")

    cx, cy = WIDTH // 2, HEIGHT // 2
    ax.scatter(cx, cy, c="lime", s=80, marker="+", linewidths=2, zorder=4, label="środek")

    ax.set_xlim(0, WIDTH)
    ax.set_ylim(HEIGHT, 0)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("cam_x [px]")
    ax.set_ylabel("cam_y [px]")
    ax.add_patch(
        Rectangle((0, 0), WIDTH, HEIGHT, fill=False, edgecolor="white", linewidth=1, linestyle="--")
    )
    ax.legend(loc="upper right", fontsize=8)


def _draw_ground_map(
    ax,
    points: list[GridPoint],
    *,
    lat_uav: float,
    lon_uav: float,
    alt_uav: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    title: str,
) -> None:
    for p in points:
        if p.lat is None or p.lon is None:
            continue
        east, north = latlon_to_meters(p.lat, p.lon, lat_uav, lon_uav)
        ax.scatter(east, north, c="tab:blue", s=40, zorder=3)
        ax.text(east + 0.5, north + 0.5, f"{p.col},{p.row}", fontsize=7)

    ax.scatter(0, 0, c="red", marker="^", s=120, label="dron (nadir)", zorder=5)
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axvline(0, color="gray", linewidth=0.5)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(
        f"{title}\n@ {alt_uav:.0f} m | "
        f"roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° yaw={yaw_deg:.1f}°"
    )
    ax.legend(loc="upper right", fontsize=8)


def _draw_attitude_sweep(ax, sweep_data, *, axis_label: str, title: str) -> None:
    angles = [row[0] for row in sweep_data]
    east = [row[1] for row in sweep_data]
    north = [row[2] for row in sweep_data]

    valid_e = [(a, e) for a, e in zip(angles, east) if e is not None]
    valid_n = [(a, n) for a, n in zip(angles, north) if n is not None]

    if valid_e:
        ax.plot([v[0] for v in valid_e], [v[1] for v in valid_e], "o-", label="East [m]", color="tab:blue")
    if valid_n:
        ax.plot([v[0] for v in valid_n], [v[1] for v in valid_n], "s-", label="North [m]", color="tab:orange")

    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axvline(0, color="gray", linewidth=0.5, alpha=0.3)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel(f"{axis_label} [deg]")
    ax.set_ylabel("offset środka kadru [m]")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)


def plot_projection_analysis(
    points_base: list[GridPoint],
    points_mod: list[GridPoint],
    sweep_roll,
    sweep_pitch,
    sweep_yaw,
    *,
    lat_uav: float,
    lon_uav: float,
    alt_uav: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    roll2_deg: float,
    pitch2_deg: float,
    yaw2_deg: float,
    n_cols: int,
    n_rows: int,
    output: str | None,
    show: bool,
) -> None:
    plt, Rectangle = _require_matplotlib()

    fig = plt.figure(figsize=(16, 14))
    gs = fig.add_gridspec(3, 6, height_ratios=[1.2, 1.2, 1.0], hspace=0.35, wspace=0.3)

    ax_img_base = fig.add_subplot(gs[0, 0:3])
    ax_map_base = fig.add_subplot(gs[0, 3:6])
    ax_img_mod = fig.add_subplot(gs[1, 0:3])
    ax_map_mod = fig.add_subplot(gs[1, 3:6])
    ax_roll = fig.add_subplot(gs[2, 0:2])
    ax_pitch = fig.add_subplot(gs[2, 2:4])
    ax_yaw = fig.add_subplot(gs[2, 4:6])

    _draw_camera_grid(
        ax_img_base,
        points_base,
        n_cols=n_cols,
        n_rows=n_rows,
        title=f"Kamera {WIDTH}×{HEIGHT} — bazowy",
        Rectangle=Rectangle,
    )
    _draw_ground_map(
        ax_map_base,
        points_base,
        lat_uav=lat_uav,
        lon_uav=lon_uav,
        alt_uav=alt_uav,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
        title="Projekcja bazowa",
    )

    _draw_camera_grid(
        ax_img_mod,
        points_mod,
        n_cols=n_cols,
        n_rows=n_rows,
        title=f"Kamera {WIDTH}×{HEIGHT} — zmodyfikowany",
        Rectangle=Rectangle,
    )
    _draw_ground_map(
        ax_map_mod,
        points_mod,
        lat_uav=lat_uav,
        lon_uav=lon_uav,
        alt_uav=alt_uav,
        roll_deg=roll2_deg,
        pitch_deg=pitch2_deg,
        yaw_deg=yaw2_deg,
        title="Projekcja zmodyfikowana",
    )

    _draw_attitude_sweep(
        ax_roll,
        sweep_roll,
        axis_label="roll",
        title=f"Środek kadru vs roll (pitch={pitch_deg:.0f}°, yaw={yaw_deg:.0f}°)",
    )
    _draw_attitude_sweep(
        ax_pitch,
        sweep_pitch,
        axis_label="pitch",
        title=f"Środek kadru vs pitch (roll={roll_deg:.0f}°, yaw={yaw_deg:.0f}°)",
    )
    _draw_attitude_sweep(
        ax_yaw,
        sweep_yaw,
        axis_label="yaw",
        title=f"Środek kadru vs yaw (roll={roll_deg:.0f}°, pitch={pitch_deg:.0f}°)",
    )

    fig.suptitle(
        f"GPS drona: {lat_uav:.6f}, {lon_uav:.6f}  |  "
        f"process_one_frame → process_target → project_target_cords",
        fontsize=11,
    )

    if output:
        out_path = os.path.abspath(output)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"Zapisano: {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def _print_grid_summary(points: list[GridPoint], lat_uav: float, lon_uav: float, label: str) -> None:
    ok = sum(1 for p in points if p.lat is not None)
    print(f"{label}: zprojektowano {ok}/{len(points)} punktów")
    for p in points:
        if p.lat is None:
            print(f"  [{p.col},{p.row}] cam=({p.cam_x},{p.cam_y}) -> BRAK")
            continue
        east, north = latlon_to_meters(p.lat, p.lon, lat_uav, lon_uav)
        print(
            f"  [{p.col},{p.row}] cam=({p.cam_x:4d},{p.cam_y:4d}) "
            f"scaled=({p.scaled_x:4d},{p.scaled_y:4d}) "
            f"-> E={east:+7.2f}m N={north:+7.2f}m"
        )


def main() -> None:
    geofence = huge_geofence(LAT, LON)

    print(
        f"Projekcja siatki {GRID_COLS}×{GRID_ROWS} "
        f"@ ({LAT:.6f}, {LON:.6f}), alt={ALT}m"
    )
    print(f"  bazowy:    roll={ROLL}° pitch={PITCH}° yaw={YAW}°")
    print(f"  zmodyf.:   roll={ROLL_MOD}° pitch={PITCH_MOD}° yaw={YAW_MOD}°")

    sweep_vals = lambda span: np.linspace(-span, span, SWEEP_STEPS).tolist()

    with suppress_projection_prints():
        drone_base = make_mock_drone(LAT, LON, ALT, ROLL, PITCH, YAW)
        pipeline_base = make_detection_pipeline(drone_base)
        points_base = project_grid_via_process_one_frame(
            pipeline_base, GRID_COLS, GRID_ROWS, geofence, is_bottle=True,
        )

        drone_mod = make_mock_drone(LAT, LON, ALT, ROLL_MOD, PITCH_MOD, YAW_MOD)
        pipeline_mod = make_detection_pipeline(drone_mod)
        points_mod = project_grid_via_process_one_frame(
            pipeline_mod, GRID_COLS, GRID_ROWS, geofence, is_bottle=True,
        )

        sweep_roll = sweep_center_pixel_vs_attitude(
            LAT, LON, ALT, geofence,
            base_roll_deg=ROLL, base_pitch_deg=PITCH, base_yaw_deg=YAW,
            axis="roll", values_deg=sweep_vals(SWEEP_ROLL),
        )
        sweep_pitch = sweep_center_pixel_vs_attitude(
            LAT, LON, ALT, geofence,
            base_roll_deg=ROLL, base_pitch_deg=PITCH, base_yaw_deg=YAW,
            axis="pitch", values_deg=sweep_vals(SWEEP_PITCH),
        )
        sweep_yaw = sweep_center_pixel_vs_attitude(
            LAT, LON, ALT, geofence,
            base_roll_deg=ROLL, base_pitch_deg=PITCH, base_yaw_deg=YAW,
            axis="yaw", values_deg=sweep_vals(SWEEP_YAW),
        )

    _print_grid_summary(points_base, LAT, LON, "Bazowy")
    _print_grid_summary(points_mod, LAT, LON, "Zmodyfikowany")

    plot_projection_analysis(
        points_base,
        points_mod,
        sweep_roll,
        sweep_pitch,
        sweep_yaw,
        lat_uav=LAT,
        lon_uav=LON,
        alt_uav=ALT,
        roll_deg=ROLL,
        pitch_deg=PITCH,
        yaw_deg=YAW,
        roll2_deg=ROLL_MOD,
        pitch2_deg=PITCH_MOD,
        yaw2_deg=YAW_MOD,
        n_cols=GRID_COLS,
        n_rows=GRID_ROWS,
        output=OUTPUT,
        show=SHOW,
    )


if __name__ == "__main__":
    main()
