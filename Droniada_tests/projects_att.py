#!/usr/bin/env python3
"""
Test projekcji piksel -> GPS w ukladzie jak na wgranym zdjeciu, ale dla
POPRAWIONEJ geometrii + porownanie z ORYGINALEM.

Zalozenie: GORA kamery = yaw 0  (gora obrazu wskazuje North przy yaw=0).

Generuje 3 figury:
  fig1_roll.png   - bazowy(0,0,0) vs zmodyfikowany roll
  fig2_pitch.png  - bazowy(0,0,0) vs zmodyfikowany pitch
  fig3_yaw.png    - bazowy(0,0,0) vs zmodyfikowany yaw
Kazda figura: rzad bazowy + rzad zmodyfikowany (siatka pikseli z numeracja
punktow kontrolnych + projekcja na ziemie), oraz dolny rzad sweepow.
Punkty kontrolne sa numerowane, by widziec jak konkretny piksel wedruje.

Dodatkowo fig0_compare_orig_vs_fix.png pokazuje te sama siatke w ORIG i FIX.
"""
from __future__ import annotations
import numpy as np
import cv2
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ── kalibracja ────────────────────────────────────────────────────────────
W, H = 640, 400
K = np.array([[346.2295, 0.0, 317.76],
              [0.0, 346.2628, 197.64],
              [0.0, 0.0, 1.0]])
DIST = np.array([-0.34219431, 0.123526508, 0.00470056821,
                 -0.000109821856, -0.0211505931])

# ── scenariusz ──────────────────────────────────────────────────────────
LAT_UAV, LON_UAV, ALT_UAV = 52.2297, 21.0122, 50.0

# kamera nadir, gora obrazu = North przy yaw=0
R_CAM_TO_BODY = np.array([[0.0, -1.0, 0.0],   # north = -Y_cam (gora obrazu)
                          [1.0,  0.0, 0.0],   # east  = +X_cam (prawo obrazu)
                          [0.0,  0.0, 1.0]])  # down  = +Z_cam


# ── macierze obrotu ───────────────────────────────────────────────────────
def rot_ned(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_orig(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def undistort_ray(u, v):
    un = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), K, DIST)[0, 0]
    r = np.array([un[0], un[1], 1.0])
    return r / np.linalg.norm(r)


# ── projekcje (zwracaja north, east) ───────────────────────────────────────
def project_fixed(u, v, roll, pitch, yaw):
    if not (0 <= u < W and 0 <= v < H):
        return None
    ray = undistort_ray(u, v)
    rw = rot_ned(roll, pitch, yaw) @ (R_CAM_TO_BODY @ ray)
    if rw[2] <= 1e-9:
        return None
    t = ALT_UAV / rw[2]
    h = rw * t
    return h[0], h[1]      # north, east


def project_orig(u, v, roll, pitch, yaw):
    if not (0 <= u < W and 0 <= v < H):
        return None
    ray = undistort_ray(u, v)
    rw = rot_orig(roll, pitch, yaw) @ ray
    if abs(rw[2]) < 1e-6 or rw[2] <= 0:
        return None
    t = ALT_UAV / rw[2]
    h = rw * t
    # w oryginale do gps_offset szlo (hit[0],hit[1]) jako (east,north):
    # efektywny east=h[0], north=h[1]
    eff_north, eff_east = h[1], h[0]
    return eff_north, eff_east


# ── siatka punktow kontrolnych ────────────────────────────────────────────
COLS, ROWS = 9, 6


def control_pixels():
    pts = []
    idx = 0
    for j in range(ROWS):
        for i in range(COLS):
            u = (i + 0.5) * W / COLS
            v = (j + 0.5) * H / ROWS
            pts.append((idx, u, v))
            idx += 1
    return pts


def project_grid(project_fn, roll, pitch, yaw):
    out = []
    for idx, u, v in control_pixels():
        res = project_fn(u, v, roll, pitch, yaw)
        if res is None:
            out.append((idx, u, v, None, None))
        else:
            out.append((idx, u, v, res[0], res[1]))  # north, east
    return out


# ── rysowanie pojedynczych paneli ──────────────────────────────────────────
def draw_pixels(ax, grid, title):
    for idx, u, v, n, e in grid:
        col = "tab:red" if n is not None else "lightgray"
        ax.scatter(u, v, c=col, s=28, zorder=3)
        if idx % 2 == 0:
            ax.text(u + 6, v + 6, str(idx), fontsize=6, color="navy")
    ax.scatter(W/2, H/2, c="lime", marker="+", s=140, linewidths=2.5,
               zorder=5, label="srodek")
    ax.add_patch(Rectangle((0, 0), W, H, fill=False, ls="--", ec="k"))
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.set_aspect("equal")
    ax.set_title(title); ax.set_xlabel("cam_x [px]"); ax.set_ylabel("cam_y [px]")
    ax.legend(loc="upper right", fontsize=7)


def draw_ground(ax, grid, title):
    for idx, u, v, n, e in grid:
        if n is None:
            continue
        ax.scatter(e, n, c="tab:blue", s=28, zorder=3)
        if idx % 2 == 0:
            ax.text(e + 1, n + 1, str(idx), fontsize=6)
    ax.scatter(0, 0, c="red", marker="^", s=130, label="dron (nadir)", zorder=5)
    ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_title(title); ax.legend(loc="upper right", fontsize=7)


def sweep(project_fn, axis, span, steps=41):
    vals = np.linspace(-span, span, steps)
    north, east = [], []
    for a in vals:
        r = math.radians(a)
        rpy = {"roll": (r, 0, 0), "pitch": (0, r, 0), "yaw": (0, 0, r)}[axis]
        res = project_fn(W/2, H/2, *rpy)
        if res is None:
            north.append(np.nan); east.append(np.nan)
        else:
            north.append(res[0]); east.append(res[1])
    return vals, north, east


def draw_sweep(ax, vals, north, east, axis, note=""):
    ax.plot(vals, east, "o-", color="tab:blue", ms=3, label="East [m]")
    ax.plot(vals, north, "s-", color="tab:orange", ms=3, label="North [m]")
    ax.axhline(0, color="gray", lw=0.5)
    ax.grid(alpha=0.3); ax.set_xlabel(f"{axis} [deg]")
    ax.set_ylabel("offset srodka kadru [m]")
    ax.set_title(f"Srodek kadru vs {axis} {note}")
    ax.legend(fontsize=7)


# ── budowa figury per os ───────────────────────────────────────────────────
def build_axis_figure(axis, mod_deg, sweep_span, fname):
    project_fn = project_fixed
    base = project_grid(project_fn, 0, 0, 0)
    r = math.radians(mod_deg)
    rpy_mod = {"roll": (r, 0, 0), "pitch": (0, r, 0), "yaw": (0, 0, r)}[axis]
    mod = project_grid(project_fn, *rpy_mod)

    fig = plt.figure(figsize=(15, 13))
    gs = fig.add_gridspec(3, 6, height_ratios=[1.2, 1.2, 1.0],
                          hspace=0.4, wspace=0.45)
    ax0 = fig.add_subplot(gs[0, 0:3]); ax1 = fig.add_subplot(gs[0, 3:6])
    ax2 = fig.add_subplot(gs[1, 0:3]); ax3 = fig.add_subplot(gs[1, 3:6])
    axr = fig.add_subplot(gs[2, 0:2]); axp = fig.add_subplot(gs[2, 2:4])
    axy = fig.add_subplot(gs[2, 4:6])

    draw_pixels(ax0, base, f"Kamera {W}x{H} — bazowy (0,0,0)")
    draw_ground(ax1, base, "Projekcja bazowa\n@ 50 m | roll=0 pitch=0 yaw=0")
    draw_pixels(ax2, mod, f"Kamera {W}x{H} — {axis}={mod_deg}deg")
    draw_ground(ax3, mod,
                f"Projekcja zmodyfikowana\n@ 50 m | {axis}={mod_deg}deg (reszta 0)")

    vr, nr, er = sweep(project_fn, "roll", sweep_span)
    vp, npi, ep = sweep(project_fn, "pitch", sweep_span)
    vy, ny, ey = sweep(project_fn, "yaw", min(sweep_span, 45))
    draw_sweep(axr, vr, nr, er, "roll", "(pitch=0,yaw=0)")
    draw_sweep(axp, vp, npi, ep, "pitch", "(roll=0,yaw=0)")
    draw_sweep(axy, vy, ny, ey, "yaw", "(roll=0,pitch=0)")

    fig.suptitle(
        f"POPRAWIONA projekcja | GPS drona {LAT_UAV:.6f},{LON_UAV:.6f} | "
        f"test osi: {axis.upper()}={mod_deg}deg | gora kamery = yaw 0",
        fontsize=12)
    fig.savefig(fname, dpi=135, bbox_inches="tight")
    plt.close(fig)
    print("zapisano", fname)


# ── figura porownawcza ORIG vs FIX (bazowa, 0,0,0) ─────────────────────────
def build_compare_figure(fname):
    g_orig = project_grid(project_orig, 0, 0, 0)
    g_fix = project_grid(project_fixed, 0, 0, 0)
    # dodatkowo z roll, by pokazac rozjazd
    g_orig_r = project_grid(project_orig, math.radians(20), 0, 0)
    g_fix_r = project_grid(project_fixed, math.radians(20), 0, 0)

    fig = plt.figure(figsize=(14, 11))
    gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
    a = fig.add_subplot(gs[0, 0]); b = fig.add_subplot(gs[0, 1])
    c = fig.add_subplot(gs[1, 0]); d = fig.add_subplot(gs[1, 1])

    def gmap(ax, g, title, color):
        for idx, u, v, n, e in g:
            if n is None:
                continue
            ax.scatter(e, n, c=color, s=26, zorder=3)
            if idx % 2 == 0:
                ax.text(e + 1, n + 1, str(idx), fontsize=6)
        ax.scatter(0, 0, c="red", marker="^", s=120, zorder=5, label="dron")
        ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
        ax.set_title(title); ax.legend(fontsize=7)

    gmap(a, g_orig, "ORYGINAL @ (0,0,0)\n(os North odbita)", "tab:orange")
    gmap(b, g_fix, "POPRAWIONA @ (0,0,0)", "tab:blue")
    gmap(c, g_orig_r, "ORYGINAL @ roll=20deg", "tab:orange")
    gmap(d, g_fix_r, "POPRAWIONA @ roll=20deg", "tab:blue")

    fig.suptitle("Porownanie ORYGINAL vs POPRAWIONA (ta sama siatka pikseli)",
                 fontsize=13)
    fig.savefig(fname, dpi=135, bbox_inches="tight")
    plt.close(fig)
    print("zapisano", fname)


if __name__ == "__main__":
    build_compare_figure("fig0_compare_orig_vs_fix.png")
    build_axis_figure("roll", 20, 80, "fig1_roll.png")
    build_axis_figure("pitch", 20, 80, "fig2_pitch.png")
    build_axis_figure("yaw", 30, 45, "fig3_yaw.png")