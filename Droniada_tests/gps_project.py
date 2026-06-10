#!/usr/bin/env python3
"""
Test i diagnostyka projekcji  piksel -> GPS  dla kamery nadir.

Sprawdza:
  1) Jaki jest faktyczny uklad odniesienia kamery (mapowanie osi obrazu na N/E).
  2) Czy oryginalna funkcja project_target_cords lokalizuje cel poprawnie.
  3) Poprawiona wersja projekcji (z jawna macierza camera->body i wlasciwa
     kolejnoscia argumentow gps_offset).
  4) Test "round-trip": GPS celu -> piksel -> GPS  (blad powinien byc ~0).

Wszystko zobrazowane na wykresach -> projection_diagnostics.png
"""
from __future__ import annotations
import numpy as np
import cv2
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pymavlink import mavextra

# ──────────────────────────────────────────────────────────────────────────
#  DANE KALIBRACYJNE (z polecenia uzytkownika)
# ──────────────────────────────────────────────────────────────────────────
RESOLUTION = (640, 400)
W, H = RESOLUTION
K = np.array([
    [346.2295, 0.0,      317.76],
    [0.0,      346.2628, 197.64],
    [0.0,      0.0,      1.0],
])
DIST = np.array([-0.34219431, 0.123526508, 0.00470056821,
                 -0.000109821856, -0.0211505931])

# ──────────────────────────────────────────────────────────────────────────
#  DANE SCENARIUSZA TESTOWEGO (dobrane tak, by sie zgadzaly)
#  Dron wisi nad punktem, kamera patrzy w dol (nadir).
# ──────────────────────────────────────────────────────────────────────────
LAT_UAV = 52.230000      # szerokosc drona
LON_UAV = 21.010000      # dlugosc drona
ALT_UAV = 50.0           # wysokosc AGL [m]
ROLL_DEG, PITCH_DEG, YAW_DEG = 0.0, 0.0, 0.0   # orientacja


# ══════════════════════════════════════════════════════════════════════════
#  WERSJA ORYGINALNA  (skopiowana 1:1 z MissionService — do porownania)
# ══════════════════════════════════════════════════════════════════════════
def rot_matrix_orig(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll),  np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def project_orig(pixel, lat_uav, lon_uav, alt_uav, roll, pitch, yaw):
    u, v = pixel
    if not (0 <= u < W and 0 <= v < H):
        return None
    und = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), K, DIST)
    x_u, y_u = und[0, 0]
    ray_cam = np.array([x_u, y_u, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)
    ray_world = rot_matrix_orig(roll, pitch, yaw) @ ray_cam
    dz = ray_world[2]
    if abs(dz) < 1e-6 or dz <= 0:
        return None
    t = alt_uav / dz
    hit = ray_world * t                      # kod twierdzi [north, east, down]
    lat_t, lon_t = mavextra.gps_offset(lat_uav, lon_uav,
                                       hit[0],   # przekazane jako EAST !!
                                       hit[1])   # przekazane jako NORTH!!
    return lat_t, lon_t


# ══════════════════════════════════════════════════════════════════════════
#  WERSJA POPRAWIONA
#  - jawna macierz camera->body (kamera OpenCV: X prawo, Y dol, Z przod==w dol)
#  - body w konwencji NED: X north, Y east, Z down
#  - poprawna kolejnosc argumentow gps_offset(lat,lon, EAST, NORTH)
# ══════════════════════════════════════════════════════════════════════════
def rot_matrix_ned(roll, pitch, yaw):
    """Body(NED) -> World(NED). Standardowa aerospace 3-2-1 (Z-Y-X)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# Kamera nadir: oś optyczna kamery (Z_cam, "przod") wskazuje w DOL (NED +Z).
# Przyjmujemy montaz: gora obrazu = przod kadlubа (north przy yaw=0),
#   prawo obrazu = prawo kadlubа (east przy yaw=0).
# Kamera OpenCV:  X_cam -> prawo,  Y_cam -> dol obrazu,  Z_cam -> przod(=w dol)
# Mapowanie na body NED:
#   X_cam (prawo)      -> +Y_body (east)
#   Y_cam (dol obrazu) -> -X_body (a wiec "gora obrazu" = +north)
#   Z_cam (w dol)      -> +Z_body (down)
R_CAM_TO_BODY = np.array([
    [0.0, -1.0, 0.0],   # north_body  =  -Y_cam
    [1.0,  0.0, 0.0],   # east_body   =  +X_cam
    [0.0,  0.0, 1.0],   # down_body   =  +Z_cam
])


def project_fixed(pixel, lat_uav, lon_uav, alt_uav, roll, pitch, yaw):
    u, v = pixel
    if not (0 <= u < W and 0 <= v < H):
        return None
    und = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), K, DIST)
    x_u, y_u = und[0, 0]
    ray_cam = np.array([x_u, y_u, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)
    ray_body = R_CAM_TO_BODY @ ray_cam
    ray_world = rot_matrix_ned(roll, pitch, yaw) @ ray_body   # [N, E, D]
    dN, dE, dD = ray_world
    if dD <= 1e-9:
        return None
    t = alt_uav / dD
    north = dN * t
    east = dE * t
    # pymavlink: gps_offset(lat, lon, EAST, NORTH)
    lat_t, lon_t = mavextra.gps_offset(lat_uav, lon_uav, east, north)
    return lat_t, lon_t, north, east


# ──────────────────────────────────────────────────────────────────────────
#  POMOCNICZE
# ──────────────────────────────────────────────────────────────────────────
def latlon_to_ne(lat, lon, lat0, lon0):
    """Lokalny rzut plaski (north, east) w metrach wzgledem (lat0,lon0)."""
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * R
    east = dlon * R * math.cos(math.radians(lat0))
    return north, east


def undistort_ray(u, v):
    und = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), K, DIST)
    x_u, y_u = und[0, 0]
    r = np.array([x_u, y_u, 1.0]); r /= np.linalg.norm(r)
    return r


# ══════════════════════════════════════════════════════════════════════════
#  1) DIAGNOSTYKA UKLADU ODNIESIENIA KAMERY
# ══════════════════════════════════════════════════════════════════════════
def diagnose_frame():
    print("=" * 70)
    print(" DIAGNOSTYKA UKLADU ODNIESIENIA KAMERY (orientacja zero, nadir)")
    print("=" * 70)
    probes = {
        "CENTRUM     (u=320,v=200)": (320, 200),
        "PRAWO       (u=600,v=200)": (600, 200),
        "LEWO        (u= 40,v=200)": (40, 200),
        "GORA obrazu (u=320,v= 40)": (320, 40),
        "DOL obrazu  (u=320,v=360)": (320, 360),
    }
    print("\n  --- ORYGINAL (hit[0],hit[1] przekazane do gps_offset jak w kodzie) ---")
    for name, (u, v) in probes.items():
        r = undistort_ray(u, v)
        rw = rot_matrix_orig(0, 0, 0) @ r
        t = ALT_UAV / rw[2]
        hit = rw * t
        # kod: gps_offset(lat,lon, hit[0], hit[1]) => hit[0]=EAST, hit[1]=NORTH
        eff_east, eff_north = hit[0], hit[1]
        print(f"   {name}: world=({hit[0]:+6.1f},{hit[1]:+6.1f}) "
              f"-> efekt: EAST={eff_east:+6.1f}  NORTH={eff_north:+6.1f}")

    print("\n  --- POPRAWIONA ---")
    for name, (u, v) in probes.items():
        res = project_fixed((u, v), LAT_UAV, LON_UAV, ALT_UAV, 0, 0, 0)
        _, _, north, east = res
        print(f"   {name}: NORTH={north:+6.1f}  EAST={east:+6.1f}")
    print()


# ══════════════════════════════════════════════════════════════════════════
#  2) TEST ROUND-TRIP:  GPS celu -> piksel -> GPS
# ══════════════════════════════════════════════════════════════════════════
def world_point_to_pixel(north, east, lat_uav, lon_uav, alt_uav, roll, pitch, yaw):
    """Odwrotnosc: znany punkt na ziemi (N,E wzgl. drona) -> piksel kamery."""
    # wektor od kamery do punktu w world NED
    vec_world = np.array([north, east, alt_uav])  # down=+alt (punkt nizej)
    R_wb = rot_matrix_ned(roll, pitch, yaw)
    vec_body = R_wb.T @ vec_world
    vec_cam = R_CAM_TO_BODY.T @ vec_body
    if vec_cam[2] <= 0:
        return None
    xn = vec_cam[0] / vec_cam[2]
    yn = vec_cam[1] / vec_cam[2]
    pts = np.array([[[xn, yn]]], dtype=np.float32)
    # projectPoints z zerowym R,t (juz w ukladzie kamery, znormalizowane)
    img, _ = cv2.projectPoints(
        np.array([[xn, yn, 1.0]], dtype=np.float32),
        np.zeros(3), np.zeros(3), K, DIST)
    u, v = img[0, 0]
    return u, v


def roundtrip_test():
    print("=" * 70)
    print(" TEST ROUND-TRIP:  cel GPS -> piksel -> projekcja -> GPS")
    print("=" * 70)
    # Zadajemy cele jako offset N/E wzgledem drona (musza miescic sie w kadrze)
    targets_ne = [(0, 0), (10, 0), (0, 10), (-8, 6), (12, -9)]
    rows = []
    for north, east in targets_ne:
        # prawdziwy GPS celu
        tlat, tlon = mavextra.gps_offset(LAT_UAV, LON_UAV, east, north)
        px = world_point_to_pixel(north, east, LAT_UAV, LON_UAV, ALT_UAV,
                                   0, 0, 0)
        if px is None:
            continue
        u, v = px
        # ORYGINAL
        o = project_orig((u, v), LAT_UAV, LON_UAV, ALT_UAV, 0, 0, 0)
        # POPRAWIONA
        f = project_fixed((u, v), LAT_UAV, LON_UAV, ALT_UAV, 0, 0, 0)
        on, oe = latlon_to_ne(o[0], o[1], LAT_UAV, LON_UAV)
        fn, fe = latlon_to_ne(f[0], f[1], LAT_UAV, LON_UAV)
        err_o = math.hypot(on - north, oe - east)
        err_f = math.hypot(fn - north, fe - east)
        rows.append((north, east, u, v, on, oe, err_o, fn, fe, err_f))
        print(f"  cel N={north:+5.1f} E={east:+5.1f}  px=({u:6.1f},{v:6.1f})")
        print(f"     ORIG -> N={on:+7.1f} E={oe:+7.1f}  blad={err_o:8.1f} m")
        print(f"     FIX  -> N={fn:+7.1f} E={fe:+7.1f}  blad={err_f:8.3f} m")
    print()
    return rows


# ══════════════════════════════════════════════════════════════════════════
#  3) WYKRESY
# ══════════════════════════════════════════════════════════════════════════
def make_grid(project_fn, roll, pitch, yaw, cols=9, rows=6):
    pts = []
    for j in range(rows):
        for i in range(cols):
            u = (i + 0.5) * W / cols
            v = (j + 0.5) * H / rows
            res = project_fn((u, v), LAT_UAV, LON_UAV, ALT_UAV, roll, pitch, yaw)
            if res is None:
                continue
            lat, lon = res[0], res[1]
            n, e = latlon_to_ne(lat, lon, LAT_UAV, LON_UAV)
            pts.append((u, v, n, e))
    return pts


def plot_all(rows):
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(2, 3, hspace=0.32, wspace=0.3)

    # siatki
    grid_orig = make_grid(project_orig, 0, 0, 0)
    grid_fix = make_grid(project_fixed, 0, 0, 0)

    # -- (0,0) siatka pikseli --
    ax = fig.add_subplot(gs[0, 0])
    for u, v, *_ in grid_fix:
        ax.scatter(u, v, c="tab:red", s=30, zorder=3)
    ax.scatter(W/2, H/2, c="lime", marker="+", s=160, linewidths=3,
               zorder=4, label="centrum")
    ax.add_patch(plt.Rectangle((0, 0), W, H, fill=False, ls="--", ec="k"))
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.set_aspect("equal")
    ax.set_title("Siatka pikseli (kamera 640x400)")
    ax.set_xlabel("u [px]"); ax.set_ylabel("v [px]"); ax.legend()

    # -- (0,1) projekcja ORYGINAL na ziemie --
    ax = fig.add_subplot(gs[0, 1])
    for u, v, n, e in grid_orig:
        ax.scatter(e, n, c="tab:orange", s=30, zorder=3)
    ax.scatter(0, 0, c="red", marker="^", s=140, label="dron", zorder=5)
    ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title("ORYGINAL: projekcja na ziemie\n(osie zamienione!)")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.legend()

    # -- (0,2) projekcja POPRAWIONA na ziemie --
    ax = fig.add_subplot(gs[0, 2])
    for u, v, n, e in grid_fix:
        ax.scatter(e, n, c="tab:blue", s=30, zorder=3)
    ax.scatter(0, 0, c="red", marker="^", s=140, label="dron", zorder=5)
    ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title("POPRAWIONA: projekcja na ziemie")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.legend()

    # -- (1,0) round-trip bledy --
    ax = fig.add_subplot(gs[1, 0])
    idx = range(len(rows))
    err_o = [r[6] for r in rows]
    err_f = [r[9] for r in rows]
    width = 0.38
    ax.bar([i - width/2 for i in idx], err_o, width, label="ORIG", color="tab:orange")
    ax.bar([i + width/2 for i in idx], err_f, width, label="FIX", color="tab:blue")
    ax.set_yscale("symlog")
    ax.set_title("Blad round-trip (cel->px->GPS)")
    ax.set_xlabel("nr celu"); ax.set_ylabel("blad [m] (symlog)")
    ax.legend(); ax.grid(alpha=0.3, axis="y")

    # -- (1,1) round-trip mapa: zadane vs odtworzone --
    ax = fig.add_subplot(gs[1, 1])
    for r in rows:
        n0, e0 = r[0], r[1]
        on, oe = r[4], r[5]
        fn, fe = r[7], r[8]
        ax.scatter(e0, n0, c="k", marker="*", s=180, zorder=5)
        ax.scatter(oe, on, c="tab:orange", s=40, zorder=4)
        ax.scatter(fe, fn, c="tab:blue", s=40, zorder=4)
    ax.scatter([], [], c="k", marker="*", s=120, label="zadany cel")
    ax.scatter([], [], c="tab:orange", label="ORIG")
    ax.scatter([], [], c="tab:blue", label="FIX")
    ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title("Round-trip: zadane vs odtworzone")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.legend(fontsize=8)

    # -- (1,2) sweep: srodek kadru vs roll (FIX) --
    ax = fig.add_subplot(gs[1, 2])
    angles = np.linspace(-60, 60, 41)
    north_r, east_r = [], []
    for a in angles:
        res = project_fixed((W/2, H/2), LAT_UAV, LON_UAV, ALT_UAV,
                            math.radians(a), 0, 0)
        if res is None:
            north_r.append(np.nan); east_r.append(np.nan); continue
        north_r.append(res[2]); east_r.append(res[3])
    ax.plot(angles, north_r, "o-", label="North", color="tab:orange", ms=3)
    ax.plot(angles, east_r, "s-", label="East", color="tab:blue", ms=3)
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_title("FIX: srodek kadru vs ROLL\n(roll dodatni -> przesuw na East)")
    ax.set_xlabel("roll [deg]"); ax.set_ylabel("offset [m]")
    ax.grid(alpha=0.3); ax.legend()

    fig.suptitle(
        f"Diagnostyka projekcji piksel->GPS   |   dron @ "
        f"({LAT_UAV:.5f}, {LON_UAV:.5f}), alt={ALT_UAV:.0f} m, nadir",
        fontsize=13)
    fig.savefig("projection_diagnostics.png", dpi=140, bbox_inches="tight")
    print("Zapisano wykres: projection_diagnostics.png")


if __name__ == "__main__":
    diagnose_frame()
    rows = roundtrip_test()
    plot_all(rows)