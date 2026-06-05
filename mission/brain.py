"""
Proces 1 — Mózg (zarządzanie misją). Jedyny komunikujący się z FC.

Fazy:
  SEARCH : lot A->B, odbiór wykryć od Oka, rzut piksel->GPS (interpolacja telemetrii
           do czasu klatki), filtr geofence, klastrowanie.
  VISIT  : nearest_neighbor, dla każdej diody GUIDED-goto + 3 s stabilizacji,
           HINT (rzut odwrotny), START_DECODE, Złota Weryfikacja, retry (do 3x),
           zapis do CSV.
"""
from __future__ import annotations

import csv
import time
import datetime
from typing import List, Dict, Optional

from Application.Services.MatekService import MatekService
from shared.protocol import (Cmd, Evt, PipeMsg, DecodeRequest,
                             CANDIDATE_FREQS, TARGET_FREQS, CLOCK_OFFSET,
                             STABILIZE_S, GOLDEN_DIST_M, MAX_RETRY, OOK_WINDOW_S)
from shared.calibration import K, DIST, CALIB_W, CALIB_H
from shared.geometry import pixel_to_gps, gps_to_pixel, point_in_polygon
from shared.clustering import TargetClusterer, distance_m

# ── Konfiguracja misji (do przeniesienia do config.toml) ──
WP_A = {"lat": 50.271366, "lon": 18.672130}
WP_B = {"lat": 50.270614, "lon": 18.676456}
POLY_PATH = "glowice_dron.poly"
ALT = 50.0
SEARCH_SPEED = 5.0
ARRIVAL_RADIUS_M = 2.5
CSV_PATH = "wyniki_diody.csv"
IMG_CENTER = (CALIB_W / 2.0, CALIB_H / 2.0)


def load_poly(path: str) -> List:
    poly = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                lat, lon = line.split()
                poly.append((float(lat), float(lon)))
    except FileNotFoundError:
        print(f"[brain] UWAGA: brak {path} — filtr geofence wyłączony")
    return poly


def nearest_neighbor(start_lat, start_lon, targets) -> List[Dict]:
    remaining, ordered = list(targets), []
    clat, clon = start_lat, start_lon
    while remaining:
        nxt = min(remaining, key=lambda t: distance_m(clat, clon, t["lat"], t["lon"]))
        ordered.append(nxt); remaining.remove(nxt)
        clat, clon = nxt["lat"], nxt["lon"]
    return ordered


def build_search_nav(alt) -> List[Dict]:
    return [
        {"command": "TAKEOFF", "alt": alt},
        {"command": "WAYPOINT", "lat": WP_A["lat"], "lon": WP_A["lon"], "alt": alt, "acr": 3},
        {"command": "WAYPOINT", "lat": WP_B["lat"], "lon": WP_B["lon"], "alt": alt, "acr": 3},
    ]


def log_result_csv(path, diode_id, lat, lon, freq, is_target, conf, attempts, status):
    new = not _file_exists(path)
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["diode_id", "lat", "lon", "freq_hz", "is_target",
                        "confidence", "attempts", "status", "timestamp"])
        w.writerow([diode_id, f"{lat:.7f}", f"{lon:.7f}",
                    "" if freq is None else freq, is_target,
                    f"{conf:.2f}", attempts, status,
                    datetime.datetime.now().isoformat(timespec="seconds")])


def _file_exists(path):
    import os
    return os.path.exists(path)


# ====================================================================== #
def phase_search(conn, drone: MatekService, poly, b_seq: int) -> List[Dict]:
    clu = TargetClusterer()
    print("[brain] SEARCH aktywny — lecę do B, zbieram wykrycia")
    while True:
        if b_seq in drone.reached_seqs():
            break
        if conn.poll(timeout=0.005):
            msg = conn.recv()
            if msg.kind != Evt.DETECTION:
                continue
            det = msg.payload
            tel = drone.get_telemetry_at(det.ts + CLOCK_OFFSET)
            if tel is None:
                continue   # brak telemetrii dla tego czasu
            gps = pixel_to_gps(det.raw_x, det.raw_y, tel.lat, tel.lon, tel.alt,
                               tel.roll, tel.pitch, tel.yaw, K, DIST, CALIB_W, CALIB_H)
            if gps is None:
                continue
            if poly and not point_in_polygon(gps[0], gps[1], poly):
                continue   # poza strefą
            opt = ((det.raw_x - IMG_CENTER[0]) ** 2 + (det.raw_y - IMG_CENTER[1]) ** 2) ** 0.5
            clu.add(gps[0], gps[1], opt)
    diodes = clu.finalize(min_frames=5, min_frame_ratio=0.25)
    print(f"[brain] SEARCH zakończony — {len(diodes)} diod")
    return diodes


def _wait_arrival(drone, lat, lon, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        tel = drone.get_latest_telemetry()
        if tel and distance_m(tel.lat, tel.lon, lat, lon) <= ARRIVAL_RADIUS_M:
            return True
        time.sleep(0.1)
    return False


def _decode_once(conn, drone, diode) -> Optional[tuple]:
    """Jedna próba OOK. Zwraca (freq, conf, golden_lat, golden_lon, dist) lub None."""
    tel = drone.get_latest_telemetry()
    if tel is None:
        return None
    hint = gps_to_pixel(diode["lat"], diode["lon"], tel.lat, tel.lon, tel.alt,
                        tel.roll, tel.pitch, tel.yaw, K, DIST, CALIB_W, CALIB_H)
    if hint is None:
        hint = IMG_CENTER   # dioda powinna być blisko nadiru
    conn.send(PipeMsg(Cmd.START_DECODE, DecodeRequest(hint[0], hint[1])))

    if not conn.poll(timeout=OOK_WINDOW_S + 3.0):
        return None
    res = conn.recv().payload
    if res.freq_hz is None:
        return (None, res.confidence, None, None, None)

    # Złota Weryfikacja: rzut uśrednionego piksela na GPS, dystans do diody
    hov = drone.get_latest_telemetry()
    golden = pixel_to_gps(res.avg_raw_x, res.avg_raw_y, hov.lat, hov.lon, hov.alt,
                          hov.roll, hov.pitch, hov.yaw, K, DIST, CALIB_W, CALIB_H)
    if golden is None:
        return (res.freq_hz, res.confidence, None, None, None)
    dist = distance_m(golden[0], golden[1], diode["lat"], diode["lon"])
    return (res.freq_hz, res.confidence, golden[0], golden[1], dist)


def phase_visit(conn, drone: MatekService, diodes: List[Dict]) -> None:
    ordered = nearest_neighbor(WP_B["lat"], WP_B["lon"], diodes)
    drone.set_mode("GUIDED")
    print(f"[brain] VISIT — {len(ordered)} diod w kolejności NN")

    for i, diode in enumerate(ordered):
        print(f"[brain] -> dioda {i+1}/{len(ordered)} ({diode['lat']:.6f},{diode['lon']:.6f})")
        drone.goto(diode["lat"], diode["lon"], ALT)
        if not _wait_arrival(drone, diode["lat"], diode["lon"]):
            log_result_csv(CSV_PATH, i, diode["lat"], diode["lon"], None, False, 0, 0, "brak_dolotu")
            continue

        time.sleep(STABILIZE_S)   # stała stabilizacja 3 s

        accepted = False
        for attempt in range(1, MAX_RETRY + 1):
            out = _decode_once(conn, drone, diode)
            if out is None:
                continue
            freq, conf, glat, glon, dist = out
            if freq is None:
                print(f"   próba {attempt}: brak wyraźnego sygnału (conf {conf:.1f})")
                continue
            if dist is not None and dist < GOLDEN_DIST_M:
                is_tgt = round(freq) in TARGET_FREQS
                log_result_csv(CSV_PATH, i, glat, glon, round(freq), is_tgt, conf, attempt, "ok")
                print(f"   ZWERYFIKOWANO: {round(freq)} Hz, cel={is_tgt} (dist {dist:.1f}m, conf {conf:.1f})")
                accepted = True
                break
            else:
                print(f"   próba {attempt}: zniesło ({dist:.1f}m > {GOLDEN_DIST_M}m) — powtarzam")
                drone.goto(diode["lat"], diode["lon"], ALT)
                time.sleep(STABILIZE_S)

        if not accepted:
            log_result_csv(CSV_PATH, i, diode["lat"], diode["lon"], None, False, 0, MAX_RETRY, "niepewny")
            print(f"   dioda {i+1}: nie zweryfikowano po {MAX_RETRY} próbach")


# ====================================================================== #
def run_mission_manager(conn, uart_port: str) -> None:
    drone = MatekService(device=uart_port)
    try:
        drone.request_streams()
        time.sleep(0.5)
        poly = load_poly(POLY_PATH)

        # 1. misja SEARCH
        drone.clear_mission()
        nav = build_search_nav(ALT)
        assert drone.set_waypoints(nav), "upload SEARCH nie powiódł się"
        a_seq, b_seq = 2, 3   # home(0) TAKEOFF(1) A(2) B(3)

        # 2. start
        assert drone.set_mode("GUIDED"), "GUIDED nie powiódł się"
        assert drone.arm(), "arm nie powiódł się"
        assert drone.set_mode("AUTO"), "AUTO nie powiódł się"
        drone.mission_start()
        drone.set_speed(SEARCH_SPEED, speed_type=1)

        # 3. po dolocie do A -> START_SEARCH
        if drone.wait_item_reached({a_seq}, timeout=120):
            conn.send(PipeMsg(Cmd.START_SEARCH))
        else:
            print("[brain] UWAGA: nie wykryto dolotu do A, mimo to startuję SEARCH")
            conn.send(PipeMsg(Cmd.START_SEARCH))

        # 4. SEARCH do B
        diodes = phase_search(conn, drone, poly, b_seq)
        drone.set_mode("GUIDED")          # zatrzymaj się nad B
        conn.send(PipeMsg(Cmd.STOP_SEARCH))

        if not diodes:
            print("[brain] brak diod — RTL")
            return

        # 5. VISIT + OOK
        phase_visit(conn, drone, diodes)

    finally:
        conn.send(PipeMsg(Cmd.SHUTDOWN))
        try:
            drone.set_mode("RTL")
        except Exception:
            pass
        drone.close()
        print("[brain] misja zakończona, RTL")
