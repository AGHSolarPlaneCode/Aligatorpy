#!/usr/bin/env python3
"""
SEGMENT 7 — Lokalizacja GPS (stanowisko C: kamera poziomo, ZNANA geometria).

==== TWÓJ KLUCZOWY TEST ====
Weryfikuje cały łańcuch get_telemetry_at -> pixel_to_gps -> rot_matrix.
To sprawdza KONWENCJĘ OSI — największe nieprzetestowane ryzyko projektu.
Round-trip jest matematycznie spójny (Segment 0), ale to NIE dowodzi zgodności
ze światem fizycznym. Tu to dowodzimy.

WYMAGA: kamera + FC z fixem GPS i sensownymi kątami (dron może stać nieruchomo,
ważne że telemetria płynie i znasz pozycję). Dioda w ZNANYM GPS.

PROCEDURA (wykonaj 2 razy: yaw≈0° północ, potem yaw≈90° wschód):
  1. Zmierz GPS diody (telefon/RTK/taśma od znanego punktu) — wpisz --diode-lat/lon
  2. Ustaw kamerę w znanym GPS i wysokości, skierowaną tak by dioda była w kadrze
  3. Uruchom skrypt — złapie piksel, pobierze telemetrię, policzy pixel_to_gps
  4. Porównaj wynik z prawdziwym GPS diody

Uruchomienie:
    python -m tests.seg7_localize --uart udp:... \\
        --diode-lat 50.27110 --diode-lon 18.67276 --alt 50

KRYTERIUM PRZEJŚCIA:
  [1] wynik w granicach ~2-5 m od prawdziwego GPS diody
  [2] KRYTYCZNE: błąd NIE rośnie systematycznie przy zmianie yaw 0->90°
      (rosnący błąd przy obrocie = błąd konwencji w rot_matrix)

JEŚLI FAIL:
  - stały offset niezależny od yaw -> błąd pomiaru GPS diody lub kalibracji
  - offset rosnący z yaw -> NAPRAW znaki/kolejność osi w shared/geometry.rot_matrix
  - offset rosnący z pitch/roll -> błąd w obrocie lub przecięciu z ziemią
"""
import argparse
import threading
import time

import numpy as np

from vision.camera_handler import CameraPipeline, WIDTH, HEIGHT
import vision.eye as eye
from Application.Services.MatekService import MatekService
from shared.geometry import pixel_to_gps
from shared.calibration import K, DIST, CALIB_W, CALIB_H
from shared.clustering import distance_m
from shared.protocol import SEARCH_THRESHOLD, CLOCK_OFFSET


def capture_blob_pixel(cam, threshold, n_frames=30):
    """Uśrednia piksel diody z kilku klatek dla stabilności."""
    xs, ys = [], []
    got = 0
    end = time.time() + 5.0
    while got < n_frames and time.time() < end:
        f1d, ts, err = cam.get_image()
        frame = eye._reshape(f1d)
        if frame is None:
            continue
        blob = eye.detect_blob(frame, threshold)
        if blob is not None:
            xs.append(blob[0]); ys.append(blob[1]); got += 1
    if not xs:
        return None
    return (float(np.mean(xs)), float(np.mean(ys)), got)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="udp:127.0.0.1:14551")
    ap.add_argument("--diode-lat", type=float, required=True)
    ap.add_argument("--diode-lon", type=float, required=True)
    ap.add_argument("--alt", type=float, default=50.0,
                    help="wysokość kamery nad płaszczyzną diody [m]")
    ap.add_argument("--threshold", type=int, default=SEARCH_THRESHOLD)
    args = ap.parse_args()

    # kamera
    cam = CameraPipeline()
    threading.Thread(target=cam.start, daemon=True).start()
    time.sleep(2.0)
    cam.set_search_active(True)
    time.sleep(0.5)

    # FC
    drone = MatekService(device=args.uart)
    drone.request_streams()
    time.sleep(2.0)

    try:
        # 1. piksel diody
        print("Łapię piksel diody (uśredniam 30 klatek)...")
        blob = capture_blob_pixel(cam, args.threshold)
        if blob is None:
            print("[FAIL] nie wykryto diody — sprawdź --threshold i czy dioda w kadrze")
            return 1
        px, py, got = blob
        print(f"    piksel = ({px:.1f}, {py:.1f}) z {got} klatek")

        # 2. telemetria
        tel = drone.get_latest_telemetry()
        if tel is None:
            print("[FAIL] brak telemetrii FC")
            return 1
        print(f"    telemetria: lat={tel.lat:.6f} lon={tel.lon:.6f} "
              f"alt={tel.alt:.1f}")
        print(f"                roll={tel.roll:.3f} pitch={tel.pitch:.3f} "
              f"yaw={tel.yaw:.3f} ({np.degrees(tel.yaw):.0f}°)")

        # 3. rzutowanie — użyj wysokości z argumentu (kamera nad płaszczyzną diody)
        gps = pixel_to_gps(px, py, tel.lat, tel.lon, args.alt,
                           tel.roll, tel.pitch, tel.yaw,
                           K, DIST, CALIB_W, CALIB_H)
        if gps is None:
            print("[FAIL] pixel_to_gps zwrócił None (promień w górę? sprawdź kąty)")
            return 1

        # 4. porównanie
        err = distance_m(gps[0], gps[1], args.diode_lat, args.diode_lon)
        print(f"\n    Rzutowany GPS:  {gps[0]:.6f}, {gps[1]:.6f}")
        print(f"    Prawdziwy GPS:  {args.diode_lat:.6f}, {args.diode_lon:.6f}")
        print(f"    BŁĄD: {err:.2f} m")

        print("\n" + "=" * 60)
        p = err < 5.0
        print(f"  [{'PASS' if p else 'FAIL'}] błąd < 5 m  (yaw={np.degrees(tel.yaw):.0f}°)")
        print("\nWAŻNE: powtórz ten test dla yaw≈0° I yaw≈90°.")
        print("Jeśli błąd przy yaw=0 mały, a przy yaw=90 duży -> błąd konwencji")
        print("w rot_matrix (shared/geometry.py). Zapisz oba wyniki i porównaj.")
        return 0 if p else 1
    finally:
        cam.stop()
        drone.close()


if __name__ == "__main__":
    raise SystemExit(main())
