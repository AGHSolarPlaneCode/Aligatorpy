#!/usr/bin/env python3
"""
Live test: odczyt telemetrii GPS + attitude z kontrolera lotu.
Opcjonalnie uruchamia kamerę w tle (10 fps) — nie blokuje odczytu telemetrii.

Użycie:
    python Droniada_tests/live/live_telemetry.py
    python Droniada_tests/live/live_telemetry.py --with-camera
    python Droniada_tests/live/live_telemetry.py --device /dev/ttyAMA2 --seconds 30
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.GiCameraService import GiCameraService
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg


def _camera_loop(camera: GiCameraService, stop: threading.Event):
    frame_count = 0
    while not stop.is_set():
        frame, ts = camera.get_frame()
        if frame is not None:
            frame_count += 1
            if frame_count % 50 == 0:
                print(f"[KAMERA] Klatka #{frame_count}, ts={ts:.3f}s, size={frame.size}")
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="Live telemetry test")
    parser.add_argument("--device", default=None, help="MAVLink device (default: cfg.mav.device)")
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument("--with-camera", action="store_true", help="Start camera in background")
    parser.add_argument("--seconds", type=float, default=0, help="Duration (0 = until Ctrl+C)")
    parser.add_argument("--hz", type=float, default=10.0, help="Telemetry read rate")
    args = parser.parse_args()

    device = args.device or cfg.mav.device
    baud = args.baud or cfg.mav.baud

    print(f"Łączenie z kontrolerem lotu: {device} @ {baud}")
    drone = MatekService(device=device)
    print(device)
    drone.set_mission_current_rate(10)

    # camera = None
    # cam_stop = threading.Event()
    # cam_thread = None

    # if args.with_camera:
    #     print("Uruchamiam kamerę w tle (10 fps)...")
    #     camera = GiCameraService()
    #     camera.start()
    #     camera.set_10fps_mode()
    #     cam_thread = threading.Thread(
    #         target=_camera_loop, args=(camera, cam_stop), daemon=True
    #     )
        # cam_thread.start()

    interval = 1.0 / 10
    start = time.monotonic()
    read_count = 0

    try:
        while True:
            loop_start = time.monotonic()

            curr_wp = drone.get_mission_status()

            coords = drone.get_current_coordinates(timeout=0.8)
            attitude = drone.get_attitude(timeout=0.8)
            read_count += 1

            if coords and attitude:
                lat, lon, alt = coords
                roll, pitch, yaw = attitude
                print(
                    f"[TELEMETRIA #{read_count}] "
                    f"GPS=({lat:.7f}, {lon:.7f}, alt={alt:.1f}m) "
                    f"att=(r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}) "
                    f"wp={curr_wp}"
                )
            else:
                print(f"[TELEMETRIA #{read_count}] Brak danych GPS lub attitude")

            if args.seconds and (time.monotonic() - start) >= args.seconds:
                print(f"Osiągnięto limit {args.seconds}s.")
                break

            elapsed = time.monotonic() - loop_start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika.")
    finally:
        # cam_stop.set()
        # if cam_thread:
        #     cam_thread.join(timeout=3)
        # if camera:
        #     camera.stop()
        drone.close()
        print(f"Podsumowanie: {read_count} odczytów telemetrii.")


if __name__ == "__main__":
    main()
