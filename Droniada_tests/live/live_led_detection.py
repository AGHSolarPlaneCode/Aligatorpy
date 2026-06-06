#!/usr/bin/env python3
"""
Live test: detekcja diod LED z kamery GStreamer.
Wypisuje informację za każdym razem gdy wykryje nową lub zaktualizowaną diodę.

Użycie:
    python Droniada_tests/live/live_led_detection.py
    python Droniada_tests/live/live_led_detection.py --frames 100
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.GiCameraService import GiCameraService
from Application.Services.led_detector import LedDetector


def main():
    parser = argparse.ArgumentParser(description="Live LED detection test")
    parser.add_argument("--frames", type=int, default=0, help="Max frames (0 = unlimited, Ctrl+C to stop)")
    parser.add_argument("--fps", type=float, default=10.0, help="Target loop rate")
    args = parser.parse_args()

    camera = GiCameraService()
    detector = LedDetector(camera.WIDTH, camera.HEIGHT)
    seen_ids: set[int] = set()

    print(f"Uruchamiam kamerę ({camera.WIDTH}x{camera.HEIGHT}) w trybie 10 fps...")
    camera.start()
    camera.set_10fps_mode()

    interval = 1.0 / args.fps
    frame_count = 0

    try:
        while True:
            loop_start = time.monotonic()

            frame, ts = camera.get_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            targets = detector.process_frame(frame)
            frame_count += 1

            for target in targets:
                if target["frames_unseen"] != 0:
                    continue

                tid = target["id"]
                if tid not in seen_ids:
                    seen_ids.add(tid)
                    print(
                        f"[DIODA] Wykryto nową diodę ID={tid} "
                        f"pixel=({target['x']}, {target['y']}) ts={ts:.3f}s"
                    )
                else:
                    print(
                        f"[DIODA] Aktualizacja ID={tid} "
                        f"pixel=({target['x']}, {target['y']}) ts={ts:.3f}s"
                    )

            if args.frames and frame_count >= args.frames:
                print(f"Osiągnięto limit {args.frames} klatek.")
                break

            elapsed = time.monotonic() - loop_start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika.")
    finally:
        camera.stop()
        print(f"Podsumowanie: {len(seen_ids)} unikalnych diod w {frame_count} klatkach.")


if __name__ == "__main__":
    main()
