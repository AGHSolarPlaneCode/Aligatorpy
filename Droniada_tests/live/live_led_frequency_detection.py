#!/usr/bin/env python3
"""
Live test: wykrywanie częstotliwości LED i wyznaczanie ich współrzędnych.

Użycie:
    python Droniada_tests/live/live_led_frequency_detection.py
    python Droniada_tests/live/live_led_frequency_detection.py --mode advanced \
        --frequencies 22 24 56 98
    python Droniada_tests/live/live_led_frequency_detection.py --without-telemetry
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.gi_camera_handler import CameraPipeline, FPS_SOURCE
from Application.Services.LedFrequencyDetectionService import (
    CANDIDATE_FREQUENCIES_ADVANCED,
    CANDIDATE_FREQUENCIES_BASIC,
    LedFrequencyDetectionService,
)
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.Services.TelemetryCacheService import TelemetryCacheService
from Application.configuration.config_loader import cfg


def print_results(detections) -> None:
    print("\nWyniki:")
    for detection in detections:
        message = (
            f"[LED] {detection.frequency_hz:g} Hz "
            f"pixel={detection.pixel} confidence={detection.confidence:.2f}"
        )
        if detection.coordinates is not None:
            latitude, longitude = detection.coordinates
            message += f" GPS=({latitude:.7f}, {longitude:.7f})"
        print(message)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live LED frequency detection test")
    parser.add_argument(
        "--frequencies",
        type=float,
        nargs=4,
        default=cfg.mission.ook.desired,
        metavar=("F1", "F2", "F3", "F4"),
        help="Four requested LED frequencies",
    )
    parser.add_argument(
        "--mode",
        choices=("basic", "advanced"),
        default="basic",
        help="Candidate frequency range",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=30.0,
        help="Maximum detection duration",
    )
    parser.add_argument("--device", default=None, help="MAVLink device")
    parser.add_argument("--baud", type=int, default=None, help="MAVLink baud rate")
    parser.add_argument(
        "--without-telemetry",
        action="store_true",
        help="Detect frequencies without projecting them to GPS",
    )
    args = parser.parse_args()

    candidates = (
        CANDIDATE_FREQUENCIES_BASIC
        if args.mode == "basic"
        else CANDIDATE_FREQUENCIES_ADVANCED
    )
    service = LedFrequencyDetectionService(
        led_frequencies=args.frequencies,
        candidate_frequencies=candidates,
        fps=FPS_SOURCE,
        camera_resolution=(CameraPipeline.WIDTH, CameraPipeline.HEIGHT),
    )
    camera = CameraPipeline()
    drone = None
    telemetry_cache = None
    mission = None

    try:
        if not args.without_telemetry:
            device = args.device or cfg.mav.device
            baud = args.baud or cfg.mav.baud
            print(f"Łączenie z kontrolerem lotu: {device} @ {baud}")
            drone = MatekService(device=device, baud=baud)
            drone.set_telemetry_rate(10)
            telemetry_cache = TelemetryCacheService(drone, rate_hz=10)
            telemetry_cache.start()
            mission = MissionService(drone)

        print(f"Uruchamiam kamerę ({camera.WIDTH}x{camera.HEIGHT}) @ {FPS_SOURCE} fps...")
        camera.start()
        camera.set_10fps_active(True)
        if not camera.wait_ready():
            raise RuntimeError("Camera pipeline started, but no frame was received")

        print(
            f"Szukam częstotliwości {list(args.frequencies)} "
            f"w trybie {args.mode} przez maksymalnie {args.seconds:g}s..."
        )
        detections = service.run(
            camera,
            telemetry_cache=telemetry_cache,
            mission=mission,
            max_duration_s=args.seconds,
        )
        print_results(detections)
        print(f"Znaleziono {len(detections)}/{len(args.frequencies)} wymaganych LED.")

    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika.")
    finally:
        if telemetry_cache is not None:
            telemetry_cache.stop()
        camera.stop()
        if drone is not None:
            drone.close()


if __name__ == "__main__":
    main()
