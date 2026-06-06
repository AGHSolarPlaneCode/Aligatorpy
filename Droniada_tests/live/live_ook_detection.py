#!/usr/bin/env python3
"""
Live test: wykrywanie modulacji OOK @ 120fps przez 10s (domyślnie z config).

Użycie:
    python Droniada_tests/live/live_ook_detection.py
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from Application.Services.GiCameraService import GiCameraService
from Application.Services.OokDetectionService import OokDetectionService
from Application.configuration.config_loader import cfg


def main():
    parser = argparse.ArgumentParser(description="Live OOK detection test")
    parser.add_argument("--duration", type=float, default=None, help="Override duration_s from config")
    args = parser.parse_args()

    camera = GiCameraService()
    print("Uruchamiam kamerę...")
    camera.start()

    ook_cfg = cfg.mission.ook
    if args.duration is not None:
        from dataclasses import replace
        ook_cfg = replace(ook_cfg, duration_s=args.duration)

    service = OokDetectionService(camera, ook_cfg)
    print(f"Rozpoczynam OOK przez {ook_cfg.duration_s}s @ 120fps...")

    try:
        result = service.detect_modulation()
        print(f"Wynik OOK: freq={result['freq']}Hz, confidence={result['confidence']:.2f}, samples={result['samples']}")
        if result["freq"] is not None and result["confidence"] >= ook_cfg.min_confidence:
            print(f"[OK] Modulacja potwierdzona: {result['freq']} Hz")
        else:
            print("[WARN] Brak wyraźnej modulacji OOK")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
