import argparse
import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np


SERVICE_PATH = (
    Path(__file__).resolve().parents[1]
    / "Application"
    / "Services"
    / "LedFrequencyDetectionService.py"
)
SPEC = importlib.util.spec_from_file_location("led_frequency_detection_demo_service", SERVICE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
LedFrequencyDetectionService = MODULE.LedFrequencyDetectionService

DEFAULT_OUTPUT_DIR = (
    Path(__file__).resolve().parents[1]
    / "Application"
    / "media"
    / "videos"
    / "led_frequency_demo"
)

def generate_demo(
    output_dir: Path,
    fps: int = 200,
    duration_s: float = 6.0,
    wind: bool = True,
    noise: bool = True,
) -> Path:
    resolution = (640, 400)
    width, height = resolution
    requested = [100.0, 99.0, 97.0, 98.0]
    service = LedFrequencyDetectionService(
        led_frequencies=requested,
        candidate_frequencies=MODULE.CANDIDATE_FREQUENCIES_ADVANCED,
        fps=fps,
        camera_resolution=resolution,
        drone_speed_mps=13.0,
        analysis_duration_s=2.2
    )
    leds = [
        (100, 20, requested[0]),
        (240, 70, requested[1]),
        (380, 120, requested[2]),
        (520, 170, requested[3]),
    ]

    output_dir.mkdir(parents=True, exist_ok=True)
    video_path = output_dir / "led_frequency_demo.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        resolution,
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not create video file: {video_path}")

    rng = np.random.default_rng(seed=42)
    detections_by_track = {}
    for frame_index in range(int(fps * duration_s)):
        timestamp = frame_index / fps
        gray = np.full((height, width), 25, dtype=np.uint8)
        if noise:
            noisy = gray.astype(np.int16)
            noisy += rng.normal(0, 5, gray.shape).astype(np.int16)
            gray = np.clip(noisy, 0, 255).astype(np.uint8)

        wind_px = 14 * np.sin(2 * np.pi * timestamp / 2.2) if wind else 0.0
        for x, initial_y, frequency in leds:
            led_x = int(round(x + wind_px))
            led_y = int(round(initial_y + service.image_velocity_y_px_s * timestamp))
            if (
                0 <= led_x < width
                and 0 <= led_y < height
                and (timestamp * frequency) % 1.0 < 0.5
            ):
                gray[led_y, led_x] = 255

        if noise and rng.random() < 0.15:
            gray[int(rng.integers(height)), int(rng.integers(width))] = 255

        for detection in service.process_frame(gray.astype(np.uint16).reshape(-1) << 8, timestamp):
            detections_by_track[detection.track_id] = detection

        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        half = service.roi_size_px // 2
        for track in service._tracks:
            center = (int(round(track.roi_x)), int(round(track.roi_y)))
            detection = detections_by_track.get(track.track_id)
            color = (0, 255, 0) if detection is not None else (0, 180, 255)
            cv2.rectangle(
                display,
                (center[0] - half, center[1] - half),
                (center[0] + half, center[1] + half),
                color,
                1,
            )
            if detection is not None:
                cv2.putText(
                    display,
                    f"{detection.frequency_hz:g}Hz conf={detection.confidence:.1f}",
                    (center[0] - half, max(20, center[1] - half - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        cv2.putText(
            display,
            f"time={timestamp:.2f}s requested={requested} detected={len(detections_by_track)}/4",
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        writer.write(display)

    writer.release()
    print(
        "Detected requested frequencies:",
        sorted({item.frequency_hz for item in detections_by_track.values()}),
    )
    return video_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate moving ROI/OOK LED demo")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--wind", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--noise", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    result = generate_demo(
        args.output_dir,
        duration_s=args.duration,
        wind=args.wind,
        noise=args.noise,
    )
    print(f"Generated video: {result}")
