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
    fps: int = 60,
    duration_s: float = 5.0,
    resolution: tuple[int, int] = (1280, 800),
    field_width_m: float = 70.0,
    drone_speed_mps: float = 5.0,
    analysis_square_size_m: float = 6.0,
) -> Path:
    width, height = resolution
    expected_frequencies = [2.0, 6.0, 10.0, 14.0]
    service = LedFrequencyDetectionService(
        led_frequencies=expected_frequencies,
        fps=fps,
        camera_resolution=resolution,
        drone_speed_mps=drone_speed_mps,
        field_width_m=field_width_m,
        brightness_threshold=128,
        analysis_duration_s=2.0,
        analysis_square_size_m=analysis_square_size_m,
        min_blob_area_px=1,
        max_blob_area_px=25,
    )

    leds = [
        (180, 80, 2.0),
        (460, -45, 6.0),
        (760, 280, 10.0),
        (1040, -120, 14.0),
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
    snapshot_indices = {
        0,
        int(fps * 0.5),
        int(fps * 1.0),
        int(fps * 2.0),
        int(fps * 3.0),
        int(fps * 4.0),
    }
    detections_by_track = {}

    for frame_index in range(int(fps * duration_s)):
        timestamp = frame_index / fps
        gray = rng.normal(loc=25, scale=4, size=(height, width))
        gray = np.clip(gray, 0, 255).astype(np.uint8)

        for x, initial_y, frequency in leds:
            y = int(round(initial_y + service.image_velocity_y_px_s * timestamp))
            is_on = (timestamp * frequency) % 1.0 < 0.5
            if 0 <= y < height:
                if is_on:
                    gray[max(0, y - 2) : min(height, y + 3), x - 2 : x + 3] = 220

        new_detections = service.process_frame(gray.reshape(-1), timestamp)
        for detection in new_detections:
            detections_by_track[detection.track_id] = detection

        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        half_square = service.analysis_square_px // 2
        for track in service._tracks:
            center_x = int(round(track.x))
            center_y = int(round(track.y))
            x0 = max(0, center_x - half_square)
            x1 = min(width - 1, center_x + half_square)
            y0 = max(0, center_y - half_square)
            y1 = min(height - 1, center_y + half_square)

            detection = detections_by_track.get(track.track_id)
            if detection is None:
                color = (0, 200, 255)
                label = f"track {track.track_id}: pending"
            else:
                color = (0, 255, 0)
                label = (
                    f"track {track.track_id}: {detection.frequency_hz:g} Hz "
                    f"score={detection.confidence:.2f}"
                )

            cv2.rectangle(display, (x0, y0), (x1, y1), color, 2)
            cv2.putText(
                display,
                label,
                (x0, max(20, y0 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            display,
            (
                f"time={timestamp:.3f}s  active_tracks={len(service._tracks)}  "
                f"detected={len(detections_by_track)}/{len(expected_frequencies)}"
            ),
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        writer.write(display)

        if frame_index in snapshot_indices:
            cv2.imwrite(str(output_dir / f"frame_{frame_index:04d}.png"), display)

    writer.release()
    detected_frequencies = sorted(
        detection.frequency_hz for detection in detections_by_track.values()
    )
    print(f"Service detected frequencies: {detected_frequencies}")
    return video_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run LedFrequencyDetectionService on synthetic frames and save a demo video."
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    result = generate_demo(args.output_dir, duration_s=args.duration)
    print(f"Generated video: {result}")
