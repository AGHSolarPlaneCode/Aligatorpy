from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional

import numpy as np

logger = logging.getLogger(__name__)
REQUIRED_OPENCV_MAJOR_VERSION = 4

try:
    import cv2
except ImportError as exc:
    logger.error(
        "OpenCV is required by LedFrequencyDetectionService. "
        "Install dependencies from Raspberry_configuration/config_files/requirements.txt."
    )
    raise ImportError("LedFrequencyDetectionService requires the cv2 module") from exc

opencv_major_version = int(cv2.__version__.split(".", maxsplit=1)[0])
if opencv_major_version != REQUIRED_OPENCV_MAJOR_VERSION:
    logger.error(
        "Unsupported OpenCV version %s; expected OpenCV %s.x",
        cv2.__version__,
        REQUIRED_OPENCV_MAJOR_VERSION,
    )
    raise RuntimeError(
        f"Unsupported OpenCV version {cv2.__version__}; "
        f"expected OpenCV {REQUIRED_OPENCV_MAJOR_VERSION}.x"
    )


@dataclass(frozen=True)
class LedDetection:
    track_id: int
    frequency_hz: float
    pixel: tuple[int, int]
    confidence: float
    timestamp: float
    coordinates: Optional[tuple[float, float]] = None


@dataclass
class _LedTrack:
    track_id: int
    x: float
    y: float
    last_timestamp: float
    samples: list[tuple[float, float]] = field(default_factory=list)
    detected_frequency: Optional[float] = None


class LedFrequencyDetectionService:
    """
    Detects moving, blinking LEDs in monochrome camera frames.

    First-version assumptions:
    - the camera points vertically down;
    - the drone flies at constant speed along the image Y axis;
    - landing sites are at least ``minimum_site_distance_m`` apart;
    - one LED represents one landing site.
    """

    def __init__(
        self,
        led_frequencies: Iterable[float],
        fps: float = 60.0,
        camera_resolution: tuple[int, int] = (1280, 800),
        drone_speed_mps: float = 5.0,
        drone_height_m: float = 60.0,
        field_width_m: float = 70.0,
        brightness_threshold: int = 128,
        analysis_duration_s: float = 2.0,
        frequency_tolerance_hz: float = 0.5,
        minimum_site_distance_m: float = 10.0,
        analysis_square_size_m: float = 6.0,
        image_motion_direction: int = 1,
        min_blob_area_px: int = 1,
        max_blob_area_px: int = 25,
        min_frequency_score: float = 0.12,
    ):
        self.led_frequencies = tuple(float(value) for value in led_frequencies)
        self.fps = float(fps)
        self.width, self.height = camera_resolution
        self.drone_speed_mps = float(drone_speed_mps)
        self.drone_height_m = float(drone_height_m)
        self.field_width_m = float(field_width_m)
        self.brightness_threshold = int(brightness_threshold)
        self.analysis_duration_s = float(analysis_duration_s)
        self.frequency_tolerance_hz = float(frequency_tolerance_hz)
        self.minimum_site_distance_m = float(minimum_site_distance_m)
        self.analysis_square_size_m = float(analysis_square_size_m)
        self.image_motion_direction = 1 if image_motion_direction >= 0 else -1
        self.min_blob_area_px = int(min_blob_area_px)
        self.max_blob_area_px = int(max_blob_area_px)
        self.min_frequency_score = float(min_frequency_score)
        self.logger = logging.getLogger(__name__)

        self._validate_parameters()

        self.meters_per_pixel = self.field_width_m / self.width
        self.analysis_square_px = max(
            3, int(round(self.analysis_square_size_m / self.meters_per_pixel))
        )
        self.image_velocity_y_px_s = (
            self.image_motion_direction
            * self.drone_speed_mps
            / self.meters_per_pixel
        )
        self._association_radius_px = self.analysis_square_px / 2.0
        self._tracks: list[_LedTrack] = []
        self._next_track_id = 0
        self._last_frame_timestamp: Optional[float] = None

    def _validate_parameters(self) -> None:
        if not self.led_frequencies:
            raise ValueError("led_frequencies cannot be empty")
        if any(frequency <= 0 or frequency >= self.fps / 2 for frequency in self.led_frequencies):
            raise ValueError("Each LED frequency must be between 0 and the Nyquist frequency")
        if self.width <= 0 or self.height <= 0 or self.field_width_m <= 0:
            raise ValueError("Camera resolution and field width must be positive")
        if not 0 <= self.brightness_threshold <= 255:
            raise ValueError("brightness_threshold must be between 0 and 255")
        if self.analysis_duration_s <= 0:
            raise ValueError("analysis_duration_s must be positive")
        if self.analysis_square_size_m * math.sqrt(2) >= self.minimum_site_distance_m:
            raise ValueError(
                "The analysis square diagonal must be smaller than the minimum "
                "distance between landing sites"
            )

    def reset(self) -> None:
        self._tracks = []
        self._next_track_id = 0
        self._last_frame_timestamp = None

    def _prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        array = np.asarray(frame)
        expected_size = self.width * self.height
        if array.ndim != 1 or array.size != expected_size:
            raise ValueError(
                f"Expected a flat GRAY8 frame with {expected_size} pixels, "
                f"received shape {array.shape}"
            )
        return array.reshape(self.height, self.width)

    def _find_bright_points(self, frame: np.ndarray) -> list[tuple[float, float]]:
        mask = (frame >= self.brightness_threshold).astype(np.uint8)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        points = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if self.min_blob_area_px <= area <= self.max_blob_area_px:
                points.append((float(centroids[label][0]), float(centroids[label][1])))
        return points

    def _predict_tracks(self, timestamp: float) -> None:
        for track in self._tracks:
            elapsed = max(0.0, timestamp - track.last_timestamp)
            track.y += self.image_velocity_y_px_s * elapsed
            track.last_timestamp = timestamp

    def _associate_points(self, points: list[tuple[float, float]], timestamp: float) -> None:
        unused_points = set(range(len(points)))

        for track in self._tracks:
            best_index = None
            best_distance = self._association_radius_px
            for index in unused_points:
                x, y = points[index]
                distance = math.hypot(x - track.x, y - track.y)
                if distance < best_distance:
                    best_index = index
                    best_distance = distance

            if best_index is not None:
                track.x, track.y = points[best_index]
                unused_points.remove(best_index)

        for index in unused_points:
            x, y = points[index]
            self._tracks.append(
                _LedTrack(
                    track_id=self._next_track_id,
                    x=x,
                    y=y,
                    last_timestamp=timestamp,
                )
            )
            self._next_track_id += 1

    def _sample_track_square(self, frame: np.ndarray, track: _LedTrack) -> float:
        half = self.analysis_square_px // 2
        center_x = int(round(track.x))
        center_y = int(round(track.y))
        x0 = max(0, center_x - half)
        x1 = min(self.width, center_x + half + 1)
        y0 = max(0, center_y - half)
        y1 = min(self.height, center_y + half + 1)
        if x0 >= x1 or y0 >= y1:
            return 0.0
        return float(np.max(frame[y0:y1, x0:x1]))

    def _estimate_frequency(
        self, samples: list[tuple[float, float]]
    ) -> Optional[tuple[float, float]]:
        timestamps = np.asarray([sample[0] for sample in samples], dtype=np.float64)
        brightness = np.asarray([sample[1] for sample in samples], dtype=np.float64)
        binary_signal = (brightness >= self.brightness_threshold).astype(np.float64)
        centered_signal = binary_signal - np.mean(binary_signal)

        if np.max(binary_signal) == np.min(binary_signal):
            return None

        rising_edges = np.flatnonzero(np.diff(binary_signal) > 0.5) + 1
        if len(rising_edges) < 2:
            return None

        edge_duration = timestamps[rising_edges[-1]] - timestamps[rising_edges[0]]
        if edge_duration <= 0:
            return None
        measured_frequency = (len(rising_edges) - 1) / float(edge_duration)
        expected_frequency = min(
            self.led_frequencies,
            key=lambda frequency: abs(frequency - measured_frequency),
        )
        if abs(expected_frequency - measured_frequency) > self.frequency_tolerance_hz:
            return None

        relative_time = timestamps - timestamps[0]
        wave = np.exp(-2j * np.pi * expected_frequency * relative_time)
        score = float(abs(np.mean(centered_signal * wave)))
        if score < self.min_frequency_score:
            return None
        return expected_frequency, score

    def process_frame(self, frame: np.ndarray, timestamp: float) -> list[LedDetection]:
        """
        Process one frame and return newly recognized target frequencies.

        Repeated or out-of-order timestamps are ignored, which prevents repeated
        calls to ``CameraPipeline.get_image()`` from analyzing the same frame.
        """
        if self._last_frame_timestamp is not None and timestamp <= self._last_frame_timestamp:
            return []
        self._last_frame_timestamp = timestamp

        gray = self._prepare_frame(frame)
        self._predict_tracks(timestamp)
        self._associate_points(self._find_bright_points(gray), timestamp)

        detections = []
        active_tracks = []
        for track in self._tracks:
            if -self.analysis_square_px <= track.y <= self.height + self.analysis_square_px:
                active_tracks.append(track)

            brightness = self._sample_track_square(gray, track)
            track.samples.append((timestamp, brightness))

            oldest_allowed = timestamp - self.analysis_duration_s
            while track.samples and track.samples[0][0] < oldest_allowed:
                track.samples.pop(0)

            if track.detected_frequency is not None or len(track.samples) < 2:
                continue
            duration = track.samples[-1][0] - track.samples[0][0]
            if duration < self.analysis_duration_s * 0.95:
                continue

            estimate = self._estimate_frequency(track.samples)
            if estimate is None:
                continue

            frequency, confidence = estimate
            track.detected_frequency = frequency
            detections.append(
                LedDetection(
                    track_id=track.track_id,
                    frequency_hz=frequency,
                    pixel=(int(round(track.x)), int(round(track.y))),
                    confidence=confidence,
                    timestamp=timestamp,
                )
            )

        self._tracks = active_tracks
        return detections

    def project_detection(self, detection: LedDetection, drone, mission) -> LedDetection:
        """
        Project a detection to GPS using the existing MatekService/MissionService.

        Drone position and attitude come from current MAVLink telemetry.
        """
        coordinates = drone.get_current_coordinates()
        attitude = drone.get_attitude()
        if coordinates is None or attitude is None:
            return detection

        latitude, longitude, altitude = coordinates
        roll, pitch, yaw = attitude
        target = mission.project_target_cords(
            detection.pixel,
            latitude,
            longitude,
            altitude,
            roll,
            pitch,
            yaw,
        )
        if target is None:
            return detection

        return LedDetection(
            track_id=detection.track_id,
            frequency_hz=detection.frequency_hz,
            pixel=detection.pixel,
            confidence=detection.confidence,
            timestamp=detection.timestamp,
            coordinates=target,
        )

    def _allow_detection_retry(self, track_id: int) -> None:
        for track in self._tracks:
            if track.track_id == track_id:
                track.detected_frequency = None
                return

    def run(
        self,
        camera,
        drone=None,
        mission=None,
        stop_event=None,
        max_duration_s: Optional[float] = None,
    ) -> list[LedDetection]:
        """Read frames from ``gi_camera_handler.CameraPipeline`` until all targets are found."""
        self.reset()
        camera.set_120fps_active(True)
        detections: list[LedDetection] = []
        detected_frequencies: set[float] = set()
        start_time = time.monotonic()

        while len(detections) < len(self.led_frequencies):
            if stop_event is not None and stop_event.is_set():
                break
            if max_duration_s is not None and time.monotonic() - start_time >= max_duration_s:
                break

            frame, timestamp, error = camera.get_image()
            if error is not None or frame is None or timestamp is None:
                time.sleep(0.001)
                continue

            new_detections = self.process_frame(frame, timestamp)
            for detection in new_detections:
                if detection.frequency_hz in detected_frequencies:
                    continue
                if drone is not None and mission is not None:
                    detection = self.project_detection(detection, drone, mission)
                    if detection.coordinates is None:
                        self._allow_detection_retry(detection.track_id)
                        continue
                detections.append(detection)
                detected_frequencies.add(detection.frequency_hz)

            time.sleep(0.0005)

        return detections
