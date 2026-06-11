from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional

import cv2
import numpy as np


CANDIDATE_FREQUENCIES_BASIC = tuple(range(2, 21, 2))
CANDIDATE_FREQUENCIES_ADVANCED = tuple(range(21, 101))


@dataclass(frozen=True)
class LedDetection:
    track_id: int
    frequency_hz: float
    pixel: tuple[int, int]
    confidence: float
    timestamp: float
    coordinates: Optional[tuple[float, float]] = None


@dataclass(frozen=True)
class _BrightBlob:
    x: float
    y: float
    area: int


@dataclass
class _LedTrack:
    track_id: int
    roi_x: float
    roi_y: float
    last_timestamp: float
    latest_pixel: tuple[int, int]
    samples: list[tuple[float, float]] = field(default_factory=list)
    observed_area: float = 0.0
    detected_frequency: Optional[float] = None


class LedFrequencyDetectionService:
    """
    Finds four requested OOK LEDs while the drone flies over the search zone.

    Each potential LED gets a square moving ROI. The ROI follows only the
    expected image movement along the flight direction. Wind can move the LED
    inside the ROI without breaking its brightness history.

    Max drone speed for LED detection on 240 FPS in test environment is 15 m/s, 
    for safe detection better use 10-12 m/s. With 200 FPS script detects all
    frequencies, but may be unstable.

    Drone speed is taken from the telemetry (untested), drone_speed_mps is a 
    fallback parameter.

    analysis_duration_s should be balanced with drone speed, for 12 m/s max 
    is 2.5 s (13 m/s - 2.2 s). Increasing analysis duration increases precision, but too big 
    can result in unfinished analysis.
    
    Service can be tested with tests/generate_led_frequency_demo, it simulates
    video from the camera and detects led on it, video is saved in media folder
    """

    def __init__(
        self,
        led_frequencies: Iterable[float],
        candidate_frequencies: Iterable[float] = CANDIDATE_FREQUENCIES_ADVANCED,
        fps: float = 240.0,
        camera_resolution: tuple[int, int] = (640, 400),
        drone_speed_mps: float = 5.0,
        field_width_m: float = 70.0,
        brightness_threshold: int = 220,
        analysis_duration_s: float = 2.0,
        roi_size_m: float = 6.0,
        minimum_site_distance_m: float = 10.0,
        image_motion_direction: int = 1,        # set -1 if camera is turned backwards
        min_blob_area_px: int = 1,
        max_blob_area_px: int = 500,
        min_confidence: float = 4.0,
        duty_cycle_tolerance: float = 0.2,
    ):
        self.led_frequencies = tuple(float(value) for value in led_frequencies)
        self.candidate_frequencies = tuple(float(value) for value in candidate_frequencies)
        self.fps = float(fps)
        self.width, self.height = camera_resolution
        self.drone_speed_mps = float(drone_speed_mps)
        self.field_width_m = float(field_width_m)
        self.brightness_threshold = int(brightness_threshold)
        self.analysis_duration_s = float(analysis_duration_s)
        self.roi_size_m = float(roi_size_m)
        self.minimum_site_distance_m = float(minimum_site_distance_m)
        self.image_motion_direction = 1 if image_motion_direction >= 0 else -1
        self.min_blob_area_px = int(min_blob_area_px)
        self.max_blob_area_px = int(max_blob_area_px)
        self.min_confidence = float(min_confidence)
        self.duty_cycle_tolerance = float(duty_cycle_tolerance)
        self._validate_parameters()

        self.meters_per_pixel = self.field_width_m / self.width
        self.roi_size_px = max(3, int(round(self.roi_size_m / self.meters_per_pixel)))
        self._roi_half_px = self.roi_size_px / 2.0
        self.image_velocity_y_px_s = (
            self.image_motion_direction * self.drone_speed_mps / self.meters_per_pixel
        )
        self._tracks: list[_LedTrack] = []
        self._next_track_id = 0
        self._last_frame_timestamp: Optional[float] = None

    def _validate_parameters(self) -> None:
        # if any(frequency >= self.fps / 2 for frequency in self.candidate_frequencies):
        #     raise ValueError("Camera FPS is too low for the candidate frequencies")
        if self.roi_size_m * math.sqrt(2) >= self.minimum_site_distance_m:
            raise ValueError(
                "The ROI diagonal must be smaller than the minimum distance between sites"
            )

    def reset(self) -> None:
        self._tracks = []
        self._next_track_id = 0
        self._last_frame_timestamp = None

    def _prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        array = np.asarray(frame)
        if array.ndim == 1:
            if array.size != self.width * self.height:
                raise ValueError("Frame size does not match camera_resolution")
            array = array.reshape(self.height, self.width)
        elif array.ndim != 2 or array.shape != (self.height, self.width):
            raise ValueError("Expected a flat or two-dimensional grayscale frame")

        if array.dtype == np.uint16:
            return (array >> 8).astype(np.uint8)
        return array.astype(np.uint8, copy=False)

    def _find_bright_blobs(self, frame: np.ndarray) -> list[_BrightBlob]:
        mask = (frame > self.brightness_threshold).astype(np.uint8)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if self.min_blob_area_px <= area <= self.max_blob_area_px:
                blobs.append(
                    _BrightBlob(
                        x=float(centroids[label][0]),
                        y=float(centroids[label][1]),
                        area=area,
                    )
                )
        return blobs

    def _predict_tracks(
        self, timestamp: float, drone_speed_mps: Optional[float] = None
    ) -> None:
        speed_mps = self.drone_speed_mps if drone_speed_mps is None else drone_speed_mps
        image_velocity_y_px_s = (
            self.image_motion_direction * max(0.0, speed_mps) / self.meters_per_pixel
        )
        for track in self._tracks:
            elapsed = max(0.0, timestamp - track.last_timestamp)
            track.roi_y += image_velocity_y_px_s * elapsed
            track.last_timestamp = timestamp
            track.observed_area = 0.0

    def _inside_roi(self, track: _LedTrack, blob: _BrightBlob) -> bool:
        return (
            abs(blob.x - track.roi_x) <= self._roi_half_px
            and abs(blob.y - track.roi_y) <= self._roi_half_px
        )

    def _associate_blobs(self, blobs: list[_BrightBlob], timestamp: float) -> None:
        unused_blobs = set(range(len(blobs)))

        for track in self._tracks:
            matches = [
                index for index in unused_blobs if self._inside_roi(track, blobs[index])
            ]
            if not matches:
                continue

            best_index = min(
                matches,
                key=lambda index: math.hypot(
                    blobs[index].x - track.latest_pixel[0],
                    blobs[index].y - track.latest_pixel[1],
                ),
            )
            blob = blobs[best_index]
            track.latest_pixel = (int(round(blob.x)), int(round(blob.y)))
            track.observed_area = float(blob.area)
            unused_blobs.remove(best_index)

        for index in unused_blobs:
            blob = blobs[index]
            if any(self._inside_roi(track, blob) for track in self._tracks):
                continue
            pixel = (int(round(blob.x)), int(round(blob.y)))
            self._tracks.append(
                _LedTrack(
                    track_id=self._next_track_id,
                    roi_x=blob.x,
                    roi_y=blob.y,
                    last_timestamp=timestamp,
                    latest_pixel=pixel,
                    observed_area=float(blob.area),
                )
            )
            self._next_track_id += 1

    def _estimate_frequency(
        self, samples: list[tuple[float, float]]
    ) -> Optional[tuple[float, float]]:
        timestamps = np.asarray([sample[0] for sample in samples], dtype=np.float64)
        brightness = np.asarray([sample[1] for sample in samples], dtype=np.float64)
        signal = brightness - np.mean(brightness)
        if float(np.dot(signal, signal)) <= 1e-9:
            return None

        duty_cycle = float(np.mean(brightness > 0))
        if abs(duty_cycle - 0.5) > self.duty_cycle_tolerance:
            return None

        relative_time = timestamps - timestamps[0]
        frequencies = np.asarray(self.candidate_frequencies, dtype=np.float64)
        phase = np.exp(-1j * 2.0 * np.pi * np.outer(frequencies, relative_time))
        projection = phase @ signal
        powers = projection.real ** 2 + projection.imag ** 2

        order = np.argsort(powers)[::-1]
        best_index = int(order[0])
        second_power = float(powers[order[1]])
        confidence = (
            float(powers[best_index]) / second_power
            if second_power > 1e-12
            else float("inf")
        )
        if confidence < self.min_confidence:
            return None
        return float(frequencies[best_index]), confidence

    def process_frame(
        self,
        frame: np.ndarray,
        timestamp: float,
        drone_speed_mps: Optional[float] = None,
    ) -> list[LedDetection]:
        if self._last_frame_timestamp is not None and timestamp <= self._last_frame_timestamp:
            return []
        self._last_frame_timestamp = timestamp

        gray = self._prepare_frame(frame)
        self._predict_tracks(timestamp, drone_speed_mps)
        self._associate_blobs(self._find_bright_blobs(gray), timestamp)

        detections = []
        active_tracks = []
        oldest_allowed = timestamp - self.analysis_duration_s

        for track in self._tracks:
            if -self.roi_size_px <= track.roi_y <= self.height + self.roi_size_px:
                active_tracks.append(track)

            track.samples.append((timestamp, track.observed_area))
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
            if frequency not in self.led_frequencies:
                continue
            detections.append(
                LedDetection(
                    track_id=track.track_id,
                    frequency_hz=frequency,
                    pixel=track.latest_pixel,
                    confidence=confidence,
                    timestamp=timestamp,
                )
            )

        self._tracks = active_tracks
        return detections

    def project_detection(self, detection: LedDetection, telemetry_cache, mission) -> LedDetection:
        snapshot = telemetry_cache.get_latest()
        if snapshot is None:
            return detection

        latitude, longitude, altitude = snapshot.coordinates
        roll, pitch, yaw = snapshot.attitude
        mission_width = getattr(mission, "image_width", self.width)
        mission_height = getattr(mission, "image_height", self.height)
        pixel = (
            int(detection.pixel[0] * mission_width / self.width),
            int(detection.pixel[1] * mission_height / self.height),
        )
        target = mission.project_target_cords(
            pixel,
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
        telemetry_cache=None,
        mission=None,
        stop_event=None,
        max_duration_s: Optional[float] = None,
    ) -> list[LedDetection]:
        self.reset()
        camera.set_120fps_active(True)
        best_detections: dict[float, LedDetection] = {}
        start_time = time.monotonic()

        while len(best_detections) < len(self.led_frequencies):
            if stop_event is not None and stop_event.is_set():
                break
            if max_duration_s is not None and time.monotonic() - start_time >= max_duration_s:
                break

            frame, timestamp, error = camera.get_image()
            if error is not None or frame is None or timestamp is None:
                time.sleep(0.5 / self.fps)
                continue

            drone_speed_mps = None
            if telemetry_cache is not None:
                snapshot = telemetry_cache.get_latest()
                if snapshot is not None:
                    drone_speed_mps = snapshot.ground_speed_mps

            for detection in self.process_frame(frame, timestamp, drone_speed_mps):
                if telemetry_cache is not None and mission is not None:
                    detection = self.project_detection(detection, telemetry_cache, mission)
                    if detection.coordinates is None:
                        self._allow_detection_retry(detection.track_id)
                        continue
                previous = best_detections.get(detection.frequency_hz)
                if previous is None or detection.confidence > previous.confidence:
                    best_detections[detection.frequency_hz] = detection

            time.sleep(0.5 / self.fps)

        return [
            best_detections[frequency]
            for frequency in self.led_frequencies
            if frequency in best_detections
        ]
