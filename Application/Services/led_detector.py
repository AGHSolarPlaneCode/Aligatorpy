from __future__ import annotations

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None
    np = None


class LedDetector:
    THRESHOLD_VALUE = 210
    MIN_AREA = 3
    MAX_AREA = 200
    MERGE_RADIUS = 25

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self._detected_targets: list[dict] = []
        self._target_id_counter = 0

    def reset(self) -> None:
        self._detected_targets = []
        self._target_id_counter = 0

    def _to_gray(self, frame) -> "np.ndarray":
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/numpy are required for LED detection")

        if frame.ndim == 1:
            return frame.reshape(self.height, self.width)
        if frame.ndim == 3:
            return frame[: self.height, : self.width, 0]  # weź tylko pierwszy kanał, są identyczne
        return frame[: self.height, : self.width]

    def process_frame(self, frame) -> list[dict]:
        gray = self._to_gray(frame)

        _, thresh = cv2.threshold(gray, self.THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_frame_centroids = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.MIN_AREA < area < self.MAX_AREA:
                m = cv2.moments(cnt)
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    current_frame_centroids.append((cx, cy))

        for target in self._detected_targets:
            target["frames_unseen"] += 1

        for cx, cy in current_frame_centroids:
            matched = False
            for target in self._detected_targets:
                dist = ((target["x"] - cx) ** 2 + (target["y"] - cy) ** 2) ** 0.5
                if dist < self.MERGE_RADIUS:
                    target["x"] = cx
                    target["y"] = cy
                    target["frames_unseen"] = 0
                    matched = True
                    break

            if not matched:
                self._detected_targets.append(
                    {
                        "id": self._target_id_counter,
                        "x": cx,
                        "y": cy,
                        "frames_unseen": 0,
                    }
                )
                self._target_id_counter += 1

        return [t.copy() for t in self._detected_targets]
