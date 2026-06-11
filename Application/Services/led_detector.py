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
    MAX_AREA = 150
    MERGE_RADIUS = 25

    # Przycięcie krawędzi (dystorsja fisheye) — tylko w analizie Python, nie w kamerze.
    CROP_HORIZONTAL = 160  # px z lewej i prawej strony
    CROP_VERTICAL = 75     # px z góry i dołu

    def __init__(
        self,
        width: int,
        height: int,
        crop_horizontal: int | None = None,
        crop_vertical: int | None = None,
    ):
        self.width = width
        self.height = height
        self.crop_horizontal = (
            self.CROP_HORIZONTAL if crop_horizontal is None else crop_horizontal
        )
        self.crop_vertical = (
            self.CROP_VERTICAL if crop_vertical is None else crop_vertical
        )
        self._detected_targets: list[dict] = []
        self._target_id_counter = 0

    def reset(self) -> None:
        self._detected_targets = []
        self._target_id_counter = 0

    def _to_gray(self, frame) -> "np.ndarray":
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/numpy are required for LED detection")

        if frame.ndim == 1:
            gray = frame.reshape(self.height, self.width)
        elif frame.ndim == 3:
            gray = frame[: self.height, : self.width, 0]
        else:
            gray = frame[: self.height, : self.width]

        # OV9281 daje 16-bit, findContours wymaga 8-bit
        if gray.dtype != np.uint8:
            gray = (gray >> 8).astype(np.uint8)  # przesunięcie bitowe 16→8

        return gray

    def _crop_gray(self, gray: "np.ndarray") -> tuple["np.ndarray", int, int]:
        """Obetnij zniekształcone brzegi; zwróć (obraz, offset_x, offset_y) w pełnej klatce."""
        ch = self.crop_horizontal
        cv = self.crop_vertical
        if 2 * ch >= self.width or 2 * cv >= self.height:
            return gray, 0, 0
        return gray[cv : self.height - cv, ch : self.width - ch], ch, cv

    def process_frame(self, frame) -> list[dict]:
        gray = self._to_gray(frame)
        gray, offset_x, offset_y = self._crop_gray(gray)

        _, thresh = cv2.threshold(gray, self.THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_frame_centroids = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.MIN_AREA < area < self.MAX_AREA:
                m = cv2.moments(cnt)
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"]) + offset_x
                    cy = int(m["m01"] / m["m00"]) + offset_y
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
