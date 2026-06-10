from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from Application.Services.MatekService import MatekService


@dataclass(frozen=True)
class TelemetrySnapshot:
    coordinates: tuple[float, float, float]
    attitude: tuple[float, float, float]
    ground_speed_mps: float
    timestamp: float


class TelemetryCacheService:
    """Reads GPS and attitude in one background task and exposes the latest values."""

    def __init__(self, drone: MatekService, rate_hz: float = 10.0):
        self.drone = drone
        self.rate_hz = float(rate_hz)
        self._snapshot: Optional[TelemetrySnapshot] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="telemetry-cache",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def get_latest(self) -> Optional[TelemetrySnapshot]:
        with self._lock:
            return self._snapshot

    def _run(self) -> None:
        interval = 1.0 / self.rate_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            position_data = self.drone.get_current_position_data(timeout=0.5)
            attitude = self.drone.get_attitude(timeout=0.5)
            if position_data is not None and attitude is not None:
                coordinates, ground_speed_mps = position_data
                snapshot = TelemetrySnapshot(
                    coordinates=coordinates,
                    attitude=attitude,
                    ground_speed_mps=ground_speed_mps,
                    timestamp=time.monotonic(),
                )
                with self._lock:
                    self._snapshot = snapshot

            remaining = interval - (time.monotonic() - started)
            if remaining > 0:
                self._stop_event.wait(remaining)
