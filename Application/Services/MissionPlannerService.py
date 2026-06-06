from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from Application.Logger.log_module import get_logger
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import LoiterConfig


class MissionPlannerService:
    def __init__(self):
        self.logger = get_logger(__name__)

    @staticmethod
    def _distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        m_per_lat, m_per_lon = MissionService.get_meters_per_degree(lat1)
        dy = (lat1 - lat2) * m_per_lat
        dx = (lon1 - lon2) * m_per_lon
        return math.sqrt(dx * dx + dy * dy)

    def order_targets_nearest(
        self,
        start_lat: float,
        start_lon: float,
        targets: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        remaining = list(targets)
        ordered: List[Dict[str, Any]] = []
        cur_lat, cur_lon = start_lat, start_lon

        while remaining:
            best = min(
                remaining,
                key=lambda t: self._distance_meters(cur_lat, cur_lon, t["lat"], t["lon"]),
            )
            ordered.append(best)
            remaining.remove(best)
            cur_lat, cur_lon = best["lat"], best["lon"]

        self.logger.info(f"Ordered {len(ordered)} targets (nearest-neighbor)")
        return ordered

    @staticmethod
    def build_loiter_waypoints(
        ordered_targets: List[Dict[str, Any]],
        loiter_cfg: LoiterConfig,
    ) -> List[Dict[str, Any]]:
        waypoints = []
        for target in ordered_targets:
            waypoints.append(
                {
                    "command": "NAV_LOITER_TIME",
                    "lat": target["lat"],
                    "lon": target["lon"],
                    "alt": loiter_cfg.alt,
                    "time": loiter_cfg.time,
                    "radius": loiter_cfg.radius,
                }
            )
        return waypoints

    @staticmethod
    def build_landing_sites(
        ordered_targets: List[Dict[str, Any]],
        ook_results: List[Dict[str, Any]],
        min_confidence: float,
    ) -> List[Tuple[float, float]]:
        sites = []
        for target, ook in zip(ordered_targets, ook_results):
            if ook.get("freq") is not None and ook.get("confidence", 0) >= min_confidence:
                sites.append((target["lat"], target["lon"]))
        return sites
