from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from pymavlink import mavutil

from Application.Logger.log_module import get_logger
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import LoiterConfig

_LOITER_TIME_CMD = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME


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
    def build_approach_and_loiter_waypoints(
        ordered_targets: List[Dict[str, Any]],
        loiter_cfg: LoiterConfig,
    ) -> List[Dict[str, Any]]:
        """Para WAYPOINT (dojazd) + NAV_LOITER_TIME (zawis) na każde lądowisko."""
        waypoints: List[Dict[str, Any]] = []
        for target in ordered_targets:
            waypoints.append(
                {
                    "command": "WAYPOINT",
                    "lat": target["lat"],
                    "lon": target["lon"],
                    "alt": loiter_cfg.alt,
                    "acr": 0,
                }
            )
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
    def loiter_wp_indices(first_approach_wp: int, site_count: int) -> List[int]:
        """Indeksy LOITER w misji z parami WAYPOINT+LOITER (LOITER co drugi wp)."""
        return [first_approach_wp + 2 * i + 1 for i in range(site_count)]

    @staticmethod
    def extract_loiter_sites_from_mission(
        mission_items: List[Dict[str, Any]],
        loiter_wp_indices: List[int] | None = None,
    ) -> Tuple[List[int], List[Dict[str, Any]]]:
        """
        Wyciąga indeksy i współrzędne NAV_LOITER_TIME z misji FC.

        Gdy podano loiter_wp_indices, bierze tylko te seq (współrz. z misji).
        W przeciwnym razie wszystkie LOITER_TIME w kolejności seq.
        """
        by_seq = {item["seq"]: item for item in mission_items}

        if loiter_wp_indices:
            indices: List[int] = []
            targets: List[Dict[str, Any]] = []
            for seq in loiter_wp_indices:
                item = by_seq.get(seq)
                if item is None:
                    raise ValueError(f"Waypoint seq {seq} not found in FC mission")
                if item["command"] != _LOITER_TIME_CMD:
                    raise ValueError(
                        f"Waypoint seq {seq} is not NAV_LOITER_TIME (cmd={item['command']})"
                    )
                indices.append(seq)
                targets.append(
                    {
                        "lat": item["param5"] / 1e7,
                        "lon": item["param6"] / 1e7,
                    }
                )
            return indices, targets

        indices = []
        targets = []
        for item in sorted(mission_items, key=lambda wp: wp["seq"]):
            if item["command"] != _LOITER_TIME_CMD:
                continue
            indices.append(item["seq"])
            targets.append(
                {
                    "lat": item["param5"] / 1e7,
                    "lon": item["param6"] / 1e7,
                }
            )
        return indices, targets

    @staticmethod
    def _freq_matches(detected: float, desired: float, tolerance: float = 0.01) -> bool:
        return abs(float(detected) - float(desired)) <= tolerance

    def select_desired_landing_sites(
        self,
        ordered_targets: List[Dict[str, Any]],
        ook_results: List[Dict[str, Any]],
        desired: tuple[float, ...],
        min_confidence: float,
    ) -> List[Tuple[float, float]]:
        """
        Wybiera lądowiska do wysłania na samolot — po jednym (lub więcej przy
        powtórzonej częstotliwości w desired) na każdy wpis z listy desired.

        Dla każdej częstotliwości z desired bierze nieużyte jeszcze obserwacje
        OOK o pasującym freq i najwyższym confidence (>= min_confidence).
        """
        observations: List[Dict[str, Any]] = []
        for target, ook in zip(ordered_targets, ook_results):
            freq = ook.get("freq")
            confidence = ook.get("confidence", 0)
            if freq is None or confidence < min_confidence:
                continue
            observations.append(
                {
                    "lat": target["lat"],
                    "lon": target["lon"],
                    "freq": float(freq),
                    "confidence": float(confidence),
                }
            )

        used: set[int] = set()
        selected: List[Tuple[float, float]] = []

        for desired_freq in desired:
            matches = [
                (idx, obs)
                for idx, obs in enumerate(observations)
                if idx not in used and self._freq_matches(obs["freq"], desired_freq)
            ]
            if not matches:
                self.logger.warning(
                    f"No OOK match for desired landing freq {desired_freq}Hz "
                    f"(min_confidence={min_confidence})"
                )
                continue

            matches.sort(key=lambda item: item[1]["confidence"], reverse=True)
            best_idx, best = matches[0]
            used.add(best_idx)
            selected.append((best["lat"], best["lon"]))
            self.logger.info(
                f"Selected landing site for desired {desired_freq}Hz: "
                f"({best['lat']:.6f}, {best['lon']:.6f}), "
                f"detected={best['freq']}Hz, confidence={best['confidence']:.2f}"
            )

        self.logger.info(
            f"Selected {len(selected)}/{len(desired)} desired landing site(s)"
        )
        return selected

    @staticmethod
    def build_landing_sites(
        ordered_targets: List[Dict[str, Any]],
        ook_results: List[Dict[str, Any]],
        min_confidence: float,
    ) -> List[Tuple[float, float]]:
        """Wszystkie potwierdzone OOK (bez filtrowania po desired)."""
        sites = []
        for target, ook in zip(ordered_targets, ook_results):
            if ook.get("freq") is not None and ook.get("confidence", 0) >= min_confidence:
                sites.append((target["lat"], target["lon"]))
        return sites
