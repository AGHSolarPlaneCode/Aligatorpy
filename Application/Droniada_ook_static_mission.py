from __future__ import annotations

import json
import os
import sys
import time
from typing import Any, Dict, List

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Application.Logger.log_module import get_logger
from Application.Services.gi_camera_handler import CameraPipeline
from Application.Services.MatekService import MatekService
from Application.Services.MissionPlannerService import MissionPlannerService
from Application.Services.MissionService import MissionService
from Application.Services.OokDetectionService import OokDetectionService
from Application.configuration.config_loader import cfg


class DroniadaOokStaticMissionOrchestrator:
    """
    Misja OOK na pełnej trasie z Mission Planner — bez dopisywania waypointów lotu.

    Przepływ:
    0. Start kamery przy tworzeniu orchestratora.
    1. Pobranie misji z FC i wykrycie NAV_LOITER_TIME (lub indeksy z config).
    2. Oczekiwanie na monitor_start_wp, detekcja modulacji na każdym LOITER.
    3. Wybór lądowisk (desired) i dopisanie waypointów zrzutu.
    """

    def __init__(
        self,
        dry_run: bool = False,
        camera: CameraPipeline | None = None,
        auto_start_camera: bool = True,
    ):
        self.logger = get_logger(__name__)
        self.dry_run = dry_run
        self.camera = camera or CameraPipeline()
        if auto_start_camera:
            self._init_camera()
        self.drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
        self.mission = MissionService(self.drone)
        self.planner = MissionPlannerService()
        self.mission_cfg = cfg.mission
        self.static_cfg = cfg.mission.ook_static

    def _init_camera(self) -> None:
        self.logger.info("Starting camera pipeline")
        self.camera.start()
        self.camera.set_10fps_active(True)
        if not self.camera.wait_ready():
            self.logger.warning("Camera pipeline started but no frame yet")
        else:
            self.logger.info("Camera ready")

    def _save_results(self, data: dict) -> None:
        path = cfg.dirs.targets_file
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.logger.info(f"Results saved to {path}")

    def _resolve_loiter_route(
        self,
    ) -> tuple[List[int], List[Dict[str, Any]]]:
        mission_items = self.drone.get_mission()
        if not mission_items:
            raise RuntimeError("FC mission is empty — upload mission in Mission Planner first")

        explicit = list(self.static_cfg.loiter_wp_indices) or None
        loiter_wp_indices, targets = self.planner.extract_loiter_sites_from_mission(
            mission_items, explicit
        )
        if not loiter_wp_indices:
            raise RuntimeError(
                "No NAV_LOITER_TIME waypoints found in FC mission — "
                "add LOITER points in MP or set mission.ook_static.loiter_wp_indices"
            )

        self.logger.info(
            f"OOK route from FC mission: {len(loiter_wp_indices)} LOITER wp "
            f"at indices {loiter_wp_indices}"
        )
        return loiter_wp_indices, targets

    def _run_loiter_and_ook_phase(
        self,
        ordered_targets: List[Dict[str, Any]],
        loiter_wp_indices: List[int],
    ) -> List[Dict[str, Any]]:
        ook_service = OokDetectionService(self.camera)
        ook_results: List[Dict[str, Any]] = []
        loiter_count = len(loiter_wp_indices)
        current_loiter_idx = 0

        self.logger.info(
            f"LOITER/OOK phase: monitoring LOITER wp indices {loiter_wp_indices}"
        )

        while current_loiter_idx < loiter_count:
            self.camera.get_image()
            curr_wp = self.drone.get_mission_status()
            expected_wp = loiter_wp_indices[current_loiter_idx]
            self.logger.info(
                f"LOITER/OOK phase - current waypoint: {curr_wp}, "
                f"expecting LOITER {expected_wp}"
            )

            if curr_wp == expected_wp:
                target = ordered_targets[current_loiter_idx]
                self.logger.info(
                    f"At LOITER wp {curr_wp} over site "
                    f"({target['lat']:.6f}, {target['lon']:.6f}) — starting OOK "
                    f"({self.mission_cfg.ook.duration_s}s)"
                )
                ook_result = ook_service.detect_modulation()
                ook_results.append(ook_result)

                if (
                    ook_result.get("freq") is not None
                    and ook_result.get("confidence", 0)
                    >= self.mission_cfg.ook.min_confidence
                ):
                    self.logger.info(
                        f"OOK confirmed at ({target['lat']:.6f}, {target['lon']:.6f}): "
                        f"{ook_result['freq']}Hz, confidence={ook_result['confidence']:.2f}"
                    )
                else:
                    self.logger.warning(
                        f"OOK not confirmed at ({target['lat']:.6f}, {target['lon']:.6f})"
                    )

                current_loiter_idx += 1

            time.sleep(0.2)

        return ook_results

    def run(self) -> dict:
        self.drone.set_telemetry_rate(10)

        try:
            loiter_wp_indices, targets = self._resolve_loiter_route()

            start_wp = self.static_cfg.monitor_start_wp
            self.logger.info(f"Waiting for monitor start wp {start_wp}")
            while True:
                self.camera.get_image()
                curr_wp = self.drone.get_mission_status()
                if curr_wp == start_wp:
                    break
                time.sleep(0.1)

            ook_results = self._run_loiter_and_ook_phase(targets, loiter_wp_indices)

            landing_sites = self.planner.select_desired_landing_sites(
                targets,
                ook_results,
                self.mission_cfg.ook.desired,
                self.mission_cfg.ook.min_confidence,
            )

            if landing_sites and not self.dry_run:
                self.mission.process_landing_sites_drone(landing_sites)
            elif landing_sites:
                self.logger.info(
                    f"[dry-run] Would append drop waypoints for {len(landing_sites)} sites"
                )
            elif self.mission_cfg.ook.desired:
                self.logger.warning(
                    "No landing sites matched desired frequencies "
                    f"{list(self.mission_cfg.ook.desired)}"
                )

            result = {
                "targets": targets,
                "loiter_wp_indices": loiter_wp_indices,
                "ook_results": ook_results,
                "desired_frequencies": list(self.mission_cfg.ook.desired),
                "landing_sites": [{"lat": lat, "lon": lon} for lat, lon in landing_sites],
            }
            self._save_results(result)
            return result

        finally:
            self.camera.stop()
            self.drone.close()


def main(dry_run: bool = False, camera: CameraPipeline | None = None):
    orchestrator = DroniadaOokStaticMissionOrchestrator(
        dry_run=dry_run, camera=camera
    )
    return orchestrator.run()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Droniada OOK static mission — full route from Mission Planner"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip process_landing_sites_drone",
    )
    args = parser.parse_args()
    main(dry_run=args.dry_run)
