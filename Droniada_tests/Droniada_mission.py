from __future__ import annotations
import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
import json
import time
from typing import Any, Dict, List, Tuple

from Application.Logger.log_module import get_logger
from Application.Services.DetectionPipelineService import DetectionPipelineService
from Application.Services.gi_camera_handler import CameraPipeline
from Application.Services.MatekService import MatekService
from Application.Services.MissionPlannerService import MissionPlannerService
from Application.Services.MissionService import MissionService
from Application.Services.OokDetectionService import OokDetectionService
from Application.configuration.config_loader import cfg


class DroniadaMissionOrchestrator:
    """
    Orchestrator misji zawodów — jeden UART, jeden wątek MAVLink.

    Kamera: CameraPipeline (gi_camera_handler) w głównym wątku.
    """

    def __init__(self, dry_run: bool = False, camera: CameraPipeline | None = None):
        self.logger = get_logger(__name__)
        self.dry_run = dry_run
        #self.drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
        self.drone = MatekService(device="tcp:192.168.161.52:5763")
        self.mission = MissionService(self.drone)
        self.planner = MissionPlannerService()
        self.camera = camera or CameraPipeline()
        self.mission_cfg = cfg.mission
        self.detection = DetectionPipelineService(
            drone=self.drone,
            camera=self.camera,
            mission=self.mission,
            fps=10,
        )

    def _save_results(self, data: dict) -> None:
        path = cfg.dirs.targets_file
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.logger.info(f"Results saved to {path}")

    def _run_detection_phase(self) -> List[Dict[str, Any]]:
        """
        Detekcja LED w głównym wątku: monitor wp + klatki @ 10 Hz.
        MAVLink (GPS/attitude/wp) tylko tutaj — bez drugiego UART.
        """
        self.detection.prepare(
            is_bottle=self.mission_cfg.is_bottle,
            start_camera=False,
        )
        self.logger.info(
            f"Detection phase: wp {self.mission_cfg.start_wp} -> {self.mission_cfg.stop_wp}"
        )

        interval = 1.0 / 10
        while True:
            loop_start = time.monotonic()

            curr_wp = self.drone.get_mission_status()
            print(f"Detection phase - current waypoint: {curr_wp}")
            if curr_wp >= self.mission_cfg.stop_wp:
                self.logger.info(f"Reached wp {curr_wp}, stopping detection")
                break

            self.detection.process_one_frame(is_bottle=self.mission_cfg.is_bottle)

            elapsed = time.monotonic() - loop_start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        targets = list(self.mission.TRG_CANDIDATES)
        self.logger.info(f"Detection phase complete: {len(targets)} targets")
        return targets

    def _run_loiter_and_ook_phase(
        self,
        ordered_targets: List[Dict[str, Any]],
        loiter_start_wp: int,
    ) -> List[Dict[str, Any]]:
        
        ook_service = OokDetectionService(self.camera)
        ook_results: List[Dict[str, Any]] = []

        loiter_count = len(ordered_targets)
        loiter_end_wp = loiter_start_wp + loiter_count
        current_loiter_idx = 0

        self.logger.info(
            f"LOITER/OOK phase: monitoring wp {loiter_start_wp}..{loiter_end_wp - 1}"
        )

        while current_loiter_idx < loiter_count:
            
            curr_wp = self.drone.get_mission_status()
            
            expected_wp = loiter_start_wp + current_loiter_idx
            self.logger.info(f"LOITER/OOK phase - current waypoint: {curr_wp}, "f"expecting {expected_wp}")

            if curr_wp == expected_wp:
                target = ordered_targets[current_loiter_idx]
                self.logger.info(
                    f"At LOITER wp {curr_wp} over target "
                    f"({target['lat']:.6f}, {target['lon']:.6f}) — starting OOK"
                )
                ook_result = ook_service.detect_modulation()
                ook_results.append(ook_result)

                if (
                    ook_result.get("freq") is not None
                    and ook_result.get("confidence", 0) >= self.mission_cfg.ook.min_confidence
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
        #self.drone.set_mission_current_rate(10)
        self.drone.set_telemetry_rate(10)
        self.camera.start()
        self.camera.set_10fps_active(True)
        if not self.camera.wait_ready():
            self.logger.warning("Camera pipeline started but no frame yet")

        try:
            self.logger.info(f"Waiting for detection start wp {self.mission_cfg.start_wp}")
            while True:
                curr_wp = self.drone.get_mission_status()
                print(f"Waiting for detection start wp {self.mission_cfg.start_wp}, current waypoint: {curr_wp}")
                if curr_wp == self.mission_cfg.start_wp:
                    break
                time.sleep(0.1)

            targets = self._run_detection_phase()
            if not targets:
                self.logger.warning("No targets detected — aborting mission extension")
                return {"targets": [], "landing_sites": []}

            coords = self.drone.get_current_coordinates()
            if coords is None:
                self.logger.error("No GPS for route planning")
                return {"targets": targets, "landing_sites": []}

            start_lat, start_lon, _ = coords
            ordered = self.planner.order_targets_nearest(start_lat, start_lon, targets)
            loiter_wps = self.planner.build_loiter_waypoints(
                ordered, self.mission_cfg.loiter
            )

            curr_wp_before = self.drone.get_mission_status()
            loiter_start_wp = curr_wp_before + 1

            if not self.dry_run:
                ok = self.drone.append_waypoints(loiter_wps)
                if not ok:
                    self.logger.error("Failed to append LOITER waypoints")
                    return {"targets": ordered, "landing_sites": []}
            else:
                self.logger.info(f"[dry-run] Would append {len(loiter_wps)} LOITER waypoints")

            ook_results = self._run_loiter_and_ook_phase(ordered, loiter_start_wp)

            landing_sites = self.planner.select_desired_landing_sites(
                ordered,
                ook_results,
                self.mission_cfg.ook.desired,
                self.mission_cfg.ook.min_confidence,
            )

            if landing_sites and not self.dry_run:
                self.mission.process_landing_sites_drone(landing_sites)
                #self.drone.send_landing_sites(landing_sites)
            elif landing_sites:
                self.logger.info(f"[dry-run] Would send {len(landing_sites)} landing sites")
            elif self.mission_cfg.ook.desired:
                self.logger.warning(
                    "No landing sites matched desired frequencies "
                    f"{list(self.mission_cfg.ook.desired)}"
                )

            result = {
                "targets": ordered,
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
    orchestrator = DroniadaMissionOrchestrator(dry_run=dry_run, camera=camera)
    return orchestrator.run()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Droniada competition mission orchestrator")
    parser.add_argument("--dry-run", action="store_true", help="Skip append_waypoints and send_landing_sites")
    args = parser.parse_args()
    main(dry_run=args.dry_run)
