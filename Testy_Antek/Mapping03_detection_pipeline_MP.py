import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Application.Logger.log_module import get_logger
from Application.Services.DetectionPipelineService import DetectionPipelineService
from Application.Services.gi_camera_handler import CameraPipeline
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import cfg


def log_targets(logger, targets):
    logger.info(f"Detected targets count: {len(targets)}")
    for i, target in enumerate(targets, start=1):
        logger.info(f"Target {i}: {target}")


def main():
    logger = get_logger(__name__)

    start_wp = cfg.mission.start_wp
    stop_wp = cfg.mission.stop_wp
    fps = 10
    interval = 1.0 / fps

    drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
    camera = CameraPipeline()
    mission = MissionService(drone)
    pipeline = DetectionPipelineService(drone=drone, camera=camera, mission=mission, fps=fps)

    drone.set_mission_current_rate(10)
    camera.start()
    targets = []

    try:
        pipeline_started = False

        while True:
            curr_wp = drone.get_mission_status()
            logger.info(f"Current waypoint: {curr_wp}")

            if curr_wp == start_wp and not pipeline_started:
                pipeline_started = True
                pipeline.prepare(is_bottle=cfg.mission.is_bottle, start_camera=False)
                logger.info(f"Reached wp {curr_wp}, starting detection (single UART, main thread)")

            if pipeline_started and curr_wp < stop_wp:
                loop_start = time.monotonic()
                pipeline.process_one_frame(is_bottle=cfg.mission.is_bottle)
                elapsed = time.monotonic() - loop_start
                remaining = interval - elapsed
                if remaining > 0:
                    time.sleep(remaining)
                continue

            if pipeline_started and curr_wp >= stop_wp:
                logger.info(f"Reached wp {curr_wp}, stopping detection")
                targets = list(mission.TRG_CANDIDATES)
                break

            time.sleep(0.2)

        log_targets(logger, targets)

    finally:
        camera.stop()
        drone.close()


if __name__ == "__main__":
    main()
