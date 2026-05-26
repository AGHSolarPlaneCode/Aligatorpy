import os
import sys
import threading
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Application.Logger.log_module import get_logger
from Application.Services.DetectionPipelineService import DetectionPipelineService
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg


def main():
    logger = get_logger(__name__)

    # Konfiguracja testu
    start_wp = 5
    stop_wp = 96
    is_bottle = True
    fps = 10

    drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
    pipeline = DetectionPipelineService(drone=drone, fps=fps)

    # Monitoring aktywnego waypointa
    drone.set_mission_current_rate(10)

    stop_event = threading.Event()
    pipeline_thread = None
    pipeline_started = False

    try:
        while True:
            curr_wp = drone.get_mission_status()
            logger.info(f"Current waypoint: {curr_wp}")

            if curr_wp == start_wp and not pipeline_started:
                pipeline_started = True
                logger.info(f"Reached waypoint {curr_wp}, starting detection pipeline")

                pipeline_thread = threading.Thread(
                    target=pipeline.run,
                    kwargs={
                        "stop_event": stop_event,
                        "is_bottle": is_bottle,
                    },
                    daemon=True,
                )
                pipeline_thread.start()

            if pipeline_started and curr_wp >= stop_wp:
                logger.info(f"Reached waypoint {curr_wp}, stopping detection pipeline")
                stop_event.set()
                break

            time.sleep(0.2)

        if pipeline_thread is not None:
            pipeline_thread.join(timeout=10)

        targets = pipeline.mission.TRG_CANDIDATES
        logger.info(f"Detected targets count: {len(targets)}")
        for i, target in enumerate(targets, start=1):
            logger.info(f"Target {i}: {target}")

    finally:
        drone.close()


if __name__ == "__main__":
    main()
