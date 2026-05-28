import os
import sys
import time
from multiprocessing import Event, Process, Queue

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Application.Logger.log_module import get_logger
from Application.Services.DetectionPipelineService import run_led_detection_pipeline
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg


def log_targets(logger, targets):
    logger.info(f"Detected targets count: {len(targets)}")
    for i, target in enumerate(targets, start=1):
        logger.info(f"Target {i}: {target}")


def main():
    logger = get_logger(__name__)

    start_wp = 5
    stop_wp = 96
    is_bottle = True
    fps = 10

    # Główna pętla misji — osobne połączenie MAVLink (device)
    drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
    drone.set_mission_current_rate(10)

    stop_event = Event()
    results_queue = Queue()
    pipeline_process = None
    pipeline_started = False
    targets = []

    try:
        while True:
            curr_wp = drone.get_mission_status()
            logger.info(f"Current waypoint: {curr_wp}")

            if curr_wp == start_wp and not pipeline_started:
                pipeline_started = True
                logger.info(
                    f"Reached waypoint {curr_wp}, starting detection pipeline process "
                    f"on {cfg.mav.device2}"
                )

                pipeline_process = Process(
                    target=run_led_detection_pipeline,
                    kwargs={
                        "stop_event": stop_event,
                        "is_bottle": is_bottle,
                        "fps": fps,
                        "device": cfg.mav.device2,
                        "baud": cfg.mav.baud,
                        "result_queue": results_queue,
                    },
                    daemon=True,
                )
                pipeline_process.start()

            if pipeline_started and curr_wp >= stop_wp:
                logger.info(f"Reached waypoint {curr_wp}, stopping detection pipeline")
                stop_event.set()
                break

            time.sleep(0.2)

        if pipeline_process is not None:
            pipeline_process.join(timeout=15)
            if pipeline_process.is_alive():
                logger.warning("Pipeline process still alive after timeout")
            if not results_queue.empty():
                targets = results_queue.get()

        log_targets(logger, targets)

    finally:
        drone.close()


if __name__ == "__main__":
    main()
