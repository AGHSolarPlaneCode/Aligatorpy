#nieuzywane w droniadzie

import os
import csv
import time
import threading
from datetime import datetime

from picamera2 import Picamera2
from pymavlink import mavutil

from Application.Logger.log_module import get_logger
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg  # mandatory

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

class CameraService:
    # --- LED/OOK detection defaults (from former LED_detect_cam.py) ---
    THRESHOLD_VALUE = 200
    MIN_AREA = 9
    MAX_AREA = 400
    MERGE_RADIUS = 50
    RESOLUTION_W = 1280
    RESOLUTION_H = 800

    def __init__(self, drone: MatekService = None):
        self.logger = get_logger(__name__)
        self.PHOTOS_DIR = cfg.dirs.photos_dir
        self.PHOTOS_MISSION_DIR = self.PHOTOS_DIR / f"mission_{datetime.now().strftime('Mission_%Y-%m-%d_%H-%M')}"
        os.makedirs(self.PHOTOS_MISSION_DIR, exist_ok=True)
        self.LOG_FILE = self.PHOTOS_MISSION_DIR / 'photos_position.csv'
        self.resolution = cfg.camera.resolution
        if drone is None:
            self.drone = MatekService(device = cfg.mav.device2, baud = cfg.mav.baud)     #TODO zmień to w uj
        else:
            self.drone = drone

        self.picam = Picamera2()
        config = self.picam.create_still_configuration(main={"size": self.resolution})
        config["main"]["quality"] = 100
        self.picam.configure(config)
        self.picam.start()
        self.picam.set_controls({"ExposureTime": 10000,
                                    "AfMode": 0,          # 0 = Manual Focus (blokuje silniczek AF)
                                    "LensPosition": 0.0,  # 0.0 = Nieskończoność (dioptrie)
                                    "AfRange": 0          # 0 = Normal (opcjonalnie, ogranicza zakres pracy)
                                })

        self.stop_event = threading.Event()

        # State for LED detection (was global variables in LED_detect_cam.py)
        self._detected_targets = []
        self._target_id_counter = 0
        
        with open(self.LOG_FILE, 'w', newline='') as f:
            csv.writer(f).writerow(['Filename', 'Index', 'Lat', 'Lon', 'Alt', 'Roll', 'Pitch', 'Yaw'])

    def image_capture_listener(self):
        """
        Nasłuchuje komunikatów CAMERA_FEEDBACK i reaguje na nie, wykonując zdjęcia i zapisując dane.
        Ten kod powinien być uruchomiony w osobnym wątku lub procesie, aby nie blokować głównej logiki drona.
        """
        log_file = open(self.LOG_FILE, 'a', newline='')
        while True:
            msg = self.drone.master.recv_match(type='CAMERA_FEEDBACK', blocking=True)   # 'TERRAIN_REPORT'
            if msg:
                self.image_capture(log_file, msg)

    def image_capture(self, f_handle, msg):
        '''
        Args:
            f_handle: uchwyt do otwartego pliku CSV, gdzie będą zapisywane dane
            msg: komunikat CAMERA_FEEDBACK z danymi o zdjęciu i pozycji
        '''
        ts = datetime.now().strftime("%m-%d_%H:%M:%S:%f")[:-3]
        img_idx = msg.img_idx
        filename = f"IMG_{img_idx:04d}_{ts}.jpg"
        self.picam.capture_file(self.PHOTOS_MISSION_DIR/filename)
        self.logger.info(f"Zrobiono zdjęcie: {filename} (img_idx={img_idx})")
        lat = msg.lat / 1e7
        lon = msg.lng / 1e7
        alt = msg.alt_msl        # Wysokość n.p.m. (lub relatywna, zależnie od ustawień)
        r, p, y = msg.roll, msg.pitch, msg.yaw
        
        # 2. Zapis precyzyjnych danych z komunikatu
        csv.writer(f_handle).writerow([filename, img_idx, lat, lon, alt, r, p, y])
    
    @staticmethod
    def image_capture_test(f_handle, msg):
        '''
        Mimics working if image capture for testing, use only on Earth
        Args:
            f_handle: uchwyt do otwartego pliku CSV, gdzie będą zapisywane dane
            msg: komunikat CAMERA_FEEDBACK z danymi o zdjęciu i pozycji
        '''
        ts = datetime.now().strftime("%m-%d_%H:%M:%S:%f")[:-3]
        img_idx = 1#msg.img_idx
        filename = f"IMG_{img_idx:04d}_{ts}.jpg"
        #self.picam.capture_file(self.PHOTOS_MISSION_DIR/filename)
        print(f"Zrobiono zdjęcie: {filename} (img_idx={img_idx})")
        lat = 1#msg.lat / 1e7
        lon = 1#msg.lng / 1e7
        alt = 1#msg.alt_msl        # Wysokość n.p.m. (lub relatywna, zależnie od ustawień)
        r, p, y = 2,2,2#msg.roll, msg.pitch, msg.yaw
        
        # 2. Zapis precyzyjnych danych z komunikatu
        csv.writer(f_handle).writerow([filename, img_idx, lat, lon, alt, r, p, y])


    def MappingListener(self):        
        logger = get_logger("MappingListener")
        with open(self.LOG_FILE, 'a', newline='') as log_file:
            while not self.stop_event.is_set():    
                msg = self.drone.master.recv_match(type='CAMERA_FEEDBACK', blocking=True)   # 'TERRAIN_REPORT'
                if msg:
                    logger.info(f"Received CAMERA_FEEDBACK message: img_idx={msg.img_idx}")
                    self.image_capture(log_file, msg)

    # ---------------- LED detection API ----------------
    def reset_led_detector(self) -> None:
        """Resets accumulated LED detections (ids/positions) for a fresh run."""
        self._detected_targets = []
        self._target_id_counter = 0

    def process_led_frame(self, frame):
        """
        Process one camera frame and update internal detected targets.

        Returns:
            display_img: visualization image (BGR) or None if cv2 is missing
            targets_snapshot: list of detected targets dicts
        """
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/numpy are required for LED detection but are not installed.")

        # Picamera2 capture_array usually returns RGB. Convert to grayscale.
        if hasattr(frame, "ndim") and frame.ndim == 3:
            gray = cv2.cvtColor(frame[: self.RESOLUTION_H, : self.RESOLUTION_W], cv2.COLOR_RGB2GRAY)
        else:
            gray = frame[: self.RESOLUTION_H, : self.RESOLUTION_W]

        _, thresh = cv2.threshold(gray, self.THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_frame_centroids = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.MIN_AREA < area < self.MAX_AREA:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    current_frame_centroids.append((cx, cy))

        # Age existing targets.
        for target in self._detected_targets:
            target["frames_unseen"] += 1

        # Merge/associate current detections with existing targets.
        for (cx, cy) in current_frame_centroids:
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

        # Visualization.
        display_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for target in self._detected_targets:
            if target["frames_unseen"] < 60:
                cv2.circle(display_img, (target["x"], target["y"]), self.MERGE_RADIUS, (0, 255, 0), 1)
                cv2.putText(
                    display_img,
                    f"ID:{target['id']}",
                    (target["x"] + 10, target["y"] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                cv2.drawMarker(
                    display_img,
                    (target["x"], target["y"]),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    self.MERGE_RADIUS // 5,
                    2,
                )

        # Snapshot (avoid callers mutating internal state).
        targets_snapshot = [t.copy() for t in self._detected_targets]
        return display_img, targets_snapshot

    def configure_for_streaming(self, size=None, fps=10):
        """Przełącza kamerę w tryb wideo do detekcji na żywo."""
        size = size or self.resolution
        self.picam.stop()
        config = self.picam.create_video_configuration(main={"size": size, "format": "RGB888"})
        config["controls"] = {"FrameRate": fps}
        self.picam.configure(config)
        self.picam.start()
        self.picam.set_controls({
            "ExposureTime": 10000,
            "AfMode": 0,
            "LensPosition": 0.0,
            "AfRange": 0,
        })
        self.logger.info(f"Camera streaming mode: {size[0]}x{size[1]} @ {fps}fps")

    def capture_frame(self):
        """Pobiera jedną klatkę z kamery (tryb wideo)."""
        return self.picam.capture_array("main")

    def led_detection_capture_loop(self, stop_event=None, max_frames=None):
        """
        Convenience loop: captures frames from Picamera2, runs `process_led_frame`.

        This is meant for local testing / manual runs; for mission-time usage prefer
        calling `process_led_frame()` explicitly on frames in your own loop.
        """
        if stop_event is None:
            stop_event = self.stop_event

        frame_count = 0
        while not stop_event.is_set():
            frame = self.capture_frame()
            display_img, targets = self.process_led_frame(frame)
            frame_count += 1

            # Minimal non-blocking behavior: just keep last targets.
            # (No cv2.imshow() here because that would block program flow.)
            if display_img is not None and False:  # keep visualization optional
                cv2.imshow("LED detection", display_img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if max_frames is not None and frame_count >= max_frames:
                break


                
