"""
Proces 2 — Oko (detekcja). BEZ dostępu do FC. Operuje na pikselach i czasie,
wyniki przepycha do rury (Pipe). Maszyna stanów IDLE / SEARCHING / DECODING.

SEARCH (30 fps): threshold -> środek plamki -> pre-filtr krawędzi -> Detection(raw_x,raw_y,ts).
DECODE (120 fps): lock-on do HINT-a, lepkie okienko ROI, korelacja na realnych czasach.

Undistort i rzutowanie NIE tutaj — robi Mózg.
"""
from __future__ import annotations

import threading
import time
import numpy as np
import cv2

from shared.protocol import (Cmd, Evt, PipeMsg, Detection, OokResult,
                             CANDIDATE_FREQS, OOK_WINDOW_S, OOK_ROI_PX, OOK_THRESHOLD,
                             SEARCH_THRESHOLD, EDGE_REJECT_PX, MIN_OOK_CONFIDENCE)
from shared.ook import classify_ook, ook_brightness
from vision.camera_handler import CameraPipeline, WIDTH, HEIGHT

CENTER = (WIDTH / 2.0, HEIGHT / 2.0)
MIN_AREA, MAX_AREA = 4, 400


def _reshape(frame_1d):
    """Płaski bufor GRAY8 -> (HEIGHT, WIDTH). Brak paddingu dla 1280-szer."""
    if frame_1d is None or frame_1d.size < WIDTH * HEIGHT:
        return None
    return frame_1d[:WIDTH * HEIGHT].reshape((HEIGHT, WIDTH))


def detect_blob(frame_2d, thr=SEARCH_THRESHOLD):
    """Największa plamka > thr. Zwraca (raw_x, raw_y) float albo None (po pre-filtrze krawędzi)."""
    _, th = cv2.threshold(frame_2d, thr, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for c in cnts:
        a = cv2.contourArea(c)
        if MIN_AREA < a < MAX_AREA and a > best_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                best = (M["m10"] / M["m00"], M["m01"] / M["m00"])
                best_area = a
    if best is None:
        return None
    if np.hypot(best[0] - CENTER[0], best[1] - CENTER[1]) > EDGE_REJECT_PX:
        return None   # za blisko krawędzi — duży błąd kąta ślizgowego
    return best


def _roi_brightness_and_centroid(frame_2d, cx, cy, half, thr):
    """Wytnij ROI wokół (cx,cy), policz jasność i środek plamki w pełnych koordynatach."""
    x0 = max(0, int(cx - half)); x1 = min(WIDTH, int(cx + half))
    y0 = max(0, int(cy - half)); y1 = min(HEIGHT, int(cy + half))
    roi = frame_2d[y0:y1, x0:x1]
    bright = ook_brightness(roi, thr)
    cen = None
    if bright > 0:
        _, th = cv2.threshold(roi, thr, 255, cv2.THRESH_BINARY)
        M = cv2.moments(th)
        if M["m00"] != 0:
            cen = (x0 + M["m10"] / M["m00"], y0 + M["m01"] / M["m00"])
    return bright, cen


def decode_ook(cam, hint_x, hint_y) -> OokResult:
    """
    Lock-on do plamki najbliższej HINT-owi, lepkie okienko ROI, zbiór jasności
    przez OOK_WINDOW_S, klasyfikacja na realnych znacznikach czasu.
    """
    half = OOK_ROI_PX // 2
    cx, cy = hint_x, hint_y

    # lock-on: znajdź plamkę najbliżej HINT-a na pełnej klatce
    f1d, _, err = cam.get_image()
    frame = _reshape(f1d)
    if frame is not None:
        _, th = cv2.threshold(frame, OOK_THRESHOLD, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_d = None, 1e18
        for c in cnts:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            px, py = M["m10"] / M["m00"], M["m01"] / M["m00"]
            d = (px - hint_x) ** 2 + (py - hint_y) ** 2
            if d < best_d:
                best, best_d = (px, py), d
        if best:
            cx, cy = best

    samples, times, hist_x, hist_y = [], [], [], []
    t_end = time.monotonic() + OOK_WINDOW_S
    last_ts = None
    while time.monotonic() < t_end:
        f1d, ts, err = cam.get_image()
        if f1d is None or ts is None or ts == last_ts:
            continue
        last_ts = ts
        frame = _reshape(f1d)
        if frame is None:
            continue
        bright, cen = _roi_brightness_and_centroid(frame, cx, cy, half, OOK_THRESHOLD)
        samples.append(bright)
        times.append(ts)
        if bright > 0 and cen is not None:
            hist_x.append(cen[0]); hist_y.append(cen[1])
            cx, cy = cen          # podążaj za diodą (dron kołysze się na wietrze)
        # gdy ciemno: ROI zamrożone (cx,cy bez zmian)

    if len(samples) < 4:
        return OokResult(None, cx, cy, 0.0)

    freq, conf = classify_ook(np.array(samples), np.array(times),
                              CANDIDATE_FREQS, min_confidence=MIN_OOK_CONFIDENCE)
    avg_x = float(np.mean(hist_x)) if hist_x else cx
    avg_y = float(np.mean(hist_y)) if hist_y else cy
    return OokResult(freq, avg_x, avg_y, conf)


def run_vision_detector(conn) -> None:
    cam = CameraPipeline()
    cam_thread = threading.Thread(target=cam.start, daemon=True)
    cam_thread.start()
    time.sleep(1.0)   # rozruch pipeline'u

    state = "IDLE"
    decode_hint = None
    try:
        while True:
            # komendy od Mózgu (nieblokujące)
            if conn.poll():
                msg = conn.recv()
                if msg.kind == Cmd.START_SEARCH:
                    cam.set_search_active(True); state = "SEARCHING"
                elif msg.kind == Cmd.STOP_SEARCH:
                    cam.set_search_active(False); state = "IDLE"
                elif msg.kind == Cmd.START_DECODE:
                    decode_hint = msg.payload; state = "DECODING"
                elif msg.kind == Cmd.SHUTDOWN:
                    break

            if state == "IDLE":
                time.sleep(0.005)
                continue

            if state == "SEARCHING":
                f1d, ts, err = cam.get_image()
                frame = _reshape(f1d)
                if frame is None or ts is None:
                    continue
                blob = detect_blob(frame)
                if blob is not None:
                    conn.send(PipeMsg(Evt.DETECTION, Detection(blob[0], blob[1], ts)))

            elif state == "DECODING":
                cam.set_decode_active(True)
                result = decode_ook(cam, decode_hint.hint_x, decode_hint.hint_y)
                conn.send(PipeMsg(Evt.OOK_RESULT, result))
                cam.set_decode_active(False)
                state = "IDLE"
    finally:
        cam.stop()
