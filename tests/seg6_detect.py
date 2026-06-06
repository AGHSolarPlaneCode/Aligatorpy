#!/usr/bin/env python3
"""
SEGMENT 6 — Detekcja plamki (stanowisko C: dron/kamera poziomo nad diodą IR).

Cel: potwierdzić że Oko wykrywa diodę IR i zwraca sensowny piksel. To test
"kamera skierowana na diodę z ~50m" (lub mniej — ważne że dioda to punkt w kadrze).

Uruchomienie:
    python -m tests.seg6_detect --threshold 220 --seconds 20

Co robić podczas testu:
  - skieruj kamerę na świecącą diodę IR (stałą, nie migającą)
  - obserwuj wypisywany piksel — czy odpowiada pozycji diody
  - przesuwaj diodę/kamerę — piksel powinien podążać
  - dostrój --threshold jeśli dioda nie jest łapana lub tło daje fałszywki

KRYTERIUM PRZEJŚCIA:
  [1] dioda stabilnie wykrywana gdy w centralnej części kadru
  [2] piksel odpowiada wizualnej pozycji (porównaj z podglądem stream)
  [3] tło nie generuje fałszywych wykryć (gdy dioda zgaszona -> brak detekcji)
  [4] dioda przy krawędzi (>EDGE_REJECT_PX) odrzucana

To test obserwacyjny — końcowy werdykt PASS/FAIL stawiasz Ty na podstawie obserwacji.
"""
import argparse
import threading
import time

import numpy as np

from vision.camera_handler import CameraPipeline, WIDTH, HEIGHT
import vision.eye as eye
from shared.protocol import EDGE_REJECT_PX


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--threshold", type=int, default=220)
    ap.add_argument("--seconds", type=int, default=20)
    args = ap.parse_args()

    cam = CameraPipeline()
    th = threading.Thread(target=cam.start, daemon=True)
    th.start()
    time.sleep(2.0)
    cam.set_search_active(True)
    time.sleep(0.5)

    print(f"Detekcja przez {args.seconds}s, próg={args.threshold}, "
          f"EDGE_REJECT={EDGE_REJECT_PX}px, środek=({WIDTH//2},{HEIGHT//2})")
    print("Skieruj kamerę na diodę. Obserwuj piksel i dystans od środka.\n")

    hits = 0
    frames = 0
    edge_rejects = 0
    end = time.time() + args.seconds
    last_print = 0
    while time.time() < end:
        f1d, ts, err = cam.get_image()
        frame = eye._reshape(f1d)
        if frame is None:
            continue
        frames += 1

        # surowa detekcja BEZ filtra krawędzi — żeby zobaczyć też odrzucone
        import cv2
        _, thr = cv2.threshold(frame, args.threshold, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if eye.MIN_AREA < a < eye.MAX_AREA and a > best_area:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    best = (M["m10"]/M["m00"], M["m01"]/M["m00"]); best_area = a

        now = time.time()
        if best is not None:
            dist = np.hypot(best[0]-WIDTH/2, best[1]-HEIGHT/2)
            in_edge = dist > EDGE_REJECT_PX
            if in_edge:
                edge_rejects += 1
            else:
                hits += 1
            if now - last_print > 0.5:   # nie zalewaj konsoli
                tag = "ODRZUCONA (krawędź)" if in_edge else "OK"
                print(f"  piksel=({best[0]:6.1f},{best[1]:6.1f}) "
                      f"pole={best_area:4.0f} dist_środek={dist:5.0f}px  {tag}")
                last_print = now

    print(f"\nKlatek: {frames}, wykryć w centrum: {hits}, "
          f"odrzuconych przy krawędzi: {edge_rejects}")
    print("\n" + "=" * 60)
    print("WERDYKT (oceniasz Ty na podstawie obserwacji):")
    print("  [ ] dioda stabilnie wykrywana w centrum")
    print("  [ ] piksel zgadza się z pozycją diody (porównaj ze streamem)")
    print("  [ ] zgaszona dioda -> brak wykryć")
    print("  [ ] dioda przy krawędzi odrzucana")
    print(f"\nJeśli dioda nie łapana: obniż --threshold (teraz {args.threshold})")
    print("Jeśli tło daje fałszywki: podnieś --threshold lub zawęź MIN/MAX_AREA")
    print("Po dostrojeniu wpisz wartość do SEARCH_THRESHOLD w protocol.py")
    cam.stop()


if __name__ == "__main__":
    main()
