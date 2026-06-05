#!/usr/bin/env python3
"""
Punkt wejścia systemu. Tworzy Pipe(), spawnuje Mózg i Oko, czeka na zakończenie.

Jeden UART do FC obsługuje wyłącznie Mózg. Oko korzysta tylko z kamery.

Uruchomienie:
    python main.py --uart /dev/ttyAMA0
"""
from __future__ import annotations

import argparse
from multiprocessing import Process, Pipe

from mission.brain import run_mission_manager
from vision.eye import run_vision_detector


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttyAMA0", help="port UART do FC")
    args = ap.parse_args()

    mission_conn, vision_conn = Pipe()

    p_brain = Process(target=run_mission_manager, args=(mission_conn, args.uart))
    p_eye = Process(target=run_vision_detector, args=(vision_conn,))

    p_eye.start()
    p_brain.start()

    p_brain.join()      # Mózg dyktuje koniec (wysyła SHUTDOWN do Oka)
    p_eye.join(timeout=5)
    if p_eye.is_alive():
        p_eye.terminate()


if __name__ == "__main__":
    main()
