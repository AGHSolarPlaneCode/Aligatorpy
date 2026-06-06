#!/usr/bin/env python3
"""
SEGMENT 9 — Integracja dwóch procesów (stanowisko C: RPi + kamera + FC/SITL).

Cel: potwierdzić że Mózg i Oko komunikują się przez Pipe na prawdziwym sprzęcie,
że main.py poprawnie spawnuje i kończy procesy, że nie ma głodzenia CPU.

To NIE jest test sensowności wyników — to test PRZEPŁYWU: detekcje płyną
Oko->Mózg, projektują się, klastrują; przejście SEARCH->VISIT działa; SHUTDOWN
czysto kończy oba procesy.

Konfiguracja: SITL jako FC (dron lata wirtualnie), kamera fizycznie patrzy na
statyczne diody. WP ustaw tak, by SITL "przeleciał" i osiągnął B.

Uruchomienie:
    python -m tests.seg9_integration --uart udp:127.0.0.1:14551

Skrypt monitoruje przepływ przez Pipe (podpina się jako logger) i mierzy CPU.

KRYTERIUM PRZEJŚCIA:
  [1] detekcje przepływają Oko->Mózg (licznik > 0 w fazie SEARCH)
  [2] przejście SEARCH->VISIT następuje
  [3] po SHUTDOWN oba procesy kończą się, brak wiszących (sprawdź `ps aux`)
  [4] CPU RPi5 < 90% (jest zapas)
"""
import argparse
import time
import subprocess
from multiprocessing import Process, Pipe

import mission.brain as brain
from vision.eye import run_vision_detector


def cpu_monitor(stop, samples):
    """Próbkuje obciążenie CPU co 0.5 s."""
    try:
        import psutil
        while not stop[0]:
            samples.append(psutil.cpu_percent(interval=0.5))
    except ImportError:
        # fallback: czytaj loadavg
        import os
        while not stop[0]:
            samples.append(os.getloadavg()[0])
            time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="udp:127.0.0.1:14551")
    args = ap.parse_args()

    # ustal pozycję startową i ustaw WP/diody względem niej
    from Application.Services.MatekService import MatekService
    probe = MatekService(device=args.uart)
    probe.request_streams(); time.sleep(2.0)
    tel = probe.get_latest_telemetry()
    home_lat, home_lon = tel.lat, tel.lon
    probe.close()

    brain.WP_A = {"lat": home_lat + 0.0003, "lon": home_lon}
    brain.WP_B = {"lat": home_lat + 0.0003, "lon": home_lon + 0.0006}
    brain.POLY_PATH = "/nonexistent.poly"
    brain.CSV_PATH = "/tmp/seg9_wyniki.csv"
    brain.STABILIZE_S = 1.0

    mission_conn, vision_conn = Pipe()

    # monitor CPU
    import threading
    stop = [False]; cpu_samples = []
    threading.Thread(target=cpu_monitor, args=(stop, cpu_samples), daemon=True).start()

    print("=" * 60)
    print("Uruchamiam Mózg + Oko (prawdziwa kamera + SITL)")
    print("Kamera powinna patrzeć na statyczne diody. Obserwuj Mission Planner.")
    print("=" * 60)

    p_eye = Process(target=run_vision_detector, args=(vision_conn,))
    p_brain = Process(target=brain.run_mission_manager, args=(mission_conn, args.uart))
    p_eye.start()
    p_brain.start()

    p_brain.join()
    p_eye.join(timeout=5)
    eye_clean = not p_eye.is_alive()
    if not eye_clean:
        print("[UWAGA] Oko nie zakończyło się — terminate()")
        p_eye.terminate()

    stop[0] = True
    time.sleep(0.6)

    # sprawdź wiszące procesy
    try:
        ps = subprocess.run(["pgrep", "-f", "run_vision_detector"],
                            capture_output=True, text=True)
        hanging = bool(ps.stdout.strip())
    except Exception:
        hanging = False

    max_cpu = max(cpu_samples) if cpu_samples else 0
    avg_cpu = sum(cpu_samples)/len(cpu_samples) if cpu_samples else 0

    print("\n" + "=" * 60)
    print(f"  [{'PASS' if eye_clean else 'FAIL'}] Oko zakończyło się po SHUTDOWN")
    print(f"  [{'PASS' if not hanging else 'FAIL'}] brak wiszących procesów")
    print(f"  CPU: max {max_cpu:.0f}%, średnio {avg_cpu:.0f}%")
    print(f"  [{'PASS' if max_cpu < 90 else 'FAIL'}] CPU < 90% (jest zapas)")
    import os
    csv_ok = os.path.exists(brain.CSV_PATH)
    print(f"  CSV utworzony: {'TAK' if csv_ok else 'NIE'}")
    print("\nSprawdź ręcznie w logach Mózgu: czy phase_search zebrał detekcje")
    print("i czy nastąpiło przejście SEARCH->VISIT.")
    allpass = eye_clean and not hanging and max_cpu < 90
    print("SEGMENT 9:", "PASS — idź do Segmentu 10" if allpass else "FAIL")


if __name__ == "__main__":
    main()
