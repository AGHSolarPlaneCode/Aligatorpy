#!/usr/bin/env python3
"""
SEGMENT 4 — Orchestracja misji z ATRAPĄ Oka (stanowisko B: SITL, bez kamery).

Cel: potwierdzić że Mózg przeprowadza pełną misję w SITL — całą sekwencję faz —
używając sztucznego Oka. Po tym segmencie wiesz, że cała logika misji działa,
brakuje tylko prawdziwej wizji.

Atrapa Oka (wątek na drugim końcu Pipe):
  - na START_SEARCH: wysyła sztuczne Detection, których piksele rzutują się na
    znane GPS w strefie (liczone z aktualnej telemetrii drona w SITL),
  - na START_DECODE: odsyła OokResult ze znaną częstotliwością,
  - testuje retry (raz freq=None) i Złotą Weryfikację (raz zły piksel).

WYMAGA SITL. Mózg będzie latał wirtualnie. Obserwuj Mission Planner.

Uruchomienie:
    python -m tests.seg4_mission_dummy_eye --uart udp:127.0.0.1:14551

KRYTERIUM PRZEJŚCIA:
  [1] dron przechodzi: takeoff -> A -> B -> wizyty -> RTL
  [2] phase_search kończy się po dolocie do B (nie zawiesza)
  [3] CSV zawiera diody z częstotliwościami i flagami
  [4] retry działa (freq=None -> ponowienie)
  [5] Złota Weryfikacja odrzuca zły piksel (status niepewny lub retry)
"""
import argparse
import os
import threading
import time

import mission.brain as brain
from shared.protocol import Cmd, Evt, PipeMsg, Detection, OokResult
from shared.geometry import gps_to_pixel
from shared.calibration import K, DIST, CALIB_W, CALIB_H
from shared.clustering import distance_m


def run_dummy_eye(conn, drone_ref, true_diodes, freqs, fault_flags):
    """Atrapa Oka. drone_ref to lista [MatekService] (by mieć żywą referencję)."""
    decode_count = [0]
    while True:
        if not conn.poll(timeout=2.0):
            continue
        msg = conn.recv()
        drone = drone_ref[0]
        if msg.kind == Cmd.START_SEARCH:
            # przez ~kilka sekund wysyłaj wykrycia diod widocznych z aktualnej pozy
            t_end = time.monotonic() + 8.0
            while time.monotonic() < t_end:
                tel = drone.get_latest_telemetry()
                if tel:
                    for d in true_diodes:
                        px = gps_to_pixel(d[0], d[1], tel.lat, tel.lon, tel.alt,
                                          tel.roll, tel.pitch, tel.yaw,
                                          K, DIST, CALIB_W, CALIB_H)
                        if px is not None:
                            conn.send(PipeMsg(Evt.DETECTION,
                                              Detection(px[0], px[1], time.monotonic())))
                time.sleep(0.1)
        elif msg.kind == Cmd.START_DECODE:
            decode_count[0] += 1
            tel = drone.get_latest_telemetry()
            # która dioda jest najbliżej?
            i = min(range(len(true_diodes)),
                    key=lambda j: distance_m(tel.lat, tel.lon,
                                             true_diodes[j][0], true_diodes[j][1]))
            # FAULT 1: pierwsza próba pierwszej diody -> freq=None (test retry)
            if fault_flags.get("none_once") and decode_count[0] == 1:
                conn.send(PipeMsg(Evt.OOK_RESULT, OokResult(None, CALIB_W/2, CALIB_H/2, 1.2)))
                continue
            px = gps_to_pixel(true_diodes[i][0], true_diodes[i][1],
                              tel.lat, tel.lon, tel.alt, tel.roll, tel.pitch, tel.yaw,
                              K, DIST, CALIB_W, CALIB_H)
            px = px or (CALIB_W/2, CALIB_H/2)
            # FAULT 2: druga próba -> przesunięty piksel (test Złotej Weryfikacji)
            if fault_flags.get("drift_once") and decode_count[0] == 2:
                px = (px[0] + 200, px[1] + 200)   # rzut wypadnie daleko -> dist>5m
            conn.send(PipeMsg(Evt.OOK_RESULT, OokResult(float(freqs[i]), px[0], px[1], 9.0)))
        elif msg.kind == Cmd.SHUTDOWN:
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="udp:127.0.0.1:14551")
    args = ap.parse_args()

    from multiprocessing import Pipe
    from Application.Services.MatekService import MatekService

    # Połącz tymczasowo żeby ustalić pozycję startową i zbudować diody w strefie
    probe = MatekService(device=args.uart)
    probe.request_streams(); time.sleep(2.0)
    tel = probe.get_latest_telemetry()
    home_lat, home_lon = tel.lat, tel.lon
    probe.close()

    # ustaw WP i diody względem pozycji startowej SITL
    brain.WP_A = {"lat": home_lat + 0.0003, "lon": home_lon}
    brain.WP_B = {"lat": home_lat + 0.0003, "lon": home_lon + 0.0006}
    brain.POLY_PATH = "/nonexistent.poly"   # wyłącz geofence w teście
    brain.CSV_PATH = "/tmp/seg4_wyniki.csv"
    if os.path.exists(brain.CSV_PATH):
        os.remove(brain.CSV_PATH)
    brain.STABILIZE_S = 1.0   # skróć stabilizację w teście

    # diody rozłożone wzdłuż A->B w strefie footprintu
    true_diodes = [
        (home_lat + 0.0003, home_lon + 0.00015),
        (home_lat + 0.0003, home_lon + 0.00035),
        (home_lat + 0.0003, home_lon + 0.00050),
    ]
    freqs = [4, 12, 16]
    fault_flags = {"none_once": True, "drift_once": True}

    bc, ec = Pipe()
    drone_ref = [None]

    # uruchom atrapę Oka w wątku
    th = threading.Thread(target=run_dummy_eye,
                          args=(ec, drone_ref, true_diodes, freqs, fault_flags),
                          daemon=True)
    th.start()

    # podmień MatekService tak, by atrapa miała referencję do tego samego drona
    orig_init = MatekService.__init__
    def patched_init(self, *a, **k):
        orig_init(self, *a, **k)
        drone_ref[0] = self
    MatekService.__init__ = patched_init

    print("=" * 60)
    print("Uruchamiam pełną misję z atrapą Oka — obserwuj Mission Planner")
    print("=" * 60)
    brain.run_mission_manager(bc, args.uart)

    # wyniki
    print("\n" + "=" * 60)
    print("CSV wyników:")
    if os.path.exists(brain.CSV_PATH):
        content = open(brain.CSV_PATH).read()
        print(content)
        lines = [l for l in content.splitlines() if l and not l.startswith("diode_id")]
        ok_rows = [l for l in lines if ",ok," in l]
        print(f"Wierszy ok: {len(ok_rows)} / diod: {len(true_diodes)}")
        allpass = len(ok_rows) >= len(true_diodes) - 1   # tolerancja na fault
        print("SEGMENT 4:", "PASS — idź do Segmentu 9" if allpass else "FAIL — sprawdź CSV")
    else:
        print("BRAK CSV — misja nie dotarła do fazy VISIT")
        print("SEGMENT 4: FAIL")


if __name__ == "__main__":
    main()
