#!/usr/bin/env python3
"""
SEGMENT 3 — Komendy i protokół misji MatekService (stanowisko B: SITL).

Cel: potwierdzić że metody komend działają na żywym FC przez demux — ACK
i protokół misji nie gubią się, błędny upload jest wykrywany.

WYMAGA SITL (ArduCopter) — będziemy uzbrajać i wgrywać misję. NIE rób tego na
prawdziwym dronie z założonymi śmigłami.

Uruchomienie:
    python -m tests.seg3_commands --uart udp:127.0.0.1:14551

Test jest INTERAKTYWNY — obserwuj Mission Planner równolegle i potwierdzaj.

KRYTERIUM PRZEJŚCIA:
  [1] set_mode GUIDED — tryb zmienia się w Mission Planner
  [2] arm — SITL uzbraja się, metoda zwraca True
  [3] set_waypoints dobra misja -> True, widoczna w MP
  [4] KRYTYCZNY: set_waypoints zła misja -> False (poprawka bug ACK)
  [5] wait_item_reached wraca po dolocie do A
  [6] goto — SITL leci do punktu w GUIDED
  [7] telemetria płynie podczas uploadu (demux nie blokuje)
"""
import argparse
import threading
import time

from Application.Services.MatekService import MatekService


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="udp:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    drone = MatekService(device=args.uart, baud=args.baud)
    res = {}
    try:
        drone.request_streams()
        time.sleep(2.0)
        tel = drone.get_latest_telemetry()
        if tel is None:
            print("[FAIL] brak telemetrii — uruchom Segment 2 najpierw")
            return 1
        home_lat, home_lon = tel.lat, tel.lon
        print(f"Pozycja startowa: {home_lat:.6f}, {home_lon:.6f}")

        # [1] GUIDED
        res["GUIDED"] = drone.set_mode("GUIDED")
        print(f"[1] set_mode GUIDED -> {res['GUIDED']}  (sprawdź MP)")

        # [2] arm
        res["arm"] = drone.arm()
        print(f"[2] arm -> {res['arm']}  (sprawdź MP że uzbrojony)")

        # [3] dobra misja
        a_lat, a_lon = home_lat + 0.0003, home_lon
        b_lat, b_lon = home_lat + 0.0003, home_lon + 0.0004
        good = [
            {"command": "TAKEOFF", "alt": 50},
            {"command": "WAYPOINT", "lat": a_lat, "lon": a_lon, "alt": 50, "acr": 3},
            {"command": "WAYPOINT", "lat": b_lat, "lon": b_lon, "alt": 50, "acr": 3},
        ]
        res["upload_good"] = drone.set_waypoints(good)
        print(f"[3] set_waypoints dobra misja -> {res['upload_good']}  (sprawdź MP: 4 punkty)")

        # [4] KRYTYCZNY: zła misja (waypoint bez 'lat') powinna dać False
        bad = [{"command": "WAYPOINT", "alt": 50, "acr": 3}]  # brak lat/lon
        try:
            res["upload_bad"] = (drone.set_waypoints(bad) is False)
        except Exception:
            res["upload_bad"] = True   # wyjątek też akceptowalny — nie zwróciło True
        print(f"[4] set_waypoints zła misja wykryta -> {res['upload_bad']} "
              f"(MUSI być True = upload odrzucony)")
        # przywróć dobrą misję
        drone.set_waypoints(good)

        # [5] wait_item_reached — uruchom misję i czekaj na A
        print("\n[5] Uruchamiam misję (AUTO) i czekam na dolot do A (seq 2)...")
        drone.set_mode("AUTO")
        drone.mission_start()
        drone.set_speed(5.0)
        reached = drone.wait_item_reached({2}, timeout=120)
        res["reached_A"] = (reached == 2)
        print(f"    wait_item_reached -> {reached}  {'OK' if res['reached_A'] else 'FAIL/timeout'}")

        # [6] goto w GUIDED
        print("\n[6] GUIDED-goto do punktu testowego...")
        drone.set_mode("GUIDED")
        goto_lat, goto_lon = home_lat + 0.0005, home_lon + 0.0002
        drone.goto(goto_lat, goto_lon, 50)
        time.sleep(2.0)
        moved = drone.get_latest_telemetry()
        # sprawdź czy w ogóle zaczął się zbliżać (nie czekamy na pełny dolot)
        from shared.clustering import distance_m
        d_before = distance_m(home_lat, home_lon, goto_lat, goto_lon)
        d_now = distance_m(moved.lat, moved.lon, goto_lat, goto_lon)
        res["goto"] = d_now < d_before
        print(f"    dystans do celu: start {d_before:.0f}m -> teraz {d_now:.0f}m "
              f"{'OK (zbliża się)' if res['goto'] else 'FAIL'}")

        # [7] demux podczas uploadu — telemetria nie zamarła
        print("\n[7] Test współbieżności: upload + jednoczesny odczyt telemetrii...")
        stop = [False]
        ts_seen = []
        def watch():
            while not stop[0]:
                t = drone.get_latest_telemetry()
                if t: ts_seen.append(t.ts)
                time.sleep(0.05)
        th = threading.Thread(target=watch, daemon=True); th.start()
        drone.set_waypoints(good)        # upload podczas obserwacji
        time.sleep(0.5); stop[0] = True; th.join()
        unique_ts = len(set(ts_seen))
        res["concurrent"] = unique_ts > 3   # telemetria się aktualizowała
        print(f"    unikalne znaczniki telemetrii podczas uploadu: {unique_ts} "
              f"{'OK (demux działa)' if res['concurrent'] else 'FAIL (zamarła)'}")

        # rozbroj na koniec
        drone.set_mode("RTL")

        print("\n" + "=" * 60)
        allpass = True
        for k, v in res.items():
            print(f"  [{'PASS' if v else 'FAIL'}] {k}")
            allpass &= bool(v)
        print("SEGMENT 3:", "PASS — idź do Segmentu 4" if allpass else "FAIL")
        return 0 if allpass else 1
    finally:
        drone.close()


if __name__ == "__main__":
    raise SystemExit(main())
