#!/usr/bin/env python3
"""
Test SITL (tylko wersja zdarzeniowa MISSION_ITEM_REACHED).

Sprawdza w symulacji:
  - wgranie misji SEARCH (A -> B) i prędkości,
  - dołożenie par WAYPOINT + NAV_LOITER_TIME dla każdej diody,
  - wykrywanie dolotu do WP_B oraz nad każdą diodę przez MISSION_ITEM_REACHED.

NIE używa kamery ani workera. W miejscu pomiaru OOK wypisuje 'CAPTURE LED i'.

Diody są na sztywno (10 szt.) – leżą wewnątrz glowice_dron.poly, >=10 m od siebie.
Geofence ustaw w Mission Plannerze (instrukcja osobno), nie w tym kodzie.

ArduCopter SITL z HOME w polu (Polska!), np.:
    sim_vehicle.py -v ArduCopter --console --map --custom-location=50.27099,18.67429,200,0

Uruchomienie:
    python test_mission_sitl.py --device udp:127.0.0.1:14550
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List

from Application.Services.MatekService import MatekService

# ====================================================================== #
#  Diody na sztywno – wewnątrz glowice_dron.poly, rozrzucone, >=10 m od siebie.
# ====================================================================== #
DIODES: List[Dict] = [
    {"lat": 50.271077, "lon": 18.672761, "isBottle": True},
    {"lat": 50.271422, "lon": 18.672847, "isBottle": False},
    {"lat": 50.270948, "lon": 18.673503, "isBottle": True},
    {"lat": 50.271291, "lon": 18.673595, "isBottle": False},
    {"lat": 50.270819, "lon": 18.674244, "isBottle": True},
    {"lat": 50.271161, "lon": 18.674343, "isBottle": False},
    {"lat": 50.270691, "lon": 18.674985, "isBottle": True},
    {"lat": 50.271031, "lon": 18.675091, "isBottle": False},
    {"lat": 50.270562, "lon": 18.675726, "isBottle": True},
    {"lat": 50.270901, "lon": 18.675839, "isBottle": False},
]

# Trasa przeszukiwania (W -> E, przez całe pole).
WP_A = {"lat": 50.271366, "lon": 18.672130}
WP_B = {"lat": 50.270614, "lon": 18.676456}

# Przybliżenie płaskiej Ziemi: 1 stopień ~ 111320 m (lon przeskalowane przez cos(lat)).
M_PER_DEG = 111320.0


def distance_m(lat1, lon1, lat2, lon2) -> float:
    dlat = (lat1 - lat2) * M_PER_DEG
    dlon = (lon1 - lon2) * M_PER_DEG * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def build_search_nav(a, b, alt, takeoff_alt) -> List[Dict]:
    return [
        {"command": "TAKEOFF", "alt": takeoff_alt},
        {"command": "WAYPOINT", "lat": a["lat"], "lon": a["lon"], "alt": alt, "acr": 10},
        {"command": "WAYPOINT", "lat": b["lat"], "lon": b["lon"], "alt": alt, "acr": 10},
    ]


def nearest_neighbor(start_lat, start_lon, targets) -> List[Dict]:
    remaining, ordered = list(targets), []
    cur_lat, cur_lon = start_lat, start_lon
    while remaining:
        nxt = min(remaining, key=lambda t: distance_m(cur_lat, cur_lon, t["lat"], t["lon"]))
        ordered.append(nxt); remaining.remove(nxt)
        cur_lat, cur_lon = nxt["lat"], nxt["lon"]
    return ordered


def build_visit_waypoints(ordered, alt, hover_time, base_seq, acr=5):
    wps, seq_to_led, seq = [], {}, base_seq
    for i, t in enumerate(ordered):
        wps.append({"command": "WAYPOINT", "lat": t["lat"], "lon": t["lon"], "alt": alt, "acr": acr})
        seq_to_led[seq] = i; seq += 1
        wps.append({"command": "NAV_LOITER_TIME", "lat": t["lat"], "lon": t["lon"],
                    "alt": alt, "time": hover_time, "radius": 0, "yaw": 0})
        seq += 1
    return wps, seq_to_led


def wait_item_reached(drone, target_seqs, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = drone.master.recv_match(type="MISSION_ITEM_REACHED", blocking=True, timeout=1.0)
        if msg and msg.seq in target_seqs:
            return msg.seq
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="udp:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--alt", type=float, default=50.0)
    ap.add_argument("--hover", type=float, default=8.0)
    ap.add_argument("--speed", type=float, default=5.0)
    args = ap.parse_args()

    print(f"[plan] {len(DIODES)} diod, A={WP_A}, B={WP_B}")
    drone = MatekService(device=args.device, baud=args.baud)
    try:
        # 1. misja SEARCH
        nav = build_search_nav(WP_A, WP_B, args.alt, takeoff_alt=args.alt)
        assert drone.set_waypoints(nav), "set_waypoints(SEARCH) nie powiodło się"
        b_seq = len(nav)            # home(0)+nav -> B ostatni = len(nav)
        base_seq = len(nav) + 1     # pierwszy WAYPOINT diody
        print(f"[mission] SEARCH wgrana. b_seq={b_seq}, base_seq={base_seq}")

        # 2. prędkość, 3. start AUTO
        drone.set_speed(args.speed)
        assert drone.start_mission(), "start_mission nie powiodło się"
        print("[mission] AUTO start, lecę do WP_B...")

        # 4. dolot do WP_B (zdarzenie)
        t0 = time.monotonic()
        if wait_item_reached(drone, {b_seq}, timeout=600) is None:
            print("[B] TIMEOUT – nie wykryto dolotu do WP_B")
        else:
            print(f"[B] osiągnięty po {time.monotonic()-t0:.1f}s")

        # 6+7. dołóż diody (pary WP+LOITER) i ustaw bieżący punkt
        ordered = nearest_neighbor(WP_B["lat"], WP_B["lon"], DIODES)
        visit, seq_to_led = build_visit_waypoints(ordered, args.alt, args.hover, base_seq)
        assert drone.append_waypoints(visit), "append_waypoints nie powiodło się"
        drone.set_current_waypoint(base_seq)
        print(f"[mission] dołożono {len(ordered)} diod; seq WP diod = {sorted(seq_to_led)}")

        # 9. odwiedzanie diod (zdarzenie) – tu byłaby akwizycja OOK
        remaining = set(seq_to_led)
        while remaining:
            seq = wait_item_reached(drone, remaining, timeout=args.hover + 180)
            if seq is None:
                print("[visit] TIMEOUT – brak kolejnego MISSION_ITEM_REACHED")
                break
            led_i = seq_to_led[seq]
            print(f"[CAPTURE] LED {led_i+1}/{len(ordered)} (seq={seq}) -> tu byłoby OOK")
            remaining.discard(seq)

        print("[done] sekwencja zakończona. Obserwuj zachowanie w Mission Planner.")
    finally:
        try:
            drone.disarm()
        except Exception:
            pass
        drone.close()


if __name__ == "__main__":
    main()