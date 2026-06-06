#!/usr/bin/env python3
"""
SEGMENT 1 — Łączność MAVLink (stanowisko B: RPi + FC przez UART, BEZ SITL).

Cel: potwierdzić że RPi gada z FC, zanim dołożysz misję. NIE używa MatekService —
testuje surowe pymavlink, żeby odizolować problem sprzętowo-konfiguracyjny.

Uruchomienie:
    python -m tests.seg1_mavlink_link --uart /dev/ttyAMA0 --baud 115200

KRYTERIUM PRZEJŚCIA:
  [1] wait_heartbeat wraca w < 10 s
  [2] w strumieniu są HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT
  [3] brak masowego gubienia bajtów (errors < 5% wiadomości)

JEŚLI FAIL:
  - brak heartbeat -> zły port/baud, SERIALx_PROTOCOL != 2, złe okablowanie
  - brak ATTITUDE/GPS -> parametry SRx_* na porcie FC blokują strumienie
  - dużo błędów -> za wysoki baud, brak flow control, zakłócenia na kablu
"""
import argparse
import time
from collections import Counter

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=15, help="ile sekund nasłuchiwać")
    args = ap.parse_args()

    print(f"Łączę z {args.uart} @ {args.baud}...")
    master = mavutil.mavlink_connection(args.uart, baud=args.baud)

    # [1] heartbeat
    t0 = time.time()
    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        print("[FAIL] brak HEARTBEAT w 10 s — sprawdź port/baud/SERIALx_PROTOCOL/kabel")
        return 1
    dt = time.time() - t0
    print(f"[1] HEARTBEAT po {dt:.1f}s — system {master.target_system}, "
          f"komponent {master.target_component}  [PASS]")

    # [2] + [3] nasłuch i statystyki
    print(f"\n[2] Nasłuch {args.seconds}s — liczę typy wiadomości i częstotliwości...")
    counts = Counter()
    errors_before = getattr(master.mav, "total_receive_errors", 0)
    end = time.time() + args.seconds
    total = 0
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        counts[msg.get_type()] += 1
        total += 1

    print(f"\n    Odebrano {total} wiadomości w {args.seconds}s:")
    for t, c in counts.most_common():
        hz = c / args.seconds
        print(f"      {t:<28} {c:>5}  ({hz:.1f} Hz)")

    have_hb  = counts["HEARTBEAT"] > 0
    have_att = counts["ATTITUDE"] > 0
    have_gps = counts["GLOBAL_POSITION_INT"] > 0
    print(f"\n    HEARTBEAT: {'OK' if have_hb else 'BRAK'}")
    print(f"    ATTITUDE:  {'OK' if have_att else 'BRAK — żądaj SET_MESSAGE_INTERVAL lub sprawdź SRx_EXTRA1'}")
    print(f"    GPS:       {'OK' if have_gps else 'BRAK — sprawdź SRx_POSITION i fix GPS'}")

    p2 = have_hb and have_att and have_gps

    # [3] błędy parsowania
    errors_after = getattr(master.mav, "total_receive_errors", 0)
    err_delta = errors_after - errors_before
    err_ratio = err_delta / max(1, total)
    print(f"\n[3] Błędy parsowania: {err_delta} ({err_ratio*100:.1f}% wiadomości)")
    p3 = err_ratio < 0.05

    print("\n" + "=" * 60)
    print(f"  [{'PASS' if p2 else 'FAIL'}] obecne HEARTBEAT+ATTITUDE+GPS")
    print(f"  [{'PASS' if p3 else 'FAIL'}] błędy < 5%")
    allpass = p2 and p3
    print("SEGMENT 1:", "PASS — idź do Segmentu 2" if allpass else "FAIL — napraw łączność")
    return 0 if allpass else 1


if __name__ == "__main__":
    raise SystemExit(main())
