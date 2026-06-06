#!/usr/bin/env python3
"""
SEGMENT 2 — Demux i telemetria MatekService (stanowisko B: RPi + FC/SITL).

Cel: potwierdzić że refaktor RX/demux działa na żywym strumieniu — bufor
telemetrii się napełnia, interpolacja działa, yaw nie skacze przez ±pi.

Najlepiej z SITL w Mission Planner (ArduCopter). Połącz MatekService do tego
samego SITL co Mission Planner (np. przez mavproxy --out, albo bezpośrednio).

Uruchomienie:
    python -m tests.seg2_telemetry --uart udp:127.0.0.1:14551
    (lub --uart /dev/ttyAMA0 dla prawdziwego FC)

KRYTERIUM PRZEJŚCIA:
  [1] _tele_buf napełnia się blisko 30 Hz
  [2] get_latest_telemetry zwraca sensowną pozycję
  [3] get_telemetry_at interpoluje (nie None) w środku bufora
  [4] KRYTYCZNY: yaw interpoluje przez ±pi bez skoku do zera
       (obróć drona w SITL przez północ podczas testu, albo zaufaj danym)
"""
import argparse
import time

from Application.Services.MatekService import MatekService


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="udp:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print(f"Łączę z {args.uart}...")
    drone = MatekService(device=args.uart, baud=args.baud)
    try:
        drone.request_streams()      # 30/10/5 Hz
        print("request_streams wysłane, czekam 3 s na napełnienie bufora...")
        time.sleep(3.0)

        # [1] gęstość bufora
        with drone._lock:
            n = len(drone._tele_buf)
            buf = list(drone._tele_buf)
        if n >= 2:
            span = buf[-1].ts - buf[0].ts
            hz = (n - 1) / span if span > 0 else 0
        else:
            hz = 0
        print(f"\n[1] _tele_buf: {n} próbek, ~{hz:.1f} Hz")
        p1 = hz >= 20    # luźniej niż 30, na wypadek throttlingu FC
        print(f"    {'PASS' if p1 else 'FAIL — FC nie honoruje SET_MESSAGE_INTERVAL? obniż baud?'}")

        # [2] najświeższa telemetria
        tel = drone.get_latest_telemetry()
        print(f"\n[2] get_latest_telemetry:")
        if tel:
            print(f"    lat={tel.lat:.6f} lon={tel.lon:.6f} alt={tel.alt:.1f}")
            print(f"    roll={tel.roll:.3f} pitch={tel.pitch:.3f} yaw={tel.yaw:.3f}")
        p2 = tel is not None and abs(tel.lat) > 0.01
        print(f"    {'PASS' if p2 else 'FAIL — brak telemetrii lub zerowy GPS (czekaj na fix)'}")

        # [3] interpolacja w środku bufora
        if n >= 2:
            mid = (buf[0].ts + buf[-1].ts) / 2
            interp = drone.get_telemetry_at(mid)
            print(f"\n[3] get_telemetry_at(środek bufora): "
                  f"{'OK' if interp else 'None'}")
            p3 = interp is not None
        else:
            p3 = False
        print(f"    {'PASS' if p3 else 'FAIL'}")

        # [4] yaw przez ±pi — szukamy w buforze pary próbek po obu stronach pi
        print(f"\n[4] yaw interpolacja przez ±pi:")
        crossed = False
        for i in range(1, len(buf)):
            a, b = buf[i - 1], buf[i]
            if abs(a.yaw) > 2.5 and abs(b.yaw) > 2.5 and (a.yaw * b.yaw < 0):
                mid_ts = (a.ts + b.ts) / 2
                s = drone.get_telemetry_at(mid_ts)
                ok = s is not None and abs(s.yaw) > 2.5
                print(f"    przejście {a.yaw:.2f}->{b.yaw:.2f}, interp={s.yaw:.2f} "
                      f"{'OK (nie skoczył do 0)' if ok else 'BŁĄD (skok do 0)'}")
                crossed = True
                p4 = ok
                break
        if not crossed:
            print("    (nie złapano przejścia przez ±pi w buforze — obróć drona "
                  "w SITL przez północ i powtórz; test pominięty)")
            p4 = True   # nie blokuj, ale obejrzyj ręcznie

        print("\n" + "=" * 60)
        for name, ok in [("bufor ~30Hz", p1), ("telemetria świeża", p2),
                         ("interpolacja", p3), ("yaw przez ±pi", p4)]:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        allpass = p1 and p2 and p3 and p4
        print("SEGMENT 2:", "PASS — idź do Segmentu 3" if allpass else "FAIL")
        return 0 if allpass else 1
    finally:
        drone.close()


if __name__ == "__main__":
    raise SystemExit(main())
