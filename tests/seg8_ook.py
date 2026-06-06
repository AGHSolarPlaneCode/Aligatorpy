#!/usr/bin/env python3
"""
SEGMENT 8 — Dekodowanie OOK (stanowisko C: kamera poziomo, MIGAJĄCA dioda).

Cel: potwierdzić że Oko odczytuje częstotliwość migania prawdziwej diody.
Testuje decode_ook: lock-on, lepkie ROI, ook_brightness, classify_ook na
realnym sygnale z kamery 120fps.

WYMAGA: dioda migająca ze ZNANĄ częstotliwością (generator/Arduino/sygnał z FC).

Uruchomienie:
    python -m tests.seg8_ook --expected 12 --threshold 220

Co robić:
  - ustaw diodę migającą np. 12 Hz w centralnej części kadru
  - HINT podajemy jako środek kadru (dioda powinna tam być)
  - sprawdź czy zwrócona freq == expected i jaki confidence
  - powtórz dla kilku częstotliwości {4,6,12,16}
  - test lepkiego ROI: lekko poruszaj diodą podczas pomiaru

KRYTERIUM PRZEJŚCIA:
  [1] zwrócona freq == expected
  [2] confidence wyraźnie > MIN_OOK_CONFIDENCE (i > confidence dla zgaszonej)
  [3] ROI śledzi poruszającą się diodę (freq nadal wykrywana)

Po przejściu kilku częstotliwości: ustaw MIN_OOK_CONFIDENCE między confidence
szumu a confidence sygnału. Wpisz dostrojone progi do protocol.py.
"""
import argparse
import threading
import time

from vision.camera_handler import CameraPipeline, WIDTH, HEIGHT
from vision.eye import decode_ook
from shared.protocol import MIN_OOK_CONFIDENCE, OOK_THRESHOLD


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expected", type=float, required=True,
                    help="znana częstotliwość migania diody [Hz]")
    ap.add_argument("--hint-x", type=float, default=WIDTH/2)
    ap.add_argument("--hint-y", type=float, default=HEIGHT/2)
    ap.add_argument("--repeats", type=int, default=3)
    args = ap.parse_args()

    cam = CameraPipeline()
    threading.Thread(target=cam.start, daemon=True).start()
    time.sleep(2.0)

    print(f"Dekodowanie OOK, oczekiwane {args.expected} Hz, "
          f"HINT=({args.hint_x:.0f},{args.hint_y:.0f}), próg={OOK_THRESHOLD}")
    print(f"MIN_OOK_CONFIDENCE = {MIN_OOK_CONFIDENCE}\n")

    cam.set_decode_active(True)
    time.sleep(0.3)

    correct = 0
    confs = []
    for r in range(1, args.repeats + 1):
        print(f"--- próba {r}/{args.repeats} (zbieram 3 s) ---")
        res = decode_ook(cam, args.hint_x, args.hint_y)
        hit = (res.freq_hz is not None and round(res.freq_hz) == round(args.expected))
        correct += hit
        confs.append(res.confidence)
        print(f"    freq = {res.freq_hz} Hz, confidence = {res.confidence:.1f}")
        print(f"    avg_pixel = ({res.avg_raw_x:.1f}, {res.avg_raw_y:.1f})")
        print(f"    {'OK' if hit else 'BŁĄD — zła lub brak częstotliwości'}\n")

    cam.set_decode_active(False)

    # pomiar szumu (dioda powinna być teraz zgaszona — poproś użytkownika)
    print("--- pomiar szumu: ZGAŚ diodę i naciśnij Enter (lub Ctrl+C by pominąć) ---")
    try:
        input()
        cam.set_decode_active(True); time.sleep(0.3)
        noise = decode_ook(cam, args.hint_x, args.hint_y)
        cam.set_decode_active(False)
        print(f"    szum: freq={noise.freq_hz}, confidence={noise.confidence:.1f}")
        noise_conf = noise.confidence
    except (KeyboardInterrupt, EOFError):
        noise_conf = None
        print("    (pominięto)")

    cam.stop()
    print("\n" + "=" * 60)
    import statistics
    avg_conf = statistics.mean(confs) if confs else 0
    p1 = correct == args.repeats
    p2 = avg_conf > MIN_OOK_CONFIDENCE and (noise_conf is None or avg_conf > noise_conf * 2)
    print(f"  [{'PASS' if p1 else 'FAIL'}] freq poprawna {correct}/{args.repeats}")
    print(f"  [{'PASS' if p2 else 'FAIL'}] confidence sygnału {avg_conf:.1f} "
          f"{'> szum' if noise_conf else ''}"
          f"{f' ({noise_conf:.1f})' if noise_conf else ''}")
    if noise_conf is not None:
        sug = (avg_conf + noise_conf) / 2
        print(f"\n  Sugerowany MIN_OOK_CONFIDENCE: ~{sug:.1f} "
              f"(między szumem {noise_conf:.1f} a sygnałem {avg_conf:.1f})")
    print("SEGMENT 8:", "PASS — idź do Segmentu 9" if (p1 and p2) else "FAIL")
    return 0 if (p1 and p2) else 1


if __name__ == "__main__":
    raise SystemExit(main())
