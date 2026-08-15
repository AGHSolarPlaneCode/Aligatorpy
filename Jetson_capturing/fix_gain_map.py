#!/usr/bin/env python3
"""
fix_gain_map.py
================
Jednorazowa naprawa istniejącego pliku gain_map.bin - bez potrzeby ponownego
zbierania klatek kalibracyjnych.

DWA PROBLEMY, KTÓRE NAPRAWIA:

1. COLOR CAST OD NIENEUTRALNEGO OŚWIETLENIA FLATÓW
   build_calibration.py normalizuje flat jedną wspólną średnią ze wszystkich
   pikseli. Jeśli latarka użyta do flatów nie była neutralna barwnie (a nie
   jest), kanały R/G/B mają w flacie różne poziomy średnie - gain map dostaje
   wtedy wbudowany, stały odwrócony balans bieli latarki i narzuca go każdemu
   zdjęciu. Objaw: fioletowo-magentowe zabarwienie całego kadru.

   Naprawa: normalizacja każdego podkanału Bayera (G1, R, B, G2) jego własną
   średnią - średni gain każdego kanału = 1.0 (zero przesunięcia kolorów),
   przy zachowaniu przestrzennej zmienności w obrębie kanału (winietowanie).

2. LOKALNE EKSTREMA
   Pojedyncze piksele o skrajnych wartościach gain (u nas max 5.35 przy
   typowym winietowaniu rzędu 2-3x) dają lokalne przebarwienia i podbijają
   szum. Naprawa: przycięcie do rozsądnego zakresu.

Oryginał zapisywany jest jako gain_map.bin.bak.

UŻYCIE:
    python3 fix_gain_map.py /sciezka/do/ar0234_isp/calib/gain_map.bin
    python3 fix_gain_map.py /sciezka/do/ar0234_isp/calib/gain_map.bin --clip 1.5
    python3 fix_gain_map.py <plik> --dry-run    # tylko pokaz statystyki
"""

import argparse
import os
import shutil
import sys

import numpy as np

WIDTH = 1920
HEIGHT = 1200
PIXELS = WIDTH * HEIGHT


def normalize_per_channel(gain_map):
    out = gain_map.copy()
    stats = []
    for name, (ys, xs) in [("G1", (slice(0, None, 2), slice(0, None, 2))),
                            ("R", (slice(0, None, 2), slice(1, None, 2))),
                            ("B", (slice(1, None, 2), slice(0, None, 2))),
                            ("G2", (slice(1, None, 2), slice(1, None, 2)))]:
        ch = out[ys, xs]
        m = ch.mean()
        stats.append((name, m))
        if m > 1e-6:
            out[ys, xs] = ch / m
    return out, stats


def describe(gm, label):
    print(f"{label}: min={gm.min():.4f} max={gm.max():.4f} mean={gm.mean():.4f} "
          f"p99={np.percentile(gm, 99):.4f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("gain_map", help="Ścieżka do gain_map.bin")
    parser.add_argument("--clip", type=float, default=1.5,
                         help="Górny limit gain (dolny = 1/clip). 0 = bez przycinania. Domyślnie 1.5")
    parser.add_argument("--no-per-channel", action="store_true",
                         help="Pomiń normalizację per kanał (tylko przytnij)")
    parser.add_argument("--dry-run", action="store_true", help="Pokaż statystyki, nie zapisuj")
    args = parser.parse_args()

    path = os.path.expanduser(args.gain_map)
    if not os.path.exists(path):
        print(f"[error] Nie znaleziono: {path}")
        sys.exit(1)

    gm = np.fromfile(path, dtype=np.float32)
    if gm.size != PIXELS:
        print(f"[error] Zły rozmiar pliku: {gm.size} wartości, oczekiwano {PIXELS}")
        sys.exit(1)
    gm = gm.reshape((HEIGHT, WIDTH))

    describe(gm, "PRZED         ")

    if not args.no_per_channel:
        gm, stats = normalize_per_channel(gm)
        means = ", ".join(f"{n}={m:.4f}" for n, m in stats)
        print(f"[info] Średnie kanałów przed normalizacją: {means}")
        print(f"[info] Rozrzut między kanałami: "
              f"{max(m for _, m in stats) / min(m for _, m in stats):.3f}x "
              f"(1.0 = brak color castu)")
        describe(gm, "po norm.      ")

    if args.clip > 0:
        lo, hi = 1.0 / args.clip, args.clip
        clipped_count = int(((gm < lo) | (gm > hi)).sum())
        gm = np.clip(gm, lo, hi)
        print(f"[info] Przycięto do [{lo:.3f}, {hi:.3f}] - "
              f"{clipped_count} pikseli ({clipped_count / PIXELS * 100:.2f}%)")
        describe(gm, "po przycieciu ", )

    if args.dry_run:
        print("\n[dry-run] Nic nie zapisano.")
        return

    backup = path + ".bak"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
        print(f"[info] Kopia oryginału: {backup}")
    else:
        print(f"[info] Kopia już istnieje, nie nadpisuję: {backup}")

    gm.astype(np.float32).tofile(path)
    print(f"[done] Zapisano naprawioną mapę: {path}")
    print("Cały istniejący pipeline (process_raw.py, debayer_flight_frames.py) "
          "użyje jej teraz bez żadnych zmian w kodzie.")


if __name__ == "__main__":
    main()
