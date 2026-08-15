#!/usr/bin/env python3
"""
tune_enhance.py
================
Strojenie parametrów post-processingu na POJEDYNCZEJ klatce, z porównaniem
wariantów obok siebie w powiększeniu 1:1 (crop, nie downscale - inaczej
różnic w ostrości/szumie w ogóle nie widać).

Sens: zamiast przetwarzać 1600 klatek na chybił trafił, wybierasz ustawienia
na jednej, a dopiero potem puszczasz batch.

Testowane osie (do wyboru przez --mode):
  gain     - różne poziomy przycięcia gain mapy (naprawa przebarwień)
  sharpen  - różne siły wyostrzania (przeciw resztkowemu rozmyciu ruchu)
  denoise  - różne warianty odszumiania
  combo    - kilka sensownych kombinacji wszystkiego naraz

UŻYCIE:
  # Zobacz warianty przycięcia gain mapy
  python3 tune_enhance.py <PLIK.raw> --mode gain

  # Zobacz warianty wyostrzania (na już przyciętej gain mapie)
  python3 tune_enhance.py <PLIK.raw> --mode sharpen --gain-clip 1.5

  # Kombinacje
  python3 tune_enhance.py <PLIK.raw> --mode combo --gain-clip 1.5

  # Inny wycinek kadru (domyślnie środek)
  python3 tune_enhance.py <PLIK.raw> --mode combo --crop-x 200 --crop-y 800
"""

import argparse
import os
import sys

import cv2
import numpy as np

WIDTH = 1920
HEIGHT = 1200
CROP_W = 620
CROP_H = 460


def load_isp(isp_repo):
    process_dir = os.path.join(os.path.expanduser(isp_repo), "process")
    if not os.path.isdir(process_dir):
        raise FileNotFoundError(f"Nie znalazłem {process_dir} - sprawdź --isp-repo")
    sys.path.insert(0, process_dir)
    import process_raw  # type: ignore
    return process_raw


def normalize_gain_per_channel(gain_map):
    """
    Normalizuje każdy podkanał Bayera (G1, R, B, G2) jego własną średnią.

    PROBLEM, KTÓRY TO ROZWIĄZUJE:
    build_calibration.py normalizuje flat jedną wspólną średnią ze wszystkich
    pikseli. Jeśli źródło światła użyte do flatów nie było neutralne barwnie
    (a latarka LED nie jest), kanały R/G/B mają w flacie różne poziomy średnie
    - i po podzieleniu przez wspólną średnią gain map dostaje wbudowane, stałe
    przesunięcie kolorów: odwrócony balans bieli latarki, narzucany potem
    każdemu zdjęciu.

    Normalizacja per-kanałowa sprowadza średni gain każdego kanału do 1.0
    (zero przesunięcia kolorów), zachowując przy tym przestrzenną zmienność
    w obrębie kanału - czyli faktyczną korekcję winietowania, o którą chodzi.
    """
    out = gain_map.copy()
    for ys, xs in [(slice(0, None, 2), slice(0, None, 2)),   # G1
                   (slice(0, None, 2), slice(1, None, 2)),   # R
                   (slice(1, None, 2), slice(0, None, 2)),   # B
                   (slice(1, None, 2), slice(1, None, 2))]:  # G2
        ch = out[ys, xs]
        m = ch.mean()
        if m > 1e-6:
            out[ys, xs] = ch / m
    return out


def apply_calibration_clipped(raw10, master_dark, gain_map, clip_hi=None, clip_lo=None):
    """
    Kalibracja z opcjonalnym przycięciem gain mapy.

    Przebarwienia (fioletowe/różowe plamy) biorą się z ekstremalnych wartości
    w gain mapie: skoro mapa działa per-piksel na mozaice Bayera, lokalnie
    zawyżona wartość wzmacnia jeden kanał koloru mocniej niż sąsiednie, co po
    debayerze daje plamę koloru. Przycięcie ogranicza to kosztem części
    korekcji winietowania.
    """
    calibrated = raw10 - master_dark
    calibrated = np.clip(calibrated, 0.0, None)

    gm = gain_map
    if clip_hi is not None or clip_lo is not None:
        lo = clip_lo if clip_lo is not None else gain_map.min()
        hi = clip_hi if clip_hi is not None else gain_map.max()
        gm = np.clip(gain_map, lo, hi)

    calibrated = calibrated * gm
    return np.clip(calibrated, 0.0, 1023.0)


def unsharp_mask(bgr8, amount=1.0, radius=2.0, threshold=0):
    """
    Wyostrzanie przez unsharp mask - odejmuje rozmytą wersję od oryginału,
    podbijając krawędzie.

    UWAGA co do rozmycia ruchu: unsharp mask podbija kontrast krawędzi, ale
    NIE odwraca rozmycia kierunkowego - informacja utracona podczas ekspozycji
    nie wraca. Poprawia subiektywną ostrość, przy zbyt dużym 'amount' tworzy
    halo wokół krawędzi i wzmacnia szum.
    """
    blurred = cv2.GaussianBlur(bgr8, (0, 0), radius)
    sharpened = cv2.addWeighted(bgr8, 1.0 + amount, blurred, -amount, 0)
    if threshold > 0:
        low_contrast = np.abs(bgr8.astype(np.int16) - blurred.astype(np.int16)) < threshold
        sharpened[low_contrast] = bgr8[low_contrast]
    return sharpened


def process_variant(pr, raw_path, master_dark, gain_map,
                    gain_clip=None, sharpen=0.0, sharpen_radius=3.0,
                    denoise_method=None, denoise_strength=7,
                    target_mean=0.15, sat_boost=1.35, bayer_code="GB",
                    per_channel_norm=False, no_gain=False):
    raw10 = pr.load_raw10(raw_path)

    gm = gain_map
    if per_channel_norm:
        gm = normalize_gain_per_channel(gm)
    if no_gain:
        gm = np.ones_like(gain_map)

    raw10 = apply_calibration_clipped(raw10, master_dark, gm,
                                       clip_hi=gain_clip,
                                       clip_lo=(1.0 / gain_clip) if gain_clip else None)
    bgr16 = pr.debayer(raw10, bayer_code)
    bgr8 = pr.simple_awb_gamma(bgr16, target_mean=target_mean, sat_boost=sat_boost)

    if denoise_method:
        bgr8 = pr.denoise(bgr8, method=denoise_method, strength=denoise_strength)
    if sharpen > 0:
        # Jeśli process_raw.py ma już własny sharpen (wersja z unsharp mask),
        # używamy jego - żeby strojenie odpowiadało dokładnie temu, co robi
        # produkcyjny pipeline. Fallback na lokalną implementację (identyczna
        # formuła) dla starszych wersji repo.
        if hasattr(pr, "sharpen"):
            bgr8 = pr.sharpen(bgr8, amount=sharpen, radius=sharpen_radius)
        else:
            bgr8 = unsharp_mask(bgr8, amount=sharpen, radius=sharpen_radius)

    return bgr8


def crop(img, x, y, w=CROP_W, h=CROP_H):
    x = max(0, min(x, img.shape[1] - w))
    y = max(0, min(y, img.shape[0] - h))
    return img[y:y + h, x:x + w].copy()


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return out


def build_grid(tiles, cols=2):
    rows = []
    for i in range(0, len(tiles), cols):
        chunk = tiles[i:i + cols]
        while len(chunk) < cols:
            chunk.append(np.zeros_like(tiles[0]))
        rows.append(np.hstack(chunk))
    return np.vstack(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw_file", help="Ścieżka do pliku .raw")
    parser.add_argument("--isp-repo", default="~/isp_npp/Antek_isp/iza/ar0234_isp")
    parser.add_argument("--mode", choices=["gain", "gainfix", "gainfix2", "sharpen", "denoise", "combo"],
                         default="combo")
    parser.add_argument("--per-channel-norm", action="store_true",
                         help="Normalizuj gain mapę per kanał Bayera (usuwa color cast od "
                              "nieneutralnego oświetlenia flatów) - dotyczy trybów sharpen/denoise/combo")
    parser.add_argument("--gain-clip", type=float, default=None,
                         help="Górny limit gain mapy używany w trybach sharpen/denoise/combo")
    parser.add_argument("--crop-x", type=int, default=None, help="Lewy górny róg wycinka (domyślnie środek kadru)")
    parser.add_argument("--crop-y", type=int, default=None)
    parser.add_argument("--target-mean", type=float, default=0.15,
                         help="Docelowa srednia jasnosc PRZED gamma (0.15 ~ 0.42 po gammie 1/2.2)")
    parser.add_argument("--sat-boost", type=float, default=1.35)
    parser.add_argument("--bayer-code", default="GB")
    parser.add_argument("-o", "--output", default=None, help="Ścieżka wynikowego .png")
    args = parser.parse_args()

    pr = load_isp(args.isp_repo)
    master_dark, gain_map = pr.load_calibration()
    print(f"[info] gain_map oryginalna -> min={gain_map.min():.3f} max={gain_map.max():.3f} "
          f"mean={gain_map.mean():.3f}")

    cx = args.crop_x if args.crop_x is not None else (WIDTH - CROP_W) // 2
    cy = args.crop_y if args.crop_y is not None else (HEIGHT - CROP_H) // 2
    print(f"[info] Wycinek 1:1 od ({cx}, {cy}), rozmiar {CROP_W}x{CROP_H}")

    base = dict(target_mean=args.target_mean, sat_boost=args.sat_boost, bayer_code=args.bayer_code)
    variants = []

    if args.mode == "gain":
        variants = [
            ("bez przyciecia (oryginal)", dict(gain_clip=None)),
            ("clip 2.5", dict(gain_clip=2.5)),
            ("clip 1.5", dict(gain_clip=1.5)),
            ("clip 1.2", dict(gain_clip=1.2)),
        ]
    elif args.mode == "gainfix":
        variants = [
            ("oryginal (globalna norm.)", dict(gain_clip=None)),
            ("clip 1.2 (agresywne ciecie)", dict(gain_clip=1.2)),
            ("norm. per kanal Bayera", dict(per_channel_norm=True)),
            ("bez gain mapy (tylko dark)", dict(no_gain=True)),
        ]
    elif args.mode == "gainfix2":
        variants = [
            ("clip 1.2 (dotychczasowy faworyt)", dict(gain_clip=1.2)),
            ("per kanal + clip 2.0", dict(per_channel_norm=True, gain_clip=2.0)),
            ("per kanal + clip 1.5", dict(per_channel_norm=True, gain_clip=1.5)),
            ("per kanal + clip 1.3", dict(per_channel_norm=True, gain_clip=1.3)),
        ]
    elif args.mode == "sharpen":
        gc = args.gain_clip
        pcn = args.per_channel_norm
        variants = [
            ("bez wyostrzania", dict(gain_clip=gc, per_channel_norm=pcn, sharpen=0.0)),
            ("sharpen 0.5 r3", dict(gain_clip=gc, per_channel_norm=pcn, sharpen=0.5, sharpen_radius=3)),
            ("sharpen 1.0 r3", dict(gain_clip=gc, per_channel_norm=pcn, sharpen=1.0, sharpen_radius=3)),
            ("sharpen 1.5 r3", dict(gain_clip=gc, per_channel_norm=pcn, sharpen=1.5, sharpen_radius=3)),
        ]
    elif args.mode == "denoise":
        gc = args.gain_clip
        variants = [
            ("bez denoise", dict(gain_clip=gc)),
            ("bilateral s5", dict(gain_clip=gc, denoise_method="bilateral", denoise_strength=5)),
            ("nlm s4", dict(gain_clip=gc, denoise_method="nlm", denoise_strength=4)),
            ("nlm s8", dict(gain_clip=gc, denoise_method="nlm", denoise_strength=8)),
        ]
    else:  # combo
        gc = args.gain_clip
        variants = [
            ("A: baseline (bez zmian)", dict(gain_clip=None)),
            ("B: clip + sharpen 1.0", dict(gain_clip=gc or 1.5, sharpen=1.0)),
            ("C: clip + nlm4 + sharpen 1.0",
             dict(gain_clip=gc or 1.5, denoise_method="nlm", denoise_strength=4, sharpen=1.0)),
            ("D: clip + bilat5 + sharpen 1.5",
             dict(gain_clip=gc or 1.5, denoise_method="bilateral", denoise_strength=5,
                  sharpen=1.5, sharpen_radius=3.0)),
        ]

    tiles = []
    for name, kwargs in variants:
        print(f"[info] Wariant: {name}")
        merged = dict(base)
        merged.update(kwargs)
        img = process_variant(pr, args.raw_file, master_dark, gain_map, **merged)
        tiles.append(label(crop(img, cx, cy), name))

    grid = build_grid(tiles, cols=2)

    out_path = args.output
    if out_path is None:
        stem = os.path.splitext(os.path.basename(args.raw_file))[0]
        out_path = f"{stem}_tune_{args.mode}.png"
    cv2.imwrite(out_path, grid)
    print(f"\n[done] Zapisano: {out_path}")
    print("Obejrzyj w 100% i wybierz wariant. Parametry zwycięzcy przekaż do batcha.")


if __name__ == "__main__":
    main()
