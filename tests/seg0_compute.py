#!/usr/bin/env python3
"""
SEGMENT 0 — Warstwa obliczeniowa (stanowisko A: samo RPi, bez FC/kamery).

Cel: potwierdzić że matematyka shared/ działa na DOCELOWYM RPi5 z prawdziwymi
wersjami numpy/opencv (mogą różnić się od piaskownicy → inne wyniki numeryczne).

Uruchomienie (z katalogu głównego projektu):
    python -m tests.seg0_compute

KRYTERIUM PRZEJŚCIA:
  [1] classify_ook wykrywa 10/10 częstotliwości na syntetyku
  [2] round-trip pixel->GPS->pixel < 0.1 px
  [3] klastrowanie zwraca 4/4 diody przy szumie 1-2.5 m
Wszystkie [PASS] → idź do Segmentu 1 (lub 5 równolegle).
"""
import sys
import math
import numpy as np

# wersje bibliotek — zapisz je, porównaj z piaskownicą jeśli wyniki się różnią
import cv2
print(f"numpy {np.__version__}, opencv {cv2.__version__}, python {sys.version.split()[0]}")
print("=" * 60)

from shared.ook import classify_ook
from shared.geometry import pixel_to_gps, gps_to_pixel
from shared.calibration import K, DIST, CALIB_W, CALIB_H, IMG_CENTER
from shared.clustering import TargetClusterer, distance_m
from shared.protocol import CANDIDATE_FREQS

results = []

# ─── TEST 1: classify_ook na syntetyku z jitterem + dropami + szumem ───
print("[1] classify_ook — 10 częstotliwości pod jitterem/dropami/szumem")
rng = np.random.default_rng(0)
ok_count = 0
for f_true in CANDIDATE_FREQS:
    fps = 120
    n = int(fps * 3.0)
    # nieregularne znaczniki czasu (jitter) + losowe dropnięcia 30%
    t = np.cumsum(rng.uniform(0.5, 1.5, n) / fps)
    keep = rng.random(n) > 0.30
    t = t[keep]
    bright = np.where(((f_true * t) % 1) < 0.5, 120.0, 0.0)
    bright += rng.normal(0, 10, len(t))          # szum
    bright = np.clip(bright, 0, None)
    f_det, conf = classify_ook(bright, t, CANDIDATE_FREQS)
    hit = (f_det is not None and round(f_det) == f_true)
    ok_count += hit
    print(f"    {f_true:>2} Hz -> {f_det} (conf {conf:.1f}) {'OK' if hit else 'BŁĄD'}")
# kontrola: czysty szum musi dać None
noise = rng.normal(50, 15, 300)
tn = np.cumsum(rng.uniform(0.5, 1.5, 300) / 120)
f_noise, c_noise = classify_ook(noise, tn, CANDIDATE_FREQS)
print(f"    szum  -> {f_noise} (conf {c_noise:.1f}) {'OK' if f_noise is None else 'BŁĄD'}")
p1 = (ok_count == len(CANDIDATE_FREQS) and f_noise is None)
results.append(("classify_ook 10/10 + szum=None", p1))

# ─── TEST 2: round-trip pixel -> GPS -> pixel ───
print("\n[2] round-trip pixel->GPS->pixel")
lat_uav, lon_uav, alt = 50.2710, 18.6730, 50.0
max_err = 0.0
for (px, py) in [(640, 400), (300, 250), (900, 550), (700, 200)]:
    gps = pixel_to_gps(px, py, lat_uav, lon_uav, alt, 0.05, -0.03, 1.2,
                       K, DIST, CALIB_W, CALIB_H)
    if gps is None:
        continue
    back = gps_to_pixel(gps[0], gps[1], lat_uav, lon_uav, alt, 0.05, -0.03, 1.2,
                        K, DIST, CALIB_W, CALIB_H)
    if back is None:
        continue
    err = math.hypot(back[0] - px, back[1] - py)
    max_err = max(max_err, err)
    print(f"    ({px},{py}) -> GPS -> ({back[0]:.2f},{back[1]:.2f})  błąd {err:.3f}px")
p2 = max_err < 0.1
results.append((f"round-trip < 0.1px (max {max_err:.3f})", p2))

# ─── TEST 3: klastrowanie przy realnym szumie ───
print("\n[3] klastrowanie — 4 diody, szum 1-2.5 m")
true_diodes = [(50.27110, 18.67276), (50.27095, 18.67350),
               (50.27082, 18.67424), (50.27069, 18.67499)]
M = 111320.0
clu = TargetClusterer()
for (lat, lon) in true_diodes:
    for _ in range(60):
        opt = rng.uniform(0, 400)
        sigma = 0.5 + opt / 200.0
        jlat = lat + rng.normal(0, sigma / M)
        jlon = lon + rng.normal(0, sigma / (M * math.cos(math.radians(lat))))
        clu.add(jlat, jlon, opt)
det = clu.finalize(min_frames=5, min_frame_ratio=0.25)
errs = []
for d in det:
    best = min(true_diodes, key=lambda t: distance_m(d['lat'], d['lon'], t[0], t[1]))
    e = distance_m(d['lat'], d['lon'], best[0], best[1])
    errs.append(e)
    print(f"    ({d['lat']:.6f},{d['lon']:.6f}) n={d['n_frames']} błąd={e:.2f}m")
p3 = (len(det) == 4 and max(errs, default=99) < 1.0)
results.append((f"klastrowanie 4/4 < 1m (wykryto {len(det)})", p3))

# ─── PODSUMOWANIE ───
print("\n" + "=" * 60)
allpass = True
for name, ok in results:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    allpass &= ok
print("=" * 60)
print("SEGMENT 0:", "PASS — idź do Segmentu 1/5" if allpass else "FAIL — napraw zanim ruszysz dalej")
sys.exit(0 if allpass else 1)
