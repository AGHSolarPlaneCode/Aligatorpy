#!/usr/bin/env python3
"""
SEGMENT 5 — Pipeline kamery surowo (stanowisko C: RPi + kamera, BEZ FC).

Cel: potwierdzić że GStreamer startuje, kamera daje klatki, przełączanie gałęzi
działa, znaczniki czasu są sensowne. KLUCZOWY produkt: zmierzony CLOCK_OFFSET.

Uruchomienie (na RPi z kamerą OV9281):
    python -m tests.seg5_camera_raw

KRYTERIUM PRZEJŚCIA:
  [1] pipeline startuje bez błędów GStreamera
  [2] get_image zwraca klatki 1280x800, ts monotonicznie rosnący
  [3] gałąź search ~30fps, decode ~120fps (zmierzone unikalne ts/s)
  [4] CLOCK_OFFSET stabilny między pomiarami -> WPISZ DO protocol.py

UWAGA: ten skrypt potrzebuje GStreamer + libcamera + gi. Na maszynie bez kamery
nie zadziała — uruchamiaj WYŁĄCZNIE na docelowym RPi.
"""
import threading
import time

from vision.camera_handler import CameraPipeline, WIDTH, HEIGHT


def measure_fps(cam, seconds=3.0):
    """Liczy unikalne znaczniki czasu na sekundę."""
    seen = set()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        f, ts, err = cam.get_image()
        if ts is not None:
            seen.add(ts)
        time.sleep(0.001)
    return len(seen) / seconds


def main():
    print("Tworzę pipeline...")
    cam = CameraPipeline()
    th = threading.Thread(target=cam.start, daemon=True)
    th.start()
    time.sleep(2.0)   # rozruch

    results = []

    # [1] + [2] gałąź search
    print("\n[1+2] Aktywuję SEARCH, sprawdzam klatki...")
    cam.set_search_active(True)
    time.sleep(0.5)
    f, ts, err = cam.get_image()
    if f is None:
        print(f"    [FAIL] brak klatki: {err}")
        cam.stop(); return 1
    print(f"    klatka: {f.shape} (size={f.size}, oczekiwane {WIDTH*HEIGHT}), ts={ts:.3f}")
    p_size = (f.size >= WIDTH * HEIGHT)
    results.append(("rozmiar klatki", p_size))

    # ts rośnie?
    ts1 = ts
    time.sleep(0.2)
    _, ts2, _ = cam.get_image()
    p_mono = (ts2 is not None and ts2 >= ts1)
    print(f"    ts rośnie: {ts1:.3f} -> {ts2:.3f}  {'OK' if p_mono else 'FAIL'}")
    results.append(("ts monotoniczny", p_mono))

    # [3] fps obu gałęzi
    print("\n[3] Pomiar fps...")
    fps_search = measure_fps(cam, 3.0)
    print(f"    SEARCH: {fps_search:.1f} fps (oczekiwane ~30)")
    cam.set_decode_active(True)
    time.sleep(0.3)
    fps_decode = measure_fps(cam, 3.0)
    print(f"    DECODE: {fps_decode:.1f} fps (oczekiwane ~120)")
    p_fps = (20 <= fps_search <= 40 and fps_decode >= 80)
    results.append(("fps search~30 decode~120", p_fps))
    cam.set_decode_active(False)
    cam.set_search_active(True)

    # [4] CLOCK_OFFSET — różnica ts klatki vs time.monotonic()
    print("\n[4] Pomiar CLOCK_OFFSET (ts klatki vs time.monotonic)...")
    offsets = []
    for _ in range(10):
        f, ts, err = cam.get_image()
        now = time.monotonic()
        if ts is not None:
            offsets.append(now - ts)
        time.sleep(0.1)
    if offsets:
        import statistics
        mean_off = statistics.mean(offsets)
        stdev = statistics.pstdev(offsets)
        print(f"    offset = {mean_off:.4f} s (odchylenie {stdev:.4f} s)")
        print(f"    ===> WPISZ DO shared/protocol.py: CLOCK_OFFSET = {-mean_off:.4f}")
        print(f"    (znak: Mózg robi get_telemetry_at(frame_ts + CLOCK_OFFSET);")
        print(f"     jeśli ts klatki jest 'starsze' niż monotonic, offset dodatni)")
        p_offset = stdev < 0.02   # stabilny
    else:
        p_offset = False
    results.append(("CLOCK_OFFSET stabilny", p_offset))

    cam.stop()
    print("\n" + "=" * 60)
    allpass = True
    for name, ok in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        allpass &= ok
    print("SEGMENT 5:", "PASS — wpisz CLOCK_OFFSET, idź do Segmentu 6" if allpass else "FAIL")
    return 0 if allpass else 1


if __name__ == "__main__":
    raise SystemExit(main())
