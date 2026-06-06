# Testy misji Droniada

Testy jednostkowe i skrypty live (wymagają sprzętu) dla pipeline detekcji LED, OOK i planowania misji.

## Uruchomienie testów jednostkowych (bez sprzętu)

```bash
python -m unittest discover -s Droniada_tests/unit -v
```

## Testy live (RPi + kamera / MAVLink)

```bash
# Detekcja diod — wypisuje każdą wykrytą diodę
python Droniada_tests/live/live_led_detection.py

# Telemetria GPS + attitude (opcjonalnie z kamerą w tle)
python Droniada_tests/live/live_telemetry.py
python Droniada_tests/live/live_telemetry.py --with-camera

# OOK — 10s próbkowanie @ 120fps
python Droniada_tests/live/live_ook_detection.py
```

## Wymagania

- Testy jednostkowe: `numpy`, `opencv-python` (cv2)
- Testy live: Raspberry Pi z kamerą OV9281, GStreamer, połączenie MAVLink
