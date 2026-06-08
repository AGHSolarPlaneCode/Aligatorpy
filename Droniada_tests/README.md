# Testy misji Droniada

## Architektura (1 UART + GiCameraService)

- **Jeden port MAVLink** (`cfg.mav.device`) — cała telemetria i misja w **głównym wątku**
- **Kamera:** `GiCameraService` → `gi_camera_handler.CameraPipeline` (appsink 10/120 fps)
- Skrypt bash może uruchomić pipeline GStreamer z zewnątrz → `GiCameraService(manage_pipeline=False)` + `attach_pipeline()`

## Testy jednostkowe (bez sprzętu)

```bash
python Droniada_tests/run_unit_tests.py
```

### Projekcja piksel → GPS (`process_one_frame` → `process_target`)

Testy sprawdzają łańcuch używany w `Droniada_mission._run_detection_phase`:
`_scale_pixel` → `process_target` → `project_target_cords` (undistort, promień, przecięcie z ziemią @ 50 m).

```bash
python Droniada_tests/unit/test_pixel_projection.py
```

Wizualizacja siatki 10×10 na mapie (wymaga `matplotlib`):

```bash
python Droniada_tests/live/live_pixel_projection_map.py
python Droniada_tests/live/live_pixel_projection_map.py --roll 10 --pitch 5 --alt 50
python Droniada_tests/live/live_pixel_projection_map.py --output media/projection_map.png --no-show
```

## Testy live (RPi)

```bash
# 1. Kamera + detekcja diod
python Droniada_tests/live/live_led_detection.py --frames 200

# 2. Telemetria (1 UART) — opcjonalnie z kamerą w tle
python Droniada_tests/live/live_telemetry.py --seconds 30
python Droniada_tests/live/live_telemetry.py --with-camera --seconds 30

# 3. OOK @ 120fps
python Droniada_tests/live/live_ook_detection.py

# 4. Pipeline + MP SITL (1 UART, main thread)
python Testy_Antek/Mapping03_detection_pipeline_T.py

# 5. Pełna misja
python Testy_Antek/DroniadaMission_T.py --dry-run
python Testy_Antek/DroniadaMission_T.py
```

## Bash + zewnętrzny pipeline

Jeśli bash uruchamia `CameraPipeline` przed Pythonem:

```python
from Application.Services.gi_camera_handler import CameraPipeline
from Application.Services.GiCameraService import GiCameraService

pipeline = CameraPipeline()  # uruchomiony w bash / osobnym wątku
camera = GiCameraService(manage_pipeline=False, pipeline=pipeline)
camera.attach_pipeline(pipeline)
# camera.start() — no-op w trybie zewnętrznym
camera.set_10fps_mode()
```
