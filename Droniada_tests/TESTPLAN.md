## Plan testowania (kolejność)

### Faza 0 — Przygotowanie

- Aktywuj venv na RPi i upewnij się, że działają: numpy, opencv-python, pymavlink, GStreamer (gi).
- Sprawdź kamerę: python Application/Services/gi_camera_handler.py (demo toggle 10/120 fps).
- W config.toml ustaw:
    [mission] start_wp / stop_wp — indeksy waypointów z Twojej misji w MP (patrz poniżej).
    [mission.ook] candidates — rzeczywiste częstotliwości modulacji z zawodów.
 -Przygotuj misję w MP:
    WP 0 = Home
    WP 1–4 = start, takeoff, trasa przed strefą detekcji
    WP 5 = wejście w strefę skanowania (start_wp = 5)
    WP 6–95 = trasa przez strefę (lot mapowania)
    WP 96 = koniec skanowania (stop_wp = 96)
    Reszta = powrót / lądowanie

1. W `config.toml`: tylko `device = "/dev/ttyAMA2"` (device2 ignorowany przez nowy kod misji).
2. Ustaw `start_wp` / `stop_wp` zgodnie z misją w MP.
3. Jeśli używasz bash: upewnij się, że pipeline GStreamer działa przed startem Pythona, albo pozwól Pythonowi startować kamerę (`manage_pipeline=True`, domyślnie).

---

### Faza 1 — Testy jednostkowe (bez MP, bez kamery)

python Droniada_tests/run_unit_tests.py

|Test|Co sprawdza|
|---|---|
|`test_led_detector`|Detekcja plamki LED|
|`test_detection_pipeline`|`process_one_frame()` w jednym wątku, bez 2. UART|
|`test_ook` / `test_ook_worker`|Dekoder modulacji|
|`test_mission_planner`|Trasa nearest-neighbor + LOITER|
|`test_mission_landing_sites`|Wysyłka lądowisk|

---

### Faza 2 — Kamera + diody (bez MP)

Cel: GiCameraService + LedDetector działają (bash może już działać w tle).

python Droniada_tests/live/live_led_detection.py --frames 200

Sukces: komunikaty `[DIODA] Wykryto nową diodę ID=... pixel=(x,y)`  
Przy bash: uruchom skrypt transmisji, potem ten test z `manage_pipeline=False` jeśli pipeline jest zewnętrzny.

---

### Faza 3 — Telemetria (1 UART + MP SITL)

Cel: Jeden link MAVLink, odczyt GPS + attitude + wp.

python Droniada_tests/live/live_telemetry.py --seconds 30

python Droniada_tests/live/live_telemetry.py --with-camera --seconds 30

Co robisz w MP: SITL w AUTO, misja leci — obserwujesz zmianę `wp=`.

Sukces:

[TELEMETRIA #N] GPS=(lat, lon, alt=...) att=(r,p,y) wp=5

Test `--with-camera`: sprawdza, że kamera (GiCamera) nie blokuje MAVLink w tym samym procesie.

---

### Faza 4 — OOK (kamera 120 fps)

python Droniada_tests/live/live_ook_detection.py

Modulacja OOK w centrum kadru → `[OK] Modulacja potwierdzona: X Hz`.

---

### Faza 5 — Pipeline detekcji + MP (1 UART, główny wątek)

python Testy_Antek/Mapping03_detection_pipeline_T.py

Przebieg:

1. MP: misja z wp `start_wp`→`stop_wp` przez strefę `.poly`
2. Skrypt czeka na `wp == start_wp`
3. W tym samym wątku: co 100 ms `get_mission_status()` + `process_one_frame()` (kamera 10 fps + GPS przez `process_target`)
4. Stop przy `wp >= stop_wp`
5. Log: `Detected targets count: N`

Sukces: brak crashy, sensowne `TRG_CANDIDATES` z diodami w polu widzenia.

---

### Faza 6 — Pełna misja (dry-run)

python Testy_Antek/DroniadaMission_T.py --dry-run

Fazy w jednym procesie / jednym UART:

1. Czeka na `start_wp`
2. Detekcja LED (main thread)
3. Plan LOITER (nearest-neighbor)
4. [dry-run] bez `append_waypoints`
5. OOK przy każdym LOITER (120 fps, osobny Process tylko do klasyfikacji — nie dotyka MAVLink)
6. Zapis `media/droniada_targets.json`

Sprawdź JSON: `targets`, `ook_results`, `landing_sites`.

---

### Faza 7 — Pełna misja (produkcja)

python Testy_Antek/DroniadaMission_T.py

Dopisuje LOITER waypoints i wysyła `send_landing_sites()` do samolotu.

---

### Faza 8 — Bridge samolotowy

Na laptopie samolotowym:

python Application/drone_plane_connection/bridge_plane.py --listen-port 5770 --plane-device tcp:localhost:5761

MP dronowy: TCP outbound → laptop samolotowy.

---

## Diagram przepływu (1 UART)

Główny wątek PythonProcess OOK 10s120fps samplesMAVLink 1x flowchart TD
    subgraph mainThread [Główny wątek Python]
        MAV[MAVLink 1x UART]
        WP[get_mission_status]
        CAM[GiCameraService.get_frame]
        LED[LedDetector]
        PT[process_target GPS+att]
        MAV --> WP
        CAM --> LED --> PT
        PT --> MAV
    end

    subgraph ookProc [Process OOK 10s]
        CLASS[classify_ook]
    end

    CAM -->|120fps samples| ookProc
    CLASS --> mainThread

![[Pasted image 20260606203943.png]]
---

## Checklist przed zawodami

-  `live_led_detection.py` widzi diody
-  `live_telemetry.py` stabilne GPS @ 10 Hz (1 połączenie MP)
-  `live_ook_detection.py` rozpoznaje częstotliwość z `candidates` w config
-  `Mapping03` — pipeline bez crashy w SITL
-  `DroniadaMission --dry-run` — sensowny JSON
-  `DroniadaMission` — LOITER + lądowiska w MP / bridge

Jeśli chcesz, mogę dopisać przykładowy skrypt bash do uruchamiania pipeline razem z `DroniadaMission_T.py` w trybie `manage_pipeline=False`.