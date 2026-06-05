# Dron IR-LED — system dwuprocesowy (Mózg + Oko)

Autonomiczna misja: przelot A→B na 50 m nocą, detekcja diod IR 850 nm,
lokalizacja każdej do GPS, nalot na każdą i odczyt częstotliwości migotania (OOK).
Częstotliwości-cele: {4, 6, 12, 16} Hz; reszta to wabiki. Wynik → CSV.

## Architektura

Dwa procesy na jednym RPi 5, spięte jednym dwukierunkowym `Pipe()`:

- **Mózg** (`mission/brain.py`) — jedyny komunikujący się z FC (jeden UART).
  Cała nawigacja, bufor telemetrii, synchronizacja czasu, rzutowanie piksel↔GPS,
  klastrowanie, weryfikacja, zapis CSV.
- **Oko** (`vision/eye.py`) — tylko kamera, BEZ FC. Detekcja plamek (SEARCH)
  i odczyt OOK (DECODE). Wysyła surowe piksele + znaczniki czasu; undistort i
  projekcję robi Mózg.

Komendy idą Mózg→Oko (`START_SEARCH`/`STOP_SEARCH`/`START_DECODE`/`SHUTDOWN`),
dane Oko→Mózg (`DETECTION`/`OOK_RESULT`).

```
                 ┌──────────── Pipe ────────────┐
   FC ── UART ── │ Mózg (brain) │◄──►│ Oko (eye) │ ── kamera OV9281
                 └──────────────┘    └───────────┘
```

## Struktura plików

```
project/
├── main.py                          # spawn Mózg + Oko, jeden Pipe
├── requirements.txt
├── glowice_dron.poly                # strefa detekcji (lat lon na linię)
├── shared/                          # warstwa wspólna, bez zależności sprzętowych
│   ├── protocol.py                  # Cmd/Evt/PipeMsg, dataclassy, stałe
│   ├── calibration.py               # K, dist, CALIB_W/H, scale_K()
│   ├── geometry.py                  # pixel↔GPS, point_in_polygon, rot_matrix
│   ├── ook.py                       # bank korelatorów na realnych czasach
│   └── clustering.py                # łączenie wykryć w diody
├── mission/
│   └── brain.py                     # Proces 1 — fazy SEARCH i VISIT
├── vision/
│   ├── eye.py                       # Proces 2 — detekcja + OOK
│   └── camera_handler.py            # GStreamer: stream / search 30fps / decode 120fps
└── Application/Services/
    └── MatekService.py              # warstwa FC: jeden wątek RX + demux
```

## Uruchomienie

```bash
pip install -r requirements.txt          # + GStreamer/libcamera systemowo
python main.py --uart /dev/ttyAMA0
```

Streaming podglądu (shmsink → laptop) działa równolegle i niezależnie — nie wymaga
osobnego startu z tej strony.

## Punkty konfiguracji (`mission/brain.py`)

| Stała | Znaczenie |
|---|---|
| `WP_A`, `WP_B` | punkty trasy przelotu |
| `POLY_PATH` | plik strefy detekcji (geofence) |
| `ALT` | wysokość przelotu [m] |
| `SEARCH_SPEED` | prędkość w fazie SEARCH [m/s] |
| `ARRIVAL_RADIUS_M` | promień uznania dolotu nad diodę |
| `CSV_PATH` | plik wyników |

Kalibracja kamery: `shared/calibration.py` (K i dist z kalibracji na 1280×800).

## Parametry do dostrojenia na realnych danych (`shared/protocol.py`)

- `CLOCK_OFFSET` — offset między zegarem klatek (GStreamer monotonic) a
  `time.monotonic()` Mózgu. Roboczo `0.0`; **zmierzyć na sprzęcie** (sekcja niżej).
- `MIN_OOK_CONFIDENCE` — próg pewności OOK. Na syntetyku szum dawał ~1,5, sygnał ≥7;
  ustawić po pierwszym realnym nagraniu.
- `CLUSTER_RADIUS_M` + `min_frame_ratio` (w `phase_search`) — zależą od realnego
  błędu GPS/atitude. Promień musi przekraczać rozrzut po odrzuceniu krawędzi, ale
  pozostać < połowy minimalnego odstępu diod (10 m).
- `SEARCH_THRESHOLD` / `OOK_THRESHOLD` — progi jasności plamki (zależą od ekspozycji).

## Checklista przedlotowa (wymaga sprzętu/SITL)

1. **Test naziemny rzutowania** — `rot_matrix` ma nietypową konwencję osi i nie był
   sprawdzony w realu. Połóż diodę w znanym GPS, zawiśnij w znanym GPS/yaw, złap
   klatkę, podaj telemetrię ręcznie, porównaj `pixel_to_gps` z prawdą (<1–2 m).
   Powtórz dla yaw=90° i z niezerowym pitch/roll — tu wyjdą błędy konwencji.
   Rzut tam i z powrotem (`gps_to_pixel`) jest matematycznie spójny (zweryfikowane),
   ale to nie potwierdza zgodności konwencji ze światem.
2. **Pomiar `CLOCK_OFFSET`** — porównaj znacznik klatki z `get_image()` z bieżącym
   `time.monotonic()` Mózgu w jednej chwili; różnicę wpisz jako `CLOCK_OFFSET`.
3. **Strojenie OOK** — nagraj realne diody, sprawdź `confidence` sygnału vs szumu,
   ustaw `MIN_OOK_CONFIDENCE` między nimi.

## Stan testów

Co przetestowano bez sprzętu (mocki MAVLink + numpy/opencv):

- `shared/`: `classify_ook` 10/10 częstotliwości mimo jittera, 30% gubionych klatek
  i szumu; round-trip GPS↔piksel do 0,03 px; klastrowanie 4/4 diody przy szumie
  1–2,5 m z błędem 0,17–0,47 m.
- `MatekService`: demux RX, interpolacja yaw przez ±π, arm/set_mode/wait_item_reached
  przez demux, upload misji przez `_proto_q`.
- Mózg: `phase_search` (rzut + klaster, 1,4–1,8 cm), `phase_visit` (OOK + Złota
  Weryfikacja + retry + CSV) z atrapą Oka przez prawdziwy Pipe.

Wymaga sprzętu/SITL (niemożliwe w piaskownicy): integracja obu procesów z realnym
FC/SITL i kamerą, `eye.py` + `camera_handler.py` na żywym GStreamerze, oraz trzy
punkty checklisty wyżej.
