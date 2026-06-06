# Plan testowania — instrukcje segment po segmencie

Testy ułożone od warstwy bez ryzyka (obliczenia) do latającego drona. Każdy
segment opiera się na poprzednim: gdy coś pęknie, problem jest w bieżącym
segmencie, bo poprzednie już przeszły. Uruchamiaj **z katalogu głównego projektu**
przez `python -m tests.segX_...` (tak działają importy pakietów).

## Stanowiska

- **A** — samo RPi (bez FC/kamery): warstwa obliczeniowa
- **B** — RPi + FC przez UART + SITL w Mission Planner: MAVLink i misja
- **C** — RPi + FC + kamera, dron trzymany poziomo nad diodą: wizja i lokalizacja

Segmenty 0–4 (B) i 5–8 (C) można robić równolegle. Łączą się w 9.

## Zależności

```
0 (oblicz.) ─ niezależny
1 (UART) → 2 (telemetria) → 3 (komendy) → 4 (misja+atrapa)   [stanowisko B]
5 (kamera) → 6 (detekcja) → 7 (lokalizacja) ; 6 → 8 (OOK)    [stanowisko C]
4 i 6 → 9 (integracja) → 10 (pełna SITL+kamera)
```

---

## Segment 0 — Warstwa obliczeniowa (A)

```bash
python -m tests.seg0_compute
```

Bez sprzętu. Potwierdza że shared/ działa na docelowych wersjach numpy/opencv.
Zapisz wypisane wersje bibliotek. **PASS** → idź do 1 lub 5.

---

## Segment 1 — Łączność MAVLink (B, bez SITL)

```bash
python -m tests.seg1_mavlink_link --uart /dev/ttyAMA0 --baud 115200 --seconds 15
```

Surowe pymavlink, bez MatekService. Sprawdza heartbeat, obecność ATTITUDE/GPS,
błędy parsowania. Najpierw sprawdź konfigurację FC: który port to companion,
`SERIALx_PROTOCOL=2`, baud zgodny.

Najczęstsze FAIL:
- brak heartbeat → zły port/baud, zły protokół na porcie FC
- brak ATTITUDE/GPS → parametry `SRx_*` blokują strumienie, brak fixu GPS
- dużo błędów → za wysoki baud, brak flow control

---

## Segment 2 — Demux i telemetria (B, SITL)

```bash
python -m tests.seg2_telemetry --uart udp:127.0.0.1:14551
```

Uruchom SITL w Mission Planner. Połącz MatekService do tego samego SITL
(np. przez dodatkowy output mavproxy `--out udp:127.0.0.1:14551`).
Sprawdza gęstość bufora (~30 Hz), interpolację, **yaw przez ±π**.

Dla testu yaw: obróć drona w SITL przez północ (yaw przez 180°) podczas działania
skryptu — skrypt złapie przejście w buforze. Jeśli nie złapie, test jest pominięty
(nie blokuje), ale obejrzyj ręcznie.

---

## Segment 3 — Komendy i protokół misji (B, SITL)

```bash
python -m tests.seg3_commands --uart udp:127.0.0.1:14551
```

INTERAKTYWNY — obserwuj Mission Planner. Uzbraja SITL, wgrywa misję, leci do A,
testuje goto. **Nie rób na prawdziwym dronie ze śmigłami.**

Kluczowe testy: [4] zła misja → False (poprawka bug ACK), [7] telemetria płynie
podczas uploadu (demux nie blokuje).

---

## Segment 4 — Orchestracja z atrapą Oka (B, SITL)

```bash
python -m tests.seg4_mission_dummy_eye --uart udp:127.0.0.1:14551
```

Pełny `run_mission_manager` z atrapą Oka na drugim końcu Pipe. Atrapa generuje
sztuczne detekcje (piksele liczone z żywej telemetrii SITL) i wyniki OOK,
testuje też retry (freq=None) i Złotą Weryfikację (przesunięty piksel).

Po tym segmencie **cała logika misji jest sprawdzona** — brakuje tylko wizji.
Sprawdź `/tmp/seg4_wyniki.csv`.

---

## Segment 5 — Pipeline kamery surowo (C, bez FC)

```bash
python -m tests.seg5_camera_raw
```

Tylko RPi z kamerą OV9281. Sprawdza start GStreamera, rozmiar klatek, fps obu
gałęzi, i **mierzy CLOCK_OFFSET**. Skrypt wypisze sugerowaną wartość — wpisz ją
do `shared/protocol.py` jako `CLOCK_OFFSET`.

Wymaga GStreamer + libcamera + python3-gi zainstalowanych systemowo.

---

## Segment 6 — Detekcja plamki (C, kamera nad diodą)

```bash
python -m tests.seg6_detect --threshold 220 --seconds 20
```

Skieruj kamerę na świecącą (stałą) diodę IR. Obserwuj wypisywany piksel — czy
zgadza się z pozycją diody (porównaj ze streamem podglądu). Test obserwacyjny —
werdykt stawiasz Ty. Dostrój `--threshold`, wpisz wynik do `SEARCH_THRESHOLD`.

---

## Segment 7 — Lokalizacja GPS (C) — KLUCZOWY

```bash
python -m tests.seg7_localize --uart udp:... \
    --diode-lat 50.27110 --diode-lon 18.67276 --alt 50
```

Weryfikuje `pixel_to_gps` + `rot_matrix` — konwencję osi. Zmierz GPS diody,
ustaw kamerę w znanym GPS/wysokości, uruchom. Porównaj rzutowany GPS z prawdziwym.

**Wykonaj DWA razy: yaw≈0° (północ) i yaw≈90° (wschód).** Jeśli błąd przy yaw=0
mały, a przy yaw=90 duży → błąd konwencji w `rot_matrix`, napraw znaki/kolejność
osi. To najbardziej prawdopodobne miejsce na buga — zarezerwuj czas.

---

## Segment 8 — Dekodowanie OOK (C, migająca dioda)

```bash
python -m tests.seg8_ook --expected 12 --threshold 220
```

Ustaw diodę migającą ze znaną częstotliwością. Sprawdza decode_ook na realnym
sygnale 120fps. Powtórz dla {4,6,12,16}. Skrypt zmierzy też confidence szumu
(poprosi o zgaszenie diody) i zasugeruje `MIN_OOK_CONFIDENCE`.

---

## Segment 9 — Integracja procesów (C, kamera + SITL)

```bash
python -m tests.seg9_integration --uart udp:127.0.0.1:14551
```

Pełny `main.py` w praktyce: Mózg + Oko, prawdziwa kamera, SITL jako FC. Test
PRZEPŁYWU (nie sensowności): detekcje płyną, przejście SEARCH→VISIT, czysty
SHUTDOWN, CPU < 90%. Po teście sprawdź `ps aux` pod kątem wiszących procesów.

Opcjonalnie zainstaluj `psutil` dla dokładnego pomiaru CPU (inaczej loadavg).

---

## Segment 10 — Pełna misja SITL + kamera (B+C)

```bash
python main.py --uart udp:127.0.0.1:14551
```

Ostatni test przed lotem. SITL lata wirtualnie, kamera patrzy na układ migających
diod (pozycje pokryj z geometrią SITL). Prześledź misję end-to-end, sprawdź
`wyniki_diody.csv`. Brak osobnego skryptu — to uruchomienie produkcyjne z
checklistą:

- [ ] misja przechodzi bez wyjątków
- [ ] CSV zawiera wykryte diody z częstotliwościami i flagami is_target
- [ ] brak zawieszeń i wiszących procesów

---

## Dwie rzeczy do zrobienia w trakcie

1. **CLOCK_OFFSET** — zmierz w Segmencie 5, zweryfikuj w Segmencie 7 (jeśli
   lokalizacja ma błąd zależny od prędkości drona, offset jest źle dobrany).
2. **Progi** — `SEARCH_THRESHOLD`/`OOK_THRESHOLD` (Segment 6/8),
   `MIN_OOK_CONFIDENCE` (Segment 8), ewentualnie `CLUSTER_RADIUS_M` i
   `min_frame_ratio` po pierwszym pełnym przelocie z realnymi danymi.

## Dlaczego ten układ działa

Gdy Segment 7 pokaże błąd lokalizacji, wiesz że 0–6 są dobre — problem jest w
`rot_matrix` lub `pixel_to_gps`, nie w detekcji ani telemetrii. To zawęża
poszukiwania z całego systemu do jednej funkcji.
