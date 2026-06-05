"""
Protokół komunikacji Mózg <-> Oko przez multiprocessing.Pipe oraz wspólne stałe.

Komendy idą Mózg -> Oko (Cmd), dane Oko -> Mózg (Evt).
Każda wiadomość to PipeMsg(kind, payload), gdzie payload zależy od kind:
    Cmd.START_SEARCH  -> None
    Cmd.STOP_SEARCH   -> None
    Cmd.START_DECODE  -> DecodeRequest
    Cmd.SHUTDOWN      -> None
    Evt.DETECTION     -> Detection
    Evt.OOK_RESULT    -> OokResult
    Evt.ERROR         -> str
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


# ── Typy wiadomości ──────────────────────────────────────────────────────────
class Cmd(Enum):
    START_SEARCH = auto()
    STOP_SEARCH = auto()
    START_DECODE = auto()
    SHUTDOWN = auto()


class Evt(Enum):
    DETECTION = auto()
    OOK_RESULT = auto()
    ERROR = auto()


@dataclass
class Detection:
    """Pojedyncze wykrycie diody w fazie SEARCH (surowy piksel + czas przechwycenia)."""
    raw_x: float
    raw_y: float
    ts: float          # monotoniczny znacznik czasu klatki [s]


@dataclass
class DecodeRequest:
    """Podpowiedź dla Oka: gdzie na SUROWEJ matrycy szukać diody do dekodowania."""
    hint_x: float
    hint_y: float


@dataclass
class OokResult:
    """Wynik dekodowania OOK: częstotliwość + uśredniony surowy piksel śledzenia."""
    freq_hz: float | None
    avg_raw_x: float
    avg_raw_y: float
    confidence: float          # P_best / P_drugie


@dataclass
class PipeMsg:
    kind: object               # Cmd | Evt
    payload: object = None      # Detection | DecodeRequest | OokResult | str | None


# ── Stałe konfiguracyjne (jedno źródło prawdy dla obu procesów) ──────────────
TARGET_FREQS = {4, 6, 12, 16}                       # częstotliwości "celów"
CANDIDATE_FREQS = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]   # pełny zbiór kandydatów [Hz]

# Synchronizacja czasu: offset między zegarem klatek (GStreamer monotonic)
# a time.monotonic() Mózgu. Roboczo 0 — do zmierzenia jednorazowo na sprzęcie.
CLOCK_OFFSET = 0.0

# OOK
OOK_WINDOW_S = 3.0          # długość okna pomiarowego [s]
OOK_ROI_PX = 80             # bok lepkiego okienka ROI [px]
OOK_THRESHOLD = 220         # próg jasności piksela (dioda on/off)
MIN_OOK_CONFIDENCE = 4.0    # dostroić na realnym nagraniu    # poniżej -> brak wyraźnego sygnału (szum), retry

# SEARCH
SEARCH_THRESHOLD = 220      # próg jasności w fazie przeszukiwania
EDGE_REJECT_PX = 550        # odrzuć bloby dalej niż tyle od środka matrycy

# Klastrowanie celów
CLUSTER_RADIUS_M = 4.0      # promień łączenia wykryć w jeden klaster [m]
TOP_FRAMES = 40             # ile najlepszych (najbliżej środka) klatek trzymać

# Faza VISIT
STABILIZE_S = 3.0           # stałe oczekiwanie na uspokojenie ramy [s]
GOLDEN_DIST_M = 5.0         # próg weryfikacji tożsamości diody [m]
MAX_RETRY = 3               # maks. prób na diodę
