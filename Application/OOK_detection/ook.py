"""
Dekoder OOK — bank korelatorów na REALNYCH znacznikach czasu klatek.

Zamiast zakładać stałe fps i sztywne wzorce, projektujemy sygnał jasności na
zespolone sinusoidy o częstotliwościach kandydujących, liczonych na faktycznych
czasach próbek. Dzięki temu metoda jest odporna na drop/jitter klatek (videorate)
i niezależna od fazy (bierzemy moc = magnitudę zespoloną).

Dla fali prostokątnej składowa podstawowa dominuje nad harmonicznymi
(amplitudy 4/π, 4/3π, 4/5π...), więc argmax mocy trafia w częstotliwość nośną.
"""
from __future__ import annotations

import numpy as np


def ook_brightness(roi: np.ndarray, thr: int = 220) -> int:
    """Liczba pikseli jaśniejszych niż thr w wycinku ROI (surowy sygnał on/off)."""
    return int(np.count_nonzero(roi > thr))


def classify_ook(samples, timestamps, candidates, min_confidence: float = 4.0):
    """
    Klasyfikuje częstotliwość migotania OOK.

    Args:
        samples:        1D sekwencja jasności (np. liczba jasnych pikseli) per klatka
        timestamps:     1D czasy próbek [s] (liczy się tylko różnica)
        candidates:     lista częstotliwości kandydujących [Hz]
        min_confidence: poniżej tego progu uznajemy, że nie ma wyraźnego sygnału
                        (sam szum) i zwracamy (None, conf). Prawdziwy sygnał OOK
                        ma conf >> 1; szum ~1.5. Ustaw 0, by wyłączyć filtr.

    Returns:
        (best_freq_hz, confidence) gdzie confidence = P_best / P_drugie_najwieksze.
        (None, conf) gdy za mało próbek, sygnał płaski, lub conf < min_confidence.
    """
    s = np.asarray(samples, dtype=np.float64)
    t = np.asarray(timestamps, dtype=np.float64)

    n = s.size
    if n < 4 or candidates is None or len(candidates) == 0 or t.size != n:
        return None, 0.0

    # usuń składową stałą (DC) — interesuje nas tylko migotanie
    x = s - s.mean()
    energy = float(np.dot(x, x))
    if energy <= 1e-9:
        return None, 0.0   # płaski sygnał — dioda nie miga albo brak detekcji

    # moc projekcji na zespoloną sinusoidę dla każdej częstotliwości kandydującej
    freqs = np.asarray(candidates, dtype=np.float64)
    phase = np.exp(-1j * 2.0 * np.pi * np.outer(freqs, t))   # [F, N]
    X = phase @ x                                            # [F]
    powers = (X.real ** 2 + X.imag ** 2)                    # [F]

    order = np.argsort(powers)[::-1]
    best_idx = int(order[0])
    best_freq = float(freqs[best_idx])
    best_power = float(powers[best_idx])

    if powers.size > 1:
        second = float(powers[order[1]])
        confidence = best_power / second if second > 1e-12 else float("inf")
    else:
        confidence = float("inf")

    if confidence < min_confidence:
        return None, confidence   # brak wyraźnego zwycięzcy — prawdopodobnie szum

    return best_freq, confidence
