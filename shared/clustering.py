"""
Klastrowanie celów z fazy SEARCH.

Każde wykrycie (lat, lon) trafia do najbliższego istniejącego klastra w promieniu
CLUSTER_RADIUS_M, albo zakłada nowy. Per klaster trzymamy tylko TOP_FRAMES klatek
o najmniejszym dystansie optycznym (najbliżej środka matrycy = najmniejszy błąd
kąta ślizgowego). Finalny GPS diody = średnia z tych najlepszych klatek.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from shared.geometry import gps_to_ned_offset
from shared.protocol import CLUSTER_RADIUS_M, TOP_FRAMES


def distance_m(lat1, lon1, lat2, lon2) -> float:
    """Dystans [m] między dwoma punktami (rzut płaski przez NED)."""
    north, east = gps_to_ned_offset(lat1, lon1, lat2, lon2)
    return math.hypot(north, east)


@dataclass
class _Cluster:
    # klatki jako (optical_dist, lat, lon), trzymane posortowane rosnąco po optical_dist
    frames: list = field(default_factory=list)

    def centroid(self) -> tuple:
        """Bieżący środek (średnia GPS wszystkich trzymanych klatek) — do dopasowania."""
        n = len(self.frames)
        lat = sum(f[1] for f in self.frames) / n
        lon = sum(f[2] for f in self.frames) / n
        return lat, lon


class TargetClusterer:
    def __init__(self, radius_m: float = CLUSTER_RADIUS_M, top: int = TOP_FRAMES):
        self.radius_m = radius_m
        self.top = top
        self.clusters: list[_Cluster] = []

    def add(self, lat: float, lon: float, optical_dist: float) -> None:
        """
        Dokłada wykrycie do najbliższego klastra w promieniu radius_m, albo tworzy nowy.
        optical_dist = dystans surowego piksela od środka matrycy (jakość klatki).
        """
        best, best_d = None, self.radius_m
        for c in self.clusters:
            clat, clon = c.centroid()
            d = distance_m(lat, lon, clat, clon)
            if d <= best_d:
                best, best_d = c, d

        if best is None:
            best = _Cluster()
            self.clusters.append(best)

        best.frames.append((optical_dist, lat, lon))
        # zostaw tylko TOP najlepszych (najmniejszy optical_dist)
        if len(best.frames) > self.top:
            best.frames.sort(key=lambda f: f[0])
            del best.frames[self.top:]

    def finalize(self, min_frames: int = 1, min_frame_ratio: float = 0.0) -> list[dict]:
        """
        1. Pass scalający: łączy klastry, których środki są w promieniu radius_m
           (naprawia rozpad jednej diody na fragmenty przy dużym błędzie GPS).
        2. Dla każdego scalonego klastra: średnia GPS WAŻONA jakością
           (waga = 1/(1+optical_dist) — klatki bliżej środka ważą więcej).
        3. Filtry: min_frames (bezwzględny, na przypadkowe błyski) oraz
           min_frame_ratio (względny — odrzuca klastry z liczbą klatek
           < ratio * max_count; tłumi fragmenty brzegowe). ratio=0 wyłącza.

        UWAGA: radius_m musi przekraczać realny rozrzut błędu GPS po odrzuceniu
        klatek brzegowych, ale pozostawać < połowy minimalnego odstępu diod (10 m).
        Wartości do dostrojenia na realnym nagraniu.
        """
        # --- pass scalający (aglomeracyjny) ---
        merged = [list(c.frames) for c in self.clusters]
        changed = True
        while changed:
            changed = False
            for i in range(len(merged)):
                if not merged[i]:
                    continue
                ci = self._centroid(merged[i])
                for j in range(i + 1, len(merged)):
                    if not merged[j]:
                        continue
                    cj = self._centroid(merged[j])
                    if distance_m(ci[0], ci[1], cj[0], cj[1]) <= self.radius_m:
                        merged[i].extend(merged[j])
                        merged[j] = []
                        changed = True
                        break

        merged = [f for f in merged if f]
        if not merged:
            return []

        max_count = max(len(f) for f in merged)
        threshold = max(min_frames, int(min_frame_ratio * max_count))

        out = []
        for frames in merged:
            if len(frames) < threshold:
                continue
            lat, lon = self._weighted_centroid(frames)
            out.append({"lat": lat, "lon": lon, "n_frames": len(frames)})
        return out

    @staticmethod
    def _centroid(frames) -> tuple:
        n = len(frames)
        return sum(f[1] for f in frames) / n, sum(f[2] for f in frames) / n

    @staticmethod
    def _weighted_centroid(frames) -> tuple:
        w = [1.0 / (1.0 + f[0]) for f in frames]   # waga = 1/(1+optical_dist)
        sw = sum(w)
        lat = sum(wi * f[1] for wi, f in zip(w, frames)) / sw
        lon = sum(wi * f[2] for wi, f in zip(w, frames)) / sw
        return lat, lon
