import tomllib
import tomli_w  # Wymagane do zapisu
from dataclasses import dataclass
from pathlib import Path
import numpy as np

# Importujemy logikę matematyczną (zakładam taką strukturę Twoich plików)
# Jeśli plik nazywa się inaczej, dostosuj import
from Application.calc_drop_translation.core_math import run_preflight_simulation 

@dataclass(frozen=True)
class MAVLinkConfig:
    device: str
    device2: str
    baud: int
    mav_version: int

@dataclass(frozen=True)
class ItemConfig:
    mass: int
    cd: float
    drop_course: int
    x_translation: float = 0.0
    y_translation: float = 0.0

@dataclass(frozen=True)
class DropsConfig:
    cruise_speed: int
    wind_speed: int
    wind_direction: int
    altitude: float
    beacon: ItemConfig
    bottle: ItemConfig

@dataclass(frozen=True)
class CameraConfig:
    resolution: tuple[int, int]
    K: np.ndarray
    distortion: np.ndarray
    fisheye: bool

@dataclass(frozen=True)
class DirsConfig:
    root_dir: Path
    logs_dir: Path
    config_dir: Path
    videos_dir: Path
    photos_dir: Path
    zones_dir: Path
    targets_file: Path

@dataclass(frozen=True)
class ZonesPaths:
    search_zone_path: str # W Twoim TOML to ścieżka do pliku .poly

@dataclass(frozen=True)
class LoiterConfig:
    time: float
    alt: float
    radius: float

@dataclass(frozen=True)
class OokConfig:
    duration_s: float
    candidates: tuple[float, ...]
    desired: tuple[float, ...]
    min_confidence: float
    roi_size: int
    brightness_threshold: int

@dataclass(frozen=True)
class LandingSiteConfig:
    lat: float
    lon: float

@dataclass(frozen=True)
class MissionConfig:
    start_wp: int
    stop_wp: int
    is_bottle: bool
    modulation_start_wp: int
    loiter: LoiterConfig
    ook: OokConfig
    landing_sites: tuple[LandingSiteConfig, ...]


class Config:
    def __init__(self, file_name: str = "config.toml"):
        self.ROOT_DIR = Path(__file__).resolve().parent.parent
        self.CONFIG_DIR = self.ROOT_DIR / "configuration"
        self.config_path = self.CONFIG_DIR / file_name
        
        # 1. Wczytaj dane i zainicjalizuj dataclassy
        self._load_and_map()
        
        # 2. Uruchom symulację i zaktualizuj plik TOML na starcie
        self._auto_update_simulation()
        

    def _load_and_map(self):
        """Wczytuje TOML i mapuje na obiekty Python"""
        try:
            with open(self.config_path, "rb") as f:
                self._raw_data = tomllib.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"KRYTYCZNY BŁĄD: Brak pliku w {self.config_path}")

        data = self._raw_data
        try:
            self.debug_enabled = data["system"]["debug_mode"]

            # MAVLink
            mav = data["mavlink"]
            self.mav = MAVLinkConfig(
                device=mav["device"],
                device2=mav.get("device2", mav["device"]),
                baud=mav["baud"],
                mav_version=mav["mav_version"]
            )

            # Drops
            drops = data["drops"]
            def parse_item(key):
                it = drops[key]
                return ItemConfig(
                    mass=it["mass"],
                    cd=it["cd"],
                    drop_course=it["drop_course"],
                    x_translation=it["x_translation"],
                    y_translation=it["y_translation"]
                )

            self.drops = DropsConfig(
                cruise_speed=drops["cruise_speed"],
                wind_speed=drops["wind_speed"],
                wind_direction=drops["wind_direction"],
                altitude=drops["altitude"],
                beacon=parse_item("beacon"),
                bottle=parse_item("bottle")
            )

            # Reszta konfiguracji
            cam = data["camera"]
            self.camera = CameraConfig(resolution=tuple(cam["resolution"]),
                        K=np.array(cam["K"], dtype=np.float32),
                        distortion=np.array(cam["distortion"], dtype=np.float32),
                        fisheye=bool(cam["fisheye"])
            )

            dirs = data["dirs"]
            self.dirs = DirsConfig(
                root_dir=self.ROOT_DIR,
                config_dir=self.CONFIG_DIR,
                logs_dir=self.ROOT_DIR / dirs["logs_dir"],
                videos_dir=self.ROOT_DIR / dirs["videos_dir"],
                photos_dir=self.ROOT_DIR / dirs["photos_dir"],
                zones_dir=self.ROOT_DIR / dirs["zones_dir"],
                targets_file=self.ROOT_DIR / dirs["targets_file"]
            )

            # Init folderów
            for d in [self.dirs.logs_dir, self.dirs.videos_dir, self.dirs.photos_dir]:
                d.mkdir(parents=True, exist_ok=True)

            self.zones = ZonesPaths(search_zone_path=data["zones"]["search_zone_path"])

            mission = data["mission"]
            loiter = mission["loiter"]
            ook = mission["ook"]
            landing_sites_raw = mission.get("landing_sites", [])
            self.mission = MissionConfig(
                start_wp=mission["start_wp"],
                stop_wp=mission["stop_wp"],
                is_bottle=mission["is_bottle"],
                modulation_start_wp=mission.get(
                    "modulation_start_wp", mission["start_wp"]
                ),
                loiter=LoiterConfig(
                    time=loiter["time"],
                    alt=loiter["alt"],
                    radius=loiter["radius"],
                ),
                ook=OokConfig(
                    duration_s=ook["duration_s"],
                    candidates=tuple(ook["candidates"]),
                    desired=tuple(ook["desired"]),
                    min_confidence=ook["min_confidence"],
                    roi_size=ook["roi_size"],
                    brightness_threshold=ook["brightness_threshold"],
                ),
                landing_sites=tuple(
                    LandingSiteConfig(lat=site["lat"], lon=site["lon"])
                    for site in landing_sites_raw
                ),
            )

        except KeyError as e:
            raise KeyError(f"BŁĄD KONFIGURACJI: Brak klucza {e}")

    def _auto_update_simulation(self):
        """Wywołuje core_math i zapisuje wyniki do pliku TOML"""
        # Obliczamy offsety za pomocą Twojej funkcji z core_math
        # Funkcja powinna zwracać np. ((bx, by), (ox, oy))
        beacon_off, bottle_off = run_preflight_simulation(self)

        # Aktualizujemy surowe dane
        self._raw_data["drops"]["beacon"]["x_translation"] = float(beacon_off[0])
        self._raw_data["drops"]["beacon"]["y_translation"] = float(beacon_off[1])
        self._raw_data["drops"]["bottle"]["x_translation"] = float(bottle_off[0])
        self._raw_data["drops"]["bottle"]["y_translation"] = float(bottle_off[1])

        # Zapisujemy fizycznie do pliku config.toml
        with open(self.config_path, "wb") as f:
            tomli_w.dump(self._raw_data, f)
        
        # Przeładowujemy dataclassy, aby miały nowe wartości
        self._load_and_map()

# Inicjalizacja przy starcie aplikacji
try:
    cfg = Config()
    print(f"Pre-flight sim done. Beacon offset: \n x: {cfg.drops.beacon.x_translation}m \n y: {cfg.drops.beacon.y_translation}m")
    print(f"Bottle offset: \n x: {cfg.drops.bottle.x_translation}m \n y: {cfg.drops.bottle.y_translation}m")
except Exception as e:
    print(f"Start drona przerwany: {e}")
    exit(1)