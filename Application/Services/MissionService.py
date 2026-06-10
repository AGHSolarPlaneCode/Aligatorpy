from __future__ import annotations
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg     # mandatory
from Application.Logger.log_module import get_logger
import numpy as np
from pymavlink import mavutil, mavextra
from typing import Tuple, List, Dict, Optional
import cv2
#import core_math
import math

class MissionService:
    """
    Args:
        drone: MatekService object instance
    """
    def __init__(self, drone: MatekService):
        self.logger = get_logger(__name__)
        self.image_width, self.image_height = cfg.camera.resolution

        self.TRG_CANDIDATES=[] #{"lat": lat, "lon": lon, "count": 1, "isBottle": isBottle}

        self.drone = drone

    K = cfg.camera.K
    dist = cfg.camera.distortion
    fisheye = cfg.camera.fisheye

    
    def rot_matrix(self, roll, pitch, yaw):
        Rx = np.array([[ np.cos(roll), 0, -np.sin(roll)],
                    [0, 1, 0],
                    [np.sin(roll), 0, np.cos(roll)]])
        Ry = np.array([[1, 0, 0],
                    [0, np.cos(pitch), np.sin(pitch)],
                    [0, -np.sin(pitch),  np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), np.sin(yaw), 0],
                    [-np.sin(yaw),  np.cos(yaw), 0],
                    [0, 0, 1]])
        return Rz @ Ry @ Rx


    def project_target_cords(self, pixel: tuple, lat_uav, lon_uav, alt_uav,roll, pitch, yaw)->tuple:
        """
        Oblicza współrzędne geograficzne punktu na ziemi
        odpowiadającego pikselowi (u,v) z obrazu kamery
        zamontowanej pionowo w dół (nadir).
        Args:
            pixel: (width, height)
        Returns:
            (lat, lon)
        """
        # pobierz rozdzielczość obrazu
        u, v = pixel

        # sprawdzamy, czy piksel jest w zakresie obrazu
        if not (0 <= u < self.image_width and 0 <= v < self.image_height):
            print(f"Piksel ({u},{v}) poza zakresem ({self.image_width}x{self.image_height})")
            return None
        
        # korekta dystorsji
        if MissionService.fisheye:
            undistorted = cv2.fisheye.undistortPoints(
            np.array([[[float(u), float(v)]]], dtype=np.float64),
            MissionService.K,
            MissionService.dist
        )
        else:
            undistorted = cv2.undistortPoints(
            np.array([[u, v]], dtype=np.float32),
            cameraMatrix=MissionService.K,
            distCoeffs=MissionService.dist
        )

        x_u, y_u = undistorted[0,0]
        
        # promień w układzie kamery
        ray_cam = np.array([x_u, -y_u, 1.0]) #minus przy y_u
        ray_cam /= np.linalg.norm(ray_cam)

        R_world_body = self.rot_matrix(roll, pitch, yaw)
        ray_world = R_world_body @ ray_cam
        dz = ray_world[2]

        # Sprawdź czy promień nie jest równoległy do ziemi lub skierowany ponad horyzont
        if abs(dz) < 1e-6 or dz <= 0:
            print("Ray is parallel to ground or above horizon, cannot compute intersection.")
            return None
        
        # ---- przecięcie z ziemią ----
        h_g = 0.0
        t = (alt_uav - h_g) / dz
        hit_local = ray_world * t   # [north, east, down]

        # ---- GPS offset ----
        lat_t, lon_t = mavextra.gps_offset(
            lat_uav, lon_uav,
            hit_local[1],  # east switched
            hit_local[0]   # north switched
        )
        print("detected point", lat_t, lon_t)
        return lat_t, lon_t

    def isinPolygon(self, lat, lon, Polygon: List[Tuple[float, float]]) -> bool:
        '''
           Polygon: List of (lat, lon) tuples defining the vertices of the polygon in order!!!
        '''
        num = len(Polygon)
        inside = False
        for i in range(num):
            j = (i + 1) % num
            xi, yi = Polygon[i][1], Polygon[i][0] # xi = lon, yi = lat
            xj, yj = Polygon[j][1], Polygon[j][0]
            # Algorytm Ray Casting
            if ((yi > lat) != (yj > lat)) and \
            (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
        return inside
    

    def is_target(self, lat, lon, threshold=5.0):
        """
            Sprawdza, czy (lat, lon) znajduje się w promieniu threshold_meters od celów.
            Modyfikuje self.TRG_CANDIDATES, zwiększając 'count' dla celów within threshold.
        """
        for target in self.TRG_CANDIDATES:
            dist = self.get_distance_meters(lat, lon, target["lat"], target["lon"])
            if dist <= threshold:
                return True
        return False
    
    def insert_target(self, lat, lon, isBottle: bool):
        self.TRG_CANDIDATES.append({"lat": lat, "lon": lon, "count": 1, "isBottle": isBottle})

    def process_target(self, pixel, isBottle: bool, search_zone)-> bool:
        msg_gps = self.drone.get_current_coordinates()
        msg_att = self.drone.get_attitude()
        if msg_att is None:
            return None
        roll, pitch, yaw = msg_att
        if msg_gps is None:
            return None
        lat_uav, lon_uav, alt_uav = msg_gps

        res = self.project_target_cords(pixel, lat_uav, lon_uav, alt_uav, roll, pitch, yaw)
        if res is None:
            return False
        lat, lon = res
        if not self.isinPolygon(lat, lon, search_zone):
            return False
        if not self.is_target(lat, lon):
            self.insert_target(lat, lon, isBottle)
        return True


    def select_targets(self):
        """
            Wybiera dwa słowniki o największych wartościach 'count' z self.TRG_CANDIDATES,
            usuwa resztę z listy i zwraca listę tych dwóch słowników.
            Jeśli kandydatów jest mniej niż dwóch, zwraca tyle ile jest.
        """
        if self.TRG_CANDIDATES==[]:
            return None
        # Sortuj malejąco po 'count'
        sorted_candidates = sorted(self.TRG_CANDIDATES, key=lambda t: t["count"], reverse=True)
        # Usuń drugiego kandydata jeśli oba są takie samme, powtarzaj aż będą różne lub zostanie jeden
        while len(sorted_candidates) > 1 and (sorted_candidates[0]["isBottle"] == sorted_candidates[1]["isBottle"]):
            sorted_candidates.pop(1)
        top_two = sorted_candidates[:2]
        # Zastąp listę tylko tymi dwoma
        self.TRG_CANDIDATES = top_two
        return top_two


    def process_landing_sites(self, sites: list[tuple[float, float]]) -> bool:
        """
        Przyjmuje listę lądowisk od drona i wysyła waypoints zrzutu do autopilota.
        Args:
            sites: lista tupli (lat, lon)
        """
        if not sites:
            self.logger.warning("process_landing_sites: empty sites list")
            return False

        attitude = self.drone.get_attitude()
        if attitude is None:
            self.logger.error("process_landing_sites: no attitude data")
            return False

        container = []
        yaw = attitude[2]

        for lat, lon in sites:
            drop_point = self.calc_drop_coords({"lat": lat, "lon": lon, "isBottle": False})
            self.calc_drop_waypoints(drop_point, yaw, container)

        ok = self.drone.append_waypoints(container)
        self.logger.info(f"process_landing_sites: sent {len(container)} waypoints, ok={ok}")
        return ok


    def calc_drop_coords(self, trg_dict: dict) -> Tuple[float, float]:
        ''' 
        Oblicza współrzędne GPS punktu zrzutu na podstawie pozycji celu i kierunku nalotu.
        '''
        # 1. Wybór parametrów (bez zmian)
        if trg_dict.get("isBottle", False):
            x_trans, y_trans = cfg.drops.bottle.x_translation, cfg.drops.bottle.y_translation
            course_deg = cfg.drops.bottle.drop_course
        else:
            x_trans, y_trans = cfg.drops.beacon.x_translation, cfg.drops.beacon.y_translation
            course_deg = cfg.drops.beacon.drop_course

        lat_center = trg_dict["lat"]
        lon_center = trg_dict["lon"]
        yaw_rad = math.radians(course_deg)

        # Obrót współrzędnych (NED)
        d_north = x_trans * math.cos(yaw_rad) - y_trans * math.sin(yaw_rad)
        d_east = x_trans * math.sin(yaw_rad) + y_trans * math.cos(yaw_rad)

        # Pobieramy precyzyjne przeliczniki dla tego konkretnego miejsca
        m_per_lat, m_per_lon = MissionService.get_meters_per_degree(lat_center)

        delta_lat = d_north / m_per_lat
        delta_lon = d_east / m_per_lon

        return {
        "lat": lat_center + delta_lat,
        "lon": lon_center + delta_lon,
        "isBottle": trg_dict.get("isBottle")
        }


    
    def calc_drop_waypoints(self, drop_point: dict, yaw: float, container: list = [], alt: float = cfg.drops.altitude) -> list:

        """
        Args: 
            drop_point['lat'], drop_point['lon'] = punkt, w którym ma otworzyć się serwo
            yaw = kierunek nalotu w radianach
            container = [{
                "command": "WAYPOINT",
                "lat": wp_lat,
                "lon": wp_lon,
                "alt": alt,
                "acr": acr
            }]
        """

        dist_and_acr = [
            (100, 15),   
            (40, 15),   
            (0, 5),     
            (-40, 15)   
        ]

        for d, acr in dist_and_acr:
            # LOGIKA: 
            # Jeśli d=80 (przed zrzutem), musimy lecieć w kierunku przeciwnym do yaw (yaw + 180)
            # Jeśli d=-40 (za zrzutem), lecimy zgodnie z yaw.
            # Możemy to uprościć: zawsze używamy yaw, ale dystans d określa przesunięcie.
            # Ale gps_newpos nie lubi ujemnych dystansów, więc:
            
            if d >= 0:
                bearing = (yaw + 180) % 360     # Odsuwamy się do tyłu od punktu zrzutu
                dist = d
            else:
                bearing = yaw                   # Przesuwamy się do przodu za punkt zrzutu
                dist = abs(d)

            wp_lat, wp_lon = mavextra.gps_newpos(
                drop_point["lat"],
                drop_point["lon"],
                bearing,
                dist
            )

            
            container.append({
                "command": "WAYPOINT",
                "lat": wp_lat,
                "lon": wp_lon,
                "alt": alt,
                "acr": acr
            })

            if d == 0:
                if drop_point.get("isBottle"): 
                    container.append({
                        "command": "SET_SERVO",
                        "channel": MatekService.BOTTLE_SERVO_CHANNEL,
                        "pwm": MatekService.PWM_DROP_SERVO
                    })
                else:
                    container.append({
                        "command": "SET_SERVO",
                        "channel": MatekService.BEACON_SERVO_CHANNEL,
                        "pwm": MatekService.PWM_DROP_SERVO
                    })

        return container

    
    

    @staticmethod
    def get_meters_per_degree(lat):
        """
        Oblicza dokładną liczbę metrów na stopień szerokości i długości 
        geograficznej dla danej szerokości (model WGS84).
        """
        # Parametry elipsoidy WGS84
        a = 6378137.0
        e2 = 0.00669437999014
        
        lat_rad = math.radians(lat)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        
        # Promień krzywizny południka (North-South)
        m_per_lat_rad = (a * (1 - e2)) / (1 - e2 * sin_lat**2)**1.5
        # Promień krzywizny równoleżnika (East-West)
        m_per_lon_rad = (a * cos_lat) / math.sqrt(1 - e2 * sin_lat**2)
        
        # Konwersja na metry na stopień (z metrów na radian)
        deg_to_rad = math.pi / 180.0
        return m_per_lat_rad * deg_to_rad, m_per_lon_rad * deg_to_rad
    
    @staticmethod
    def get_distance_meters(lat1, lon1, lat2, lon2):
        """Oblicza dystans w metrach między dwoma punktami (uproszczony rzut płaski)."""
        m_per_lat, m_per_lon = MissionService.get_meters_per_degree(lat1)
        dy = (lat1 - lat2) * m_per_lat
        dx = (lon1 - lon2) * m_per_lon
        return math.sqrt(dx*dx + dy*dy)

    @staticmethod
    def load_Poly(filename) -> list[tuple[float, float]]:
        '''
        Loads polygon vertices from a .poly file
        
        Returns:
            List of (lat, lon) tuples
        '''
        polygon = []
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                lat_str, lon_str = line.split()
                polygon.append((float(lat_str), float(lon_str)))
        return polygon

    def calc_release_point(
        self, 
        target: dict, 
        yaw: float, 
        speed: float, 
        altitude_agl: float, 
        solver,  # ShootingSolver instance
        env # simulation environment instance
    ) -> dict:
        """
        Wyznacza punkt GPS, który wyliczyła symulacja by trafić w cel.
        
        Args:
            target: Słownik z kluczami 'lat', 'lon', 'isRed' reprezentujący cel
            yaw: Kierunek podejścia w radianach (0 = North, pi/2 = East)
            speed: Prędkość samolotu w m/s
            altitude_agl: Wysokość nad celem w metrach (AGL)
            solver: Instancja klasy ShootingSolver, która ma metodę calculate_release_point()
        Returns:
            Słownik z kluczami 'lat', 'lon', 'isRed', 'original_target_lat', 'original_target_lon'
            reprezentujący punkt GPS do zrzutu oraz informacje o celu.
        """
        # skłądowe prędkości w lokalnym ukladzie odniesienia
        vx = speed * np.cos(yaw)
        vy = speed * np.sin(yaw)
        vz = 0.0  # zakładamy lot poziomy
        approach_velocity = np.array([vx, vy, vz])

        # liczymy od punku (0,0) i potem zamienimy displacement na GPS
        target_local = np.array([0.0, 0.0])
        
        offset_meters = solver.calculate_release_point(
            target_position=target_local,
            approach_altitude=altitude_agl,
            approach_velocity=approach_velocity,
            env=env
        )
        
        # offset_meters to wektor [north_offset, east_offset] w metrach, który mówi jak daleko od celu (w lokalnym układzie) powinien być punkt zrzutu        
        # konvertujemy na GPS
        release_lat, release_lon = mavextra.gps_offset(
            target["lat"], 
            target["lon"], 
            offset_meters[0], 
            offset_meters[1]  
        )
        
        # Zwracamy punkt zrzutu wraz z informacją o celu (kolor, oryginalne współrzędne) 
        return {
            "lat": release_lat,
            "lon": release_lon,
            "isRed": target["isRed"],
            "original_target_lat": target["lat"], 
            "original_target_lon": target["lon"]
        }    



        def process_landing_sites(self, sites: list[tuple[float, float]]):
            """
            Przyjmuje listę lądowisk od drona i wysyła waypoints zrzutu do autopilota. Do przemyślenia logika jak to chcemy robić,
            na razie testuję czy wgl cały pipeline działa
            Args:
                sites: lista tupli (lat, lon)
            """
            container = []
            yaw = self.drone.get_attitude()[2] 

            for lat, lon in sites:
                drop_point = self.calc_drop_coords({"lat": lat, "lon": lon, "isBottle": False})
                self.calc_drop_waypoints(drop_point, yaw, container)

            self.drone.append_waypoints(container)
            print(f"[MissionService] Wysłano {len(container)} waypointów zrzutu")


if __name__ == "__main__":
    a=1
    mission = MissionService(a, (1280, 720))
    ll = [{"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True }]
    
    pp = [{"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True }]

    for i in mission.NEW_MISSION:
        print(i)

    mission.prepare_wps(ll, pp)
    for i in mission.NEW_MISSION:
        print(i)
