import math
import numpy as np
from unittest.mock import MagicMock

# Importujemy Twoją klasę (upewnij się, że ścieżka się zgadza z Twoim projektem)
from Application.Services.MissionService import MissionService

def run_projection_tests():
    print("--- ROZPOCZYNAMY TESTY PROJEKCJI Z MISSSION SERVICE ---")

    # 1. Tworzymy sztucznego (zmockowanego) drona
    mock_drone = MagicMock()
    
    # 2. Inicjalizujemy serwis
    mission = MissionService(mock_drone)

    # 3. Nadpisujemy parametry kamery twardymi danymi (aby uniknąć problemów z config_loaderem)
    mission.image_width = 640
    mission.image_height = 400
    
    # Ważne: OpenCV wymaga, aby macierz K i dist były tablicami numpy, a nie zwykłymi listami
    MissionService.K = np.array([
        [346.2295, 0.0, 317.76],
        [0.0, 346.2628, 197.64],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    
    MissionService.dist = np.array([
        -0.34219431, 0.123526508, 0.00470056821, -0.000109821856, -0.0211505931
    ], dtype=np.float32)

    # 4. Ustawiamy pozycję bazową drona (np. Kraków)
    lat_uav = 50.0614300
    lon_uav = 19.9365800
    alt_uav = 50.0  # 50 metrów nad ziemią
    
    print(f"Pozycja drona: lat = {lat_uav}, lon = {lon_uav}, alt = {alt_uav}m\n")

    # ==============================================================
    # TEST 1: Dron w poziomie, sprawdzamy dokładnie środek matrycy
    # ==============================================================
    pixel_center = (320, 200) # Środek dla 640x400
    roll, pitch, yaw = 0.0, 0.0, 0.0

    print("[TEST 1] Dron poziomo, piksel: środek ekranu (320, 200)")
    result1 = mission.project_target_cords(pixel_center, lat_uav, lon_uav, alt_uav, roll, pitch, yaw)
    
    if result1:
        res_lat, res_lon = result1
        print(f"Wynik GPS: lat = {res_lat:.7f}, lon = {res_lon:.7f}")
        # Różnica powinna być minimalna (wynikająca jedynie z przesunięcia środka optycznego w macierzy K)
        lat_diff = res_lat - lat_uav
        lon_diff = res_lon - lon_uav
        print(f"Różnica względem drona: lat: {lat_diff:.7f}, lon: {lon_diff:.7f}\n")
    else:
        print("Błąd: Funkcja zwróciła None!\n")

    # ==============================================================
    # TEST 2: Dron pochylony w dół (Pitch = 15 stopni), środek matrycy
    # ==============================================================
    # W MAVLink/Pixhawk pitch dodatni oznacza zazwyczaj nos w górę, ujemny nos w dół (lub odwrotnie zależnie od układu NED).
    # Sprawdźmy pochylenie 15 stopni przeliczone na radiany:
    pitch_15 = math.radians(15.0) 
    
    print("[TEST 2] Dron pochylony o 15 stopni (Pitch), piksel: środek ekranu")
    result2 = mission.project_target_cords(pixel_center, lat_uav, lon_uav, alt_uav, roll, pitch_15, yaw)
    
    if result2:
        res_lat, res_lon = result2
        print(f"Wynik GPS: lat = {res_lat:.7f}, lon = {res_lon:.7f}")
        
        # Zgrubne przeliczenie przesunięcia w stopniach na metry (1 stopień to ok. 111.32 km)
        offset_m_lat = (res_lat - lat_uav) * 111320.0
        offset_m_lon = (res_lon - lon_uav) * 111320.0 * math.cos(math.radians(lat_uav))
        print(f"Przesunięcie na ziemi: ok. {offset_m_lat:.2f}m (N-S), ok. {offset_m_lon:.2f}m (E-W)")
    else:
        print("Błąd: Funkcja zwróciła None!")

if __name__ == "__main__":
    run_projection_tests()