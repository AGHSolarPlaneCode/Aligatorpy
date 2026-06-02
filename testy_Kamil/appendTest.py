 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService



drone = MatekService(device="/dev/ttyAMA0")
drone_mission = MissionService(drone) 
print("Inicjalizacja programu")


waypoints = [
    {"command": "WAYPOINT", "lat": -35.363261, "lon": -100.165230, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
]
append = [
    {"command": "WAYPOINT", "lat": -35.363800, "lon": 100.165800, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.364000, "lon": 110.166000, "alt": 25, "acr": 0},
]


drone.append_waypoints(append)
