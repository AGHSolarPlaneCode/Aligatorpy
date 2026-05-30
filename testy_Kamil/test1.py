 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService



drone = MatekService(device="tcp:192.168126.32:5763")
drone_mission = MissionService(drone) 
print("Inicjalizacja programu")



print(drone.get_current_mode())

waypoints = [
    {"command": "WAYPOINT", "lat": -35.363261, "lon": -100.165230, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
]


droppoint = {"command": "WAYPOINT", "lat": -33.0000, "lon": 149.165230, "alt": 20, "acr": 0}

#prosta implementacja dodania zrzutu do obecnej misji

mission = drone.get_mission()

drone.set_waypoints(waypoints)
print("Mission read successfully")
cur_waypoint = drone.get_mission_status()



container = drone_mission.calc_drop_waypoints(droppoint, yaw=10, container=[])
new_mission = drone.add_drop_sequence(container)
drone.append_waypoints(new_mission)
drone.set_current_waypoint(cur_waypoint)
drone.set_mode("AUTO")





