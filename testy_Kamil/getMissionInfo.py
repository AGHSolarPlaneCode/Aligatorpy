# Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService



drone = MatekService(device="tcp:192.168.0.204:5763")
drone_mission = MissionService(drone) 
drone.set_mission_current_rate(10)

print("Inicjalizacja programu")
while True:
    drone.get_mission()
    #print("Current mode: ", drone.get_current_mode())
    #print("current waypoint: ", drone.get_mission_status())
    print("Attitude: ", drone.get_attitude())
    #print("cords: ", drone.get_current_coordinates())
    