import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
 
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
 
# Połączenie z dronem (SITL lub realny FC)
drone = MatekService(device="tcp:192.168.161.240:5771")
mission = MissionService(drone)
 
# Testowe lądowiska - 4 punkty (2 pary)
sites = [
    (50.2683316, 18.6695051),
    (50.2707181, 18.6691618),
    (50.270752, 18.676794),
    (50.2685510, 18.6979151),
]
 
ok = mission.process_landing_sites_drone(sites)
print("process_landing_sites_drone ok =", ok)
 
print("Done")