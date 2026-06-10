import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService

plane = MatekService(device="tcp:localhost:5771")
mission = MissionService(plane)

sites = [(50.2683316, 18.6695051),
    (50.2707181, 18.6691618),
    (50.270752, 18.676794),
    (50.2685510, 18.6979151)]
loiter_points = [
    (50.2683316, 18.6695051),
    (50.2707181, 18.6691618),
    (50.270752, 18.676794),
    (50.2685510, 18.6979151),
    (50.2697031, 18.6978722),
    (50.2683316, 18.6695051)
]

mission.process_landing_sites(sites, loiter_points)

print("Done")