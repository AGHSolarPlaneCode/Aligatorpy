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
    (50.2694014, 18.6771655),
    (50.2705535, 18.6981297),
    (50.2685510, 18.6979151),
    (50.2683316, 18.6695051)
]

break_points = [
    (50.2685373,18.6730886)
]


mission.process_landing_sites(sites, loiter_points, break_points)

print("Done")