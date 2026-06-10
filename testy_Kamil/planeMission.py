import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService

plane = MatekService(device="tcp:localhost:5761")
mission = MissionService(plane)

sites = [(50.1, 19.1), (50.2, 19.2)]
mission.process_landing_sites(sites)

print("Done")