 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService




drone = MatekService(device="tcp:172.20.10.2:5761")

drone.send_landing_sites([
    (50.1, 19.1),
    (50.2, 19.2),
    (50.3, 19.3),
    (50.4, 19.4),
])

print("Przesłano lądowiska pomyślnie")
