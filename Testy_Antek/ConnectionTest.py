import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg

#print(cfg.mav.device)
#print(cfg.mav.baud)
print(f"Connecting to drone on {cfg.mav.device} with baud rate {cfg.mav.baud}")
drone = MatekService(device = "tcp:192.168.0.186:5763")
#drone = MatekService(device = cfg.mav.device, baud = cfg.mav.baud)

print(f"Connected to drone on {cfg.mav.device}")

#drone = MatekService(device = "tcp:192.168.0.186:5764")     #192.168.0.186
