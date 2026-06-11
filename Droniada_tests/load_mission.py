import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg

if len(sys.argv) != 2:
    print("Użycie: python3 load_mission_file.py <plik.waypoints>")
    sys.exit(1)

filename = sys.argv[1]


def load_waypoints_file(path):
    """Parsuje plik .waypoints (QGC WPL 110) i zwraca listę dictów dla set_waypoints"""
    waypoints = []
    with open(path, "r") as f:
        lines = f.readlines()

    cmd_map_rev = {16: "WAYPOINT", 183: "SET_SERVO", 19: "NAV_LOITER_TIME", 20: "RTL", 22: "TAKEOFF"}

    for line in lines[2:]:  # pomijamy nagłówek QGC WPL 110 i punkt HOME (seq 0)
        parts = line.strip().split("\t")
        if len(parts) < 12:
            continue

        cmd_id = int(parts[3])
        param1 = float(parts[4])
        param2 = float(parts[5])
        param3 = float(parts[6])
        lat = float(parts[8])
        lon = float(parts[9])
        alt = float(parts[10])

        cmd = cmd_map_rev.get(cmd_id)

        if cmd == "WAYPOINT":
            waypoints.append({
                "command": "WAYPOINT",
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "acr": param2,
            })
        elif cmd == "SET_SERVO":
            waypoints.append({
                "command": "SET_SERVO",
                "channel": int(param1),
                "pwm": int(param2),
            })
        elif cmd == "NAV_LOITER_TIME":
            waypoints.append({
                "command": "NAV_LOITER_TIME",
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "time": param1,
                "radius": param3,
            })
        elif cmd == "RTL":
            waypoints.append({"command": "RTL"})
        elif cmd == "TAKEOFF":
    	    waypoints.append({"command": "TAKEOFF", "alt": alt})
    return waypoints

#drone = MatekService(device="tcp:192.168.161.240:5771")
drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)

waypoints = load_waypoints_file(filename)
print(f"Wczytano {len(waypoints)} waypointów z {filename}")

ok = drone.set_waypoints(waypoints)
print("append_waypoints ok =", ok)
