przykład użycia dodawania zawisu

waypoints = [
    {"command": "TAKEOFF", "alt": 50},
    {
        "command": "NAV_LOITER_UNLIM",
        "lat": 40.736,
        "lon": 30.073,
        "alt": 50,
        "radius": 100,   # opcjonalnie, domyślnie 0
        "yaw": 0,        # opcjonalnie
    },
    {
        "command": "NAV_LOITER_TIME",
        "lat": 40.737,
        "lon": 30.074,
        "alt": 50,
        "time": 30,      # wymagane — sekundy
        "radius": 80,
    },
    {"command": "WAYPOINT", "lat": 40.738, "lon": 30.075, "alt": 50, "acr": 0},
]