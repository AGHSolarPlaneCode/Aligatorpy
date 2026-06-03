python -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcpin:0.0.0.0:5762')
while True:
    msg = m.recv_match(type=['STATUSTEXT','HEARTBEAT','MISSION_REQUEST'], blocking=True, timeout=2)
    if msg:
        print(msg)
"
