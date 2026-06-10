import sys
from pymavlink import mavutil
 
if len(sys.argv) != 2:
    print("Użycie: python conn_test.py <port>")
    sys.exit(1)
 
port = sys.argv[1]
 
m = mavutil.mavlink_connection(f'tcpin:0.0.0.0:{port}')
while True:
    msg = m.recv_match(type=['STATUSTEXT', 'HEARTBEAT', 'MISSION_REQUEST'], blocking=True, timeout=2)
    if msg:
        print(msg)
 