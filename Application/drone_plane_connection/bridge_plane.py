"""
Część samolotowa komunikacji do obsługi połączenia samolot-dron

Pakiety mavlink są przekierowane po tcp z laptopa obsługującego drona do laptopa samolotowego. Laptop samolotowy po otrzymaniu pakietu współrzędnych lądowisk wykonuje funkcje

Użycie:
  python plane_bridge.py --listen-port 5763 --forward-host 172.20.10.2 --forward-port 5764 --plane-device tcp:localhost:5761

  --listen-port   : port na którym nasłuchujemy (od laptopa dronowego)
  --forward-host  : IP laptopa dronowego
  --forward-port  : port laptopa dronowego
  --plane-device  : połączenie do autopilota samolotu np. tcp:localhost:5761
"""

import socket
import threading
import argparse
import sys
from pymavlink import mavutil
from Application.Services.MatekService import MatekService

landing_sites = []
collecting = False
plane: MatekService = None
stop_event = threading.Event()


def parse_args():
    parser = argparse.ArgumentParser(description="Plane MAVLink Bridge")
    parser.add_argument("--listen-port", type=int, required=True)
    parser.add_argument("--forward-host", type=str, required=True)
    parser.add_argument("--forward-port", type=int, required=True)
    parser.add_argument("--plane-device", type=str, required=True,
                        help="Połączenie do autopilota samolotu np. tcp:localhost:5761")
    parser.add_argument("--plane-baud", type=int, default=57600)
    return parser.parse_args()


def handle_landing():
    global plane, mission

    if not landing_sites:
        print("[LANDING] Brak lądowisk")
        return

    mission.process_landing_sites(landing_sites)
    print("[LANDING] Gotowe")

def mavlink_listener(plane_device, plane_baud):
    global landing_sites, collecting, plane

    print(f"[MAVLink] Łączę się z autopilotem: {plane_device}")
    plane = MatekService(device=plane_device, baud=plane_baud)
    print(f"[MAVLink] Połączono z autopilotem")

    while not stop_event.is_set():
        msg = plane.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if not msg:
            continue

        text = msg.text.strip()
        print(f"[STATUSTEXT] {text}")

        if text == "LANDING_START":
            landing_sites = []
            collecting = True
            print("[LANDING] Start zbierania lądowisk")

        elif text.startswith("LANDING:") and collecting:
            try:
                _, coords = text.split(":")
                lat, lon = map(float, coords.split(","))
                landing_sites.append((lat, lon))
                print(f"[LANDING] Dodano: {lat}, {lon}")
            except Exception as e:
                print(f"[LANDING] Błąd parsowania: {text} -> {e}")

        elif text == "LANDING_END" and collecting:
            collecting = False
            print("[LANDING] Koniec zbierania lądowisk")
            handle_landing()


def forward(src, dst):
    src.settimeout(1.0)
    while not stop_event.is_set():
        try:
            data = src.recv(4096)
            if not data:
                break
            dst.sendall(data)
        except socket.timeout:
            continue
        except:
            break


def tcp_bridge(listen_port, forward_host, forward_port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(1.0)
    server.bind(("0.0.0.0", listen_port))
    server.listen(1)
    print(f"[Bridge] Nasłuchuję na porcie {listen_port}...")

    mp = None
    while not stop_event.is_set():
        try:
            mp, addr = server.accept()
            break
        except socket.timeout:
            continue

    if mp is None:
        return

    print("[Bridge] Mission Planner połączony")

    remote = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    remote.connect((forward_host, forward_port))
    print(f"[Bridge] Połączono z {forward_host}:{forward_port}")

    threading.Thread(target=forward, args=(mp, remote), daemon=True).start()
    forward(remote, mp)


if __name__ == "__main__":
    args = parse_args()

    t = threading.Thread(
        target=mavlink_listener,
        args=(args.plane_device, args.plane_baud),
        daemon=True
    )
    t.start()

    try:
        tcp_bridge(args.listen_port, args.forward_host, args.forward_port)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print("\n[Bridge] Zatrzymano.")
        sys.exit(0)