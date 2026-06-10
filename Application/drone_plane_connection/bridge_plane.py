"""
Część samolotowa komunikacji do obsługi połączenia samolot-dron.

Użycie:
  python plane_bridge.py --listen-port 5765 --plane-device tcp:localhost:5761

  --listen-port  : port na którym nasłuchujemy (MP dronowy tu wysyła Outbound)
  --plane-device : połączenie do autopilota samolotu np. tcp:localhost:5761
"""
import argparse
import sys
import time
from pymavlink import mavutil
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService

landing_sites = []
collecting = False

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-port", type=int, required=True)
    parser.add_argument("--plane-device", type=str, required=True)
    parser.add_argument("--plane-baud", type=int, default=57600)
    return parser.parse_args()

def handle_landing(mission):
    if not landing_sites:
        print("[LANDING] Brak lądowisk")
        return


    try:
         # Zapisz lądowiska do pliku
        with open("landing_sites.txt", "a") as f:
            import datetime
            f.write(f"\n--- {datetime.datetime.now()} ---\n")
            for lat, lon in landing_sites:
                f.write(f"{lat},{lon}\n")
        print("[LANDING] Zapisano lądowiska do landing_sites.txt")
        mission.process_landing_sites(landing_sites)
        print("[LANDING] Gotowe")
    except Exception as e:
        print(f"[LANDING] Błąd: {type(e).__name__}: {e}")

def main():
    global landing_sites, collecting
    args = parse_args()

    while True:
        try:
            print(f"[MAVLink] Łączę się z autopilotem: {args.plane_device}...")
            plane = MatekService(device=args.plane_device, baud=args.plane_baud)
            mission = MissionService(plane)
            print("[MAVLink] Połączono z autopilotem")
            break
        except Exception as e:
            print(f"[MAVLink] Błąd: {type(e).__name__}: {e}, retry za 3s...")
            time.sleep(3)

    print(f"[Bridge] Nasłuchuję na porcie {args.listen_port}...")
    mav = mavutil.mavlink_connection(
        f"tcpin:0.0.0.0:{args.listen_port}",
        source_system=255
    )

    print("[Bridge] TCP podłączony / czekam na wiadomości MAVLink...")

    try:
        while True:
            
            msg = mav.recv_match(blocking=True, timeout=1)
            if not msg:
                continue

            if msg.get_type() == 'HEARTBEAT':
                print(f"[HEARTBEAT] mode: {msg.custom_mode}")
                continue

            if msg.get_type() != 'STATUSTEXT':
                continue

            text = msg.text.strip()
            print(f"[STATUSTEXT] '{text}' (len={len(text)})")

            if text == "LANDING_START":
                landing_sites = []
                collecting = True
                print("[LANDING] Start zbierania lądowisk")
            elif text.startswith("LANDING:") and collecting:
                try:
                    _, coords = text.split(":", 1)
                    lat, lon = map(float, coords.split(",", 1))
                    landing_sites.append((lat, lon))
                    print(f"[LANDING] Dodano: {lat}, {lon}")
                except Exception as e:
                    print(f"[LANDING] Błąd parsowania: {text} -> {type(e).__name__}: {e}")
            elif text == "LANDING_END" and collecting:
                collecting = False
                print("[LANDING] Koniec zbierania lądowisk")
                handle_landing(mission)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            mav.close()
        except Exception:
            pass
        print("\n[Bridge] Zatrzymano.")
        sys.exit(0)

if __name__ == "__main__":
    main()
