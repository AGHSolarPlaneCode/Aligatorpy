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
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from pymavlink import mavutil
from Application.Logger.log_module import get_logger
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService

logger = get_logger(__name__)

zone_points = [
    (50.272217, 18.670302),
    (50.271481, 18.665653),
    (50.268558, 18.665451),
    (50.267404, 18.669445),
    (50.267695, 18.67982),
    (50.267561, 18.689233),
    (50.266878, 18.700921),
    (50.271334, 18.701793),
    (50.271067, 18.687992),
    (50.270752, 18.676794),
]

loiter_points = [
    (50.2683316, 18.6695051),
    (50.2707181, 18.6691618),
    (50.2694014, 18.6771655),
    (50.2705535, 18.6981297),
    (50.2685510, 18.6979151),
    (50.2683316, 18.6695051)
]

break_points = [
    (50.2685648,18.6733460)
]
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
        logger.info("[LANDING] Zapisano lądowiska do landing_sites.txt")
        mission.process_landing_sites(landing_sites, loiter_points, break_points)
        logger.info("[LANDING] Gotowe")
    except Exception as e:
        logger.info(f"[LANDING] Błąd: {type(e).__name__}: {e}")

def main():
    global landing_sites, collecting
    args = parse_args()

    while True:
        try:
            logger.info(f"[MAVLink] Łączę się z autopilotem: {args.plane_device}...")
            plane = MatekService(device=args.plane_device, baud=args.plane_baud)
            mission = MissionService(plane)
            logger.info("[MAVLink] Połączono z autopilotem")
            break
        except Exception as e:
            logger.info(f"[MAVLink] Błąd: {type(e).__name__}: {e}, retry za 3s...")
            time.sleep(3)

    logger.info(f"[Bridge] Nasłuchuję na porcie {args.listen_port}...")
    mav = mavutil.mavlink_connection(
        f"tcpin:0.0.0.0:{args.listen_port}",
        source_system=255
    )

    logger.info("[Bridge] TCP podłączony / czekam na wiadomości MAVLink...")

    try:
        while True:
            
            msg = mav.recv_match(blocking=True, timeout=1)
            if not msg:
                continue

            if msg.get_type() == 'HEARTBEAT':
                logger.info(f"[HEARTBEAT] mode: {msg.custom_mode}")
                continue

            if msg.get_type() != 'STATUSTEXT':
                continue

            text = msg.text.strip()
            logger.info(f"[STATUSTEXT] '{text}' (len={len(text)})")

            if text == "LANDING_START":
                landing_sites = []
                collecting = True
                logger.info("[LANDING] Start zbierania lądowisk")
            elif text.startswith("LANDING:") and collecting:
                try:
                    _, coords = text.split(":", 1)
                    lat, lon = map(float, coords.split(",", 1))
                    landing_sites.append((lat, lon))
                    logger.info(f"[LANDING] Dodano: {lat}, {lon}")
                except Exception as e:
                    logger.info(f"[LANDING] Błąd parsowania: {text} -> {type(e).__name__}: {e}")
            elif text == "LANDING_END" and collecting:
                collecting = False
                logger.info("[LANDING] Koniec zbierania lądowisk")
                handle_landing(mission)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            mav.close()
        except Exception:
            pass
        logger.info("\n[Bridge] Zatrzymano.")
        sys.exit(0)

if __name__ == "__main__":
    main()
