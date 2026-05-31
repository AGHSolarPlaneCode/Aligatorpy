"""
Część dronowa komunikacji do obsługi połączenia samolot-dron.

Skrypt należy odpalić na laptopie który obsługuje MissionPlannera drona i w ustawieniach Mavlinkowych MissionPlannera (Ctrl+F) w połączeniach dodać:
- outbound: port laptopa samolotowego (nie zaznaczać write!), połączenie przekazuje tylko telemetrię drona do samolotu

"""
import socket, threading, argparse, sys
 
stop_event = threading.Event()
 
def forward(src, dst):
    src.settimeout(1.0)
    while not stop_event.is_set():
        try:
            data = src.recv(4096)
            if not data: break
            dst.sendall(data)
        except socket.timeout:
            continue
        except: break
 
parser = argparse.ArgumentParser()
parser.add_argument("--listen-port", type=int, required=True)
parser.add_argument("--forward-host", required=True)
parser.add_argument("--forward-port", type=int, required=True)
args = parser.parse_args()
 
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.settimeout(1.0)
server.bind(("0.0.0.0", args.listen_port))
server.listen(1)
print(f"Czekam na MP na porcie {args.listen_port}...")
 
try:
    mp = None
    while not stop_event.is_set():
        try:
            mp, _ = server.accept()
            break
        except socket.timeout:
            continue
 
    if mp:
        print("MP połączony, łączę się z drugim laptopem...")
        remote = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        remote.connect((args.forward_host, args.forward_port))
        print("Połączono!")
 
        threading.Thread(target=forward, args=(mp, remote), daemon=True).start()
        forward(remote, mp)
 
except KeyboardInterrupt:
    pass
finally:
    stop_event.set()
    print("\n[Bridge] Zatrzymano.")
    sys.exit(0)
 