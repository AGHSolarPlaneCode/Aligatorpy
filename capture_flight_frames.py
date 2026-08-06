#!/usr/bin/env python3
"""
capture_flight_frames.py
=========================
Collects frames from the AR0234 camera (ArduCam B0429, Jetson Orin Nano)
during flight, tagged with GPS/attitude data from the flight controller
(MAVLink — Orange Cube or any other ArduPilot/pymavlink-compatible FC).

Output goes into ~/Documents/flights/gps/{date and time of the flight}
(override the parent with --output-dir), along with a CSV file using the
same schema as `photos_position.csv` from the Aligatorpy project (Filename,
Index, Lat, Lon, Alt, Roll, Pitch, Yaw), so it can later be used with e.g.
ImageMosaicService to build a map from the frames (once debayered, if saved
as .raw).

See also capture_frames_no_gps.py — a simpler variant with no MAVLink
dependency, for quick raw-frame collection when a flight controller isn't
connected or GPS tagging isn't needed. It saves to
~/Documents/flights/no_gps/{date and time} instead.

TWO SAVE MODES (--format):
  raw   Only writes the raw .raw file (BA10, 16-bit/pixel) via v4l2-ctl,
        with no processing in the loop. Lightest possible loop -> highest
        achievable FPS. Debayer/calibrate offline afterwards, e.g. with
        process_raw.py from ar0234_isp.
  jpg   For every frame, immediately runs a full debayer (+ calibration,
        if master_dark.bin / gain_map.bin are available) through the
        pipeline in ar0234_isp/process/process_raw.py and saves a .jpg.
        Slower, lower achievable FPS, but you get ready-to-view preview
        images right away.

RECORDING LENGTH CONTROL (--frames):
  --frames N   (N > 0) stops after exactly N frames.
  --frames 0   (default) runs with no limit, until Ctrl+C.
  In both cases there is NO artificial sleep between frames — it runs as
  fast as v4l2-ctl allows (and, in jpg mode, processing time allows).

REQUIREMENTS:
  pymavlink, numpy, the v4l2-ctl tool on PATH.
  For --format jpg additionally: opencv-python and a path to the
  ar0234_isp repo (--isp-repo), from which process_raw.py is imported.

EXAMPLES:
  # raw mode, no frame limit, until Ctrl+C
  python3 capture_flight_frames.py --mav-device /dev/ttyACM0 --format raw

  # jpg mode, exactly 300 frames
  python3 capture_flight_frames.py --mav-device /dev/ttyACM0 --format jpg \\
      --frames 300 --isp-repo ~/isp_npp/Antek_isp/iza/ar0234_isp
"""

import argparse
import csv
import os
import subprocess
import sys
import time
from datetime import datetime
from itertools import count
from pathlib import Path

import numpy as np
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Camera configuration (AR0234 / ArduCam B0429 on Jetson Orin Nano)
# ---------------------------------------------------------------------------
CAM_DEVICE_DEFAULT = "/dev/video0"
WIDTH = 1920
HEIGHT = 1200
PIXELS = WIDTH * HEIGHT
EXPECTED_BYTES = PIXELS * 2  # 16 bits/pixel (BA10, 10-bit packed into 16-bit)
FOURCC = "BA10"


def run(cmd, check=True, timeout=None):
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode != 0:
        print(f"[warn] Command failed: {' '.join(cmd)}")
        print(f"       stderr: {result.stderr.strip()}")
    return result


def set_v4l2_control(device, name, value):
    r = run(["v4l2-ctl", "-d", device, "-c", f"{name}={value}"], check=False)
    if r.returncode == 0:
        print(f"[info] Set {name}={value}")
    else:
        print(f"[warn] Failed to set {name}={value}: {r.stderr.strip()}")


def set_format(device, width=WIDTH, height=HEIGHT, fourcc=FOURCC):
    r = run(["v4l2-ctl", "-d", device,
             f"--set-fmt-video=width={width},height={height},pixelformat={fourcc}"])
    if r.returncode == 0:
        print(f"[info] Format set: {width}x{height} {fourcc}")


def capture_one_frame(device, out_path, timeout_s=5):
    """Captures exactly one frame to out_path via v4l2-ctl."""
    cmd = ["v4l2-ctl", "-d", device,
           "--stream-mmap",
           "--stream-count=1",
           f"--stream-to={out_path}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        print(f"[warn] Timeout while capturing frame: {out_path}")
        return False
    if r.returncode != 0:
        print(f"[warn] Frame capture failed: {r.stderr.strip()}")
        return False
    return True


def validate_frame(path):
    size = os.path.getsize(path)
    if size != EXPECTED_BYTES:
        print(f"[warn] Wrong frame size {path}: {size}B, expected {EXPECTED_BYTES}B — discarding")
        try:
            os.remove(path)
        except OSError:
            pass
        return False
    return True


# ---------------------------------------------------------------------------
# MAVLink — drone position and orientation (Orange Cube / any ArduPilot FC)
# ---------------------------------------------------------------------------
class FlightDataSource:
    """
    Keeps track of the most recent GLOBAL_POSITION_INT and ATTITUDE
    messages received from the FC.

    Unlike CameraService.py in Aligatorpy (which synchronizes each shot
    to a specific CAMERA_FEEDBACK message from the autopilot), here we
    simply use the most recently known position at the time each frame is
    saved. That's accurate enough for building a test dataset for
    detection/mapping work, and it's much simpler — and works identically
    no matter which FC is connected, Matek or Orange Cube, since both just
    speak MAVLink.
    """

    def __init__(self, device: str, baud: int):
        print(f"[info] Connecting to FC: {device} @ {baud}...")
        self.master = mavutil.mavlink_connection(device, baud=baud)
        self.master.wait_heartbeat()
        print(f"[info] Heartbeat OK (system {self.master.target_system}, "
              f"component {self.master.target_component})")

        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
        )
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1  # ATTITUDE
        )

        self._lat = self._lon = self._alt = None
        self._roll = self._pitch = self._yaw = None

    def poll(self):
        """Non-blocking read of any available messages, updates internal state."""
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                self._lat = msg.lat / 1e7
                self._lon = msg.lon / 1e7
                self._alt = msg.relative_alt / 1000.0  # AGL, meters
            elif t == "ATTITUDE":
                self._roll = msg.roll
                self._pitch = msg.pitch
                self._yaw = msg.yaw

    def snapshot(self):
        self.poll()
        return {
            "lat": self._lat, "lon": self._lon, "alt": self._alt,
            "roll": self._roll, "pitch": self._pitch, "yaw": self._yaw,
        }


# ---------------------------------------------------------------------------
# JPG mode — in-loop debayer via the ar0234_isp pipeline
# ---------------------------------------------------------------------------
def load_isp_process_fn(isp_repo: str):
    """Imports the process() function from ar0234_isp/process/process_raw.py."""
    process_dir = os.path.join(os.path.expanduser(isp_repo), "process")
    if not os.path.isdir(process_dir):
        raise FileNotFoundError(
            f"Could not find {process_dir} — check --isp-repo (path to the ar0234_isp repo)"
        )
    sys.path.insert(0, process_dir)
    from process_raw import process as isp_process  # type: ignore
    return isp_process


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--format", choices=["raw", "jpg"], default="raw",
                         help="raw = .raw only (fast), jpg = full debayer in-loop (slower)")
    parser.add_argument("--frames", type=int, default=0,
                         help="Number of frames to collect (0 = no limit, until Ctrl+C)")
    parser.add_argument("--cam-device", default=CAM_DEVICE_DEFAULT, help="Camera V4L2 device")
    parser.add_argument("--exposure", type=int, default=500, help="ExposureTime (v4l2-ctl units)")
    parser.add_argument("--gain", type=int, default=100, help="analogue_gain (v4l2-ctl units)")
    parser.add_argument("--warmup-frames", type=int, default=3,
                         help="Throwaway frames after setting exposure/gain, before real recording starts")
    parser.add_argument("--mav-device", required=True, help="Flight controller MAVLink port, e.g. /dev/ttyACM0")
    parser.add_argument("--mav-baud", type=int, default=115200, help="MAVLink baud rate")
    parser.add_argument("--output-dir", default="~/Documents/flights/gps",
                         help="Parent directory for flight folders (a folder named after the current date/time "
                              "is created inside it for each run)")
    parser.add_argument("--isp-repo", default="~/isp_npp/Antek_isp/iza/ar0234_isp",
                         help="Path to the ar0234_isp repo (only used for --format jpg)")
    parser.add_argument("--keep-raw", action="store_true",
                         help="In jpg mode, also keep the original .raw file (deleted by default after conversion)")
    parser.add_argument("--log-every", type=int, default=10, help="How often (in frames) to print status")
    args = parser.parse_args()

    if not os.path.exists(args.cam_device):
        print(f"[error] Camera device {args.cam_device} not found. Check the connection / v4l2-ctl --list-devices")
        sys.exit(1)

    isp_process = None
    if args.format == "jpg":
        import cv2  # noqa: F401  (validate opencv is available before we start flying)
        isp_process = load_isp_process_fn(args.isp_repo)
        print("[info] jpg mode — ISP pipeline loaded.")

    # --- Set up the flight folder ---
    # Layout: ~/Documents/flights/gps/{date and time of this flight}
    output_root = Path(os.path.expanduser(args.output_dir))
    mission_dir = output_root / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    mission_dir.mkdir(parents=True, exist_ok=True)
    csv_path = mission_dir / "photos_position.csv"
    print(f"[info] Flight folder: {mission_dir}")

    # --- Camera setup ---
    set_format(args.cam_device)
    set_v4l2_control(args.cam_device, "exposure", args.exposure)
    set_v4l2_control(args.cam_device, "analogue_gain", args.gain)

    warmup_path = mission_dir / "_warmup.raw"
    for _ in range(args.warmup_frames):
        capture_one_frame(args.cam_device, str(warmup_path), timeout_s=5)
    if warmup_path.exists():
        warmup_path.unlink()
    print(f"[info] Warmup ({args.warmup_frames} frames) done.")

    # --- Connect to the FC ---
    flight_data = FlightDataSource(args.mav_device, args.mav_baud)

    # --- Capture loop ---
    frame_iter = range(args.frames) if args.frames > 0 else count()
    saved = 0
    t_start = time.time()

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Filename", "Index", "Lat", "Lon", "Alt", "Roll", "Pitch", "Yaw"])

        try:
            for i in frame_iter:
                ts = datetime.now().strftime("%m-%d_%H:%M:%S:%f")[:-3]
                raw_name = f"IMG_{i:04d}_{ts}.raw"
                raw_path = mission_dir / raw_name

                if not capture_one_frame(args.cam_device, str(raw_path), timeout_s=5):
                    continue
                if not validate_frame(str(raw_path)):
                    continue

                pos = flight_data.snapshot()
                out_filename = raw_name

                if args.format == "jpg":
                    import cv2
                    try:
                        bgr8 = isp_process(str(raw_path))
                        jpg_name = f"IMG_{i:04d}_{ts}.jpg"
                        jpg_path = mission_dir / jpg_name
                        cv2.imwrite(str(jpg_path), bgr8)
                        out_filename = jpg_name
                    except Exception as e:
                        print(f"[warn] Debayer failed for {raw_name}: {e}")
                    finally:
                        if not args.keep_raw:
                            raw_path.unlink(missing_ok=True)

                writer.writerow([
                    out_filename, i,
                    pos["lat"], pos["lon"], pos["alt"],
                    pos["roll"], pos["pitch"], pos["yaw"],
                ])
                f.flush()
                saved += 1

                if saved % args.log_every == 0:
                    elapsed = time.time() - t_start
                    fps = saved / elapsed if elapsed > 0 else 0.0
                    gps_str = f"lat={pos['lat']}, lon={pos['lon']}" if pos["lat"] is not None else "GPS: no fix"
                    print(f"[{saved} frames] {fps:.2f} avg FPS | {gps_str}")

        except KeyboardInterrupt:
            print("\n[info] Interrupted (Ctrl+C).")

    elapsed = time.time() - t_start
    fps = saved / elapsed if elapsed > 0 else 0.0
    print(f"[done] Saved {saved} frames in {elapsed:.1f}s ({fps:.2f} avg FPS)")
    print(f"[done] Data in: {mission_dir}")


if __name__ == "__main__":
    main()
