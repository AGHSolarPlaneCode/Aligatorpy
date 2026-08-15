#!/usr/bin/env python3
"""
capture_frames_no_gps.py
=========================
Simplified frame-capture script for the AR0234 camera (ArduCam B0429,
Jetson Orin Nano). Just shoots raw frames as fast as possible — no MAVLink
connection, no flight controller, no GPS/attitude tagging.

Use this when you want quick raw test data (e.g. bench testing, indoor runs,
or any time the flight controller isn't connected) without the overhead and
requirements of capture_flight_frames.py. For frames tagged with GPS/attitude
during an actual flight, use capture_flight_frames.py instead.

Frames are written to ~/Documents/flights/no_gps/{date and time of the run}
(override the parent with --output-dir), plus a lightweight CSV
(Filename, Index, Timestamp) — no GPS/attitude columns, since there's no
flight controller to get them from.

REQUIREMENTS:
  numpy, the v4l2-ctl tool on PATH.

RECORDING LENGTH CONTROL (--frames):
  --frames N   (N > 0) stops after exactly N frames.
  --frames 0   (default) runs with no limit, until Ctrl+C.
  No artificial sleep between frames — runs as fast as v4l2-ctl allows.

EXAMPLES:
  # Unlimited frames, until Ctrl+C
  python3 capture_frames_no_gps.py

  # Exactly 200 frames
  python3 capture_frames_no_gps.py --frames 200
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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=int, default=0,
                         help="Number of frames to collect (0 = no limit, until Ctrl+C)")
    parser.add_argument("--cam-device", default=CAM_DEVICE_DEFAULT, help="Camera V4L2 device")
    parser.add_argument("--exposure", type=int, default=500, help="ExposureTime (v4l2-ctl units)")
    parser.add_argument("--gain", type=int, default=100, help="analogue_gain (v4l2-ctl units)")
    parser.add_argument("--warmup-frames", type=int, default=3,
                         help="Throwaway frames after setting exposure/gain, before real recording starts")
    parser.add_argument("--output-dir", default="~/Documents/flights/no_gps",
                         help="Parent directory for flight folders (a folder named after the current date/time "
                              "is created inside it for each run)")
    parser.add_argument("--log-every", type=int, default=10, help="How often (in frames) to print status")
    args = parser.parse_args()

    if not os.path.exists(args.cam_device):
        print(f"[error] Camera device {args.cam_device} not found. Check the connection / v4l2-ctl --list-devices")
        sys.exit(1)

    # --- Set up the flight folder ---
    # Layout: ~/Documents/flights/no_gps/{date and time of this run}
    output_root = Path(os.path.expanduser(args.output_dir))
    mission_dir = output_root / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    mission_dir.mkdir(parents=True, exist_ok=True)
    csv_path = mission_dir / "frames.csv"
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

    # --- Capture loop ---
    frame_iter = range(args.frames) if args.frames > 0 else count()
    saved = 0
    t_start = time.time()

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Filename", "Index", "Timestamp"])

        try:
            for i in frame_iter:
                ts = datetime.now().strftime("%m-%d_%H:%M:%S:%f")[:-3]
                raw_name = f"IMG_{i:04d}_{ts}.raw"
                raw_path = mission_dir / raw_name

                if not capture_one_frame(args.cam_device, str(raw_path), timeout_s=5):
                    continue
                if not validate_frame(str(raw_path)):
                    continue

                writer.writerow([raw_name, i, ts])
                f.flush()
                saved += 1

                if saved % args.log_every == 0:
                    elapsed = time.time() - t_start
                    fps = saved / elapsed if elapsed > 0 else 0.0
                    print(f"[{saved} frames] {fps:.2f} avg FPS")

        except KeyboardInterrupt:
            print("\n[info] Interrupted (Ctrl+C).")

    elapsed = time.time() - t_start
    fps = saved / elapsed if elapsed > 0 else 0.0
    print(f"[done] Saved {saved} frames in {elapsed:.1f}s ({fps:.2f} avg FPS)")
    print(f"[done] Data in: {mission_dir}")


if __name__ == "__main__":
    main()
