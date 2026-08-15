#!/usr/bin/env python3
"""
debayer_flight_frames.py
==========================
Post-processes raw frames captured by `capture_flight_frames.py` or
`capture_frames_no_gps.py` — debayers (+ optionally calibrates, denoises)
every `.raw` file in a flight folder and writes `.jpg` output, using the
existing ISP pipeline from the `ar0234_isp` repo (imports `process()` from
`ar0234_isp/process/process_raw.py` — no logic is duplicated here).

Works on folders produced by either capture script:
    ~/Documents/flights/gps/{date_time}/      (has photos_position.csv)
    ~/Documents/flights/no_gps/{date_time}/   (has frames.csv)

Debayered `.jpg` files are written into a `debayered/` subfolder inside the
flight folder, alongside a copy of the original CSV with the `Filename`
column updated to point at the new `.jpg` names — so e.g.
`ImageMosaicService` (Aligatorpy) can be pointed straight at
`debayered/photos_position.csv` without further editing. If no CSV is
present in the flight folder, frames are still processed; only the
per-frame `.jpg` files are written.

Already-processed frames are skipped on re-run (pass --overwrite to redo
everything). A single corrupt/unreadable frame doesn't stop the batch — it's
reported and skipped.

REQUIREMENTS:
  opencv-python, numpy, and a local checkout of the ar0234_isp repo.

EXAMPLES:
  # Debayer a GPS-tagged flight, with calibration (needs master_dark.bin /
  # gain_map.bin to exist in ar0234_isp/calib)
  python3 debayer_flight_frames.py ~/Documents/flights/gps/2026-08-06_14-30-05

  # Debayer a no_gps flight without calibration data available
  python3 debayer_flight_frames.py ~/Documents/flights/no_gps/2026-08-06_15-02-11 \\
      --no-calibrate

  # Redo everything, custom bayer code / ISP repo location
  python3 debayer_flight_frames.py ~/Documents/flights/gps/2026-08-06_14-30-05 \\
      --overwrite --bayer-code GR --isp-repo ~/isp_npp/Antek_isp/iza/ar0234_isp
"""

import argparse
import csv
import os
import sys
import time
from pathlib import Path

GPS_CSV_NAME = "photos_position.csv"
NO_GPS_CSV_NAME = "frames.csv"


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


def find_csv(flight_dir: Path):
    """Returns (csv_path, kind) where kind is 'gps', 'no_gps', or None if no CSV found."""
    gps_csv = flight_dir / GPS_CSV_NAME
    no_gps_csv = flight_dir / NO_GPS_CSV_NAME
    if gps_csv.exists():
        return gps_csv, "gps"
    if no_gps_csv.exists():
        return no_gps_csv, "no_gps"
    return None, None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("flight_dir",
                         help="Flight folder to process, e.g. ~/Documents/flights/gps/2026-08-06_14-30-05")
    parser.add_argument("--isp-repo", default="~/isp_npp/Antek_isp/iza/ar0234_isp",
                         help="Path to the ar0234_isp repo")
    parser.add_argument("--bayer-code", choices=["GR", "BG", "RG", "GB"], default="GB",
                         help="Bayer pattern code passed to process() (default GB)")
    parser.add_argument("--calibrate", dest="calibrate", action="store_true", default=True,
                         help="Apply dark/gain calibration if available (default)")
    parser.add_argument("--no-calibrate", dest="calibrate", action="store_false",
                         help="Skip calibration — debayer only")
    parser.add_argument("--target-mean", type=float, default=0.15, help="Target mean brightness for AWB/gamma step")
    parser.add_argument("--sat-boost", type=float, default=1.35, help="Saturation multiplier")
    parser.add_argument("--denoise", choices=["bilateral", "nlm"], default=None,
                         help="Optional denoise pass after debayer (nlm is usually better but slower)")
    parser.add_argument("--denoise-strength", type=float, default=7, help="Denoise strength")
    parser.add_argument("--output-subdir", default="debayered",
                         help="Subfolder (inside the flight folder) to write .jpg output and CSV into")
    parser.add_argument("--overwrite", action="store_true",
                         help="Reprocess frames even if the output .jpg already exists")
    parser.add_argument("--sharpen", type=float, default=1.0,
                         help="Unsharp mask amount (0 = off)")
    parser.add_argument("--sharpen-radius", type=float, default=3)
    parser.add_argument("--channel-gains", type=float, nargs=3,
                          metavar=("R", "G", "B"), default=[1.5922, 1.0, 1.3440])
    args = parser.parse_args()

    flight_dir = Path(os.path.expanduser(args.flight_dir))
    if not flight_dir.is_dir():
        print(f"[error] Flight folder not found: {flight_dir}")
        sys.exit(1)

    import cv2  # imported here so --help works without opencv installed

    isp_process = load_isp_process_fn(args.isp_repo)
    print(f"[info] ISP pipeline loaded from {os.path.expanduser(args.isp_repo)}")

    out_dir = flight_dir / args.output_subdir
    out_dir.mkdir(parents=True, exist_ok=True)

    raw_files = sorted(
        p for p in flight_dir.glob("*.raw")
        if not p.name.startswith("_")  # skip leftover _warmup.raw etc.
    )
    if not raw_files:
        print(f"[error] No .raw frames found in {flight_dir}")
        sys.exit(1)
    print(f"[info] Found {len(raw_files)} raw frames in {flight_dir}")

    csv_path, kind = find_csv(flight_dir)
    csv_rows = {}
    header = None
    if csv_path:
        print(f"[info] Found {kind} CSV: {csv_path.name}")
        with open(csv_path, newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                if row:
                    csv_rows[row[0]] = row
    else:
        print("[info] No CSV found in this flight folder — processing raw frames only")

    processed = 0
    skipped = 0
    failed = 0
    t_start = time.time()

    for raw_path in raw_files:
        jpg_name = raw_path.stem + ".jpg"
        jpg_path = out_dir / jpg_name

        if jpg_path.exists() and not args.overwrite:
            skipped += 1
            # still carry the row forward into the new CSV if we have one
            if raw_path.name in csv_rows:
                row = list(csv_rows[raw_path.name])
                row[0] = jpg_name
                csv_rows[raw_path.name] = row
            continue

        try:
            bgr8 = isp_process(
                str(raw_path),
                bayer_code=args.bayer_code,
                calibrate=args.calibrate,
                target_mean=args.target_mean,
                sat_boost=args.sat_boost,
                denoise_method=args.denoise,
                denoise_strength=args.denoise_strength,
                sharpen_amount=args.sharpen,          # <-- dodaj
                sharpen_radius=args.sharpen_radius,   # <-- dodaj
                channel_gains=args.channel_gains,     # <-- dodaj
            )
            cv2.imwrite(str(jpg_path), bgr8)
            processed += 1
        except Exception as e:
            print(f"[warn] Failed to debayer {raw_path.name}: {e}")
            failed += 1
            continue

        if raw_path.name in csv_rows:
            row = list(csv_rows[raw_path.name])
            row[0] = jpg_name
            csv_rows[raw_path.name] = row

        if (processed + skipped) % 10 == 0:
            elapsed = time.time() - t_start
            print(f"[{processed + skipped}/{len(raw_files)}] {elapsed:.1f}s elapsed")

    if csv_path and header:
        new_csv_path = out_dir / csv_path.name
        with open(new_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for raw_path in raw_files:
                row = csv_rows.get(raw_path.name)
                if row:
                    writer.writerow(row)
        print(f"[info] Wrote updated CSV: {new_csv_path}")

    elapsed = time.time() - t_start
    print(f"[done] Processed {processed}, skipped {skipped} (already done), failed {failed} "
          f"— in {elapsed:.1f}s")
    print(f"[done] Output in: {out_dir}")


if __name__ == "__main__":
    main()
