# Flight frame capture scripts

Two scripts for collecting camera frames from the AR0234 (ArduCam B0429) on
a Jetson Orin Nano, to build test data for object detection and
terrain-mapping work.

| Script | GPS/attitude tagging | Requires flight controller? |
|---|---|---|
| `capture_flight_frames.py` | Yes (MAVLink) | Yes |
| `capture_frames_no_gps.py` | No | No |

Both write raw `.raw` frames (BA10, 16-bit/pixel) via `v4l2-ctl` in a tight
loop with no artificial delay — as fast as the hardware allows.

## Output layout

```
~/Documents/flights/
├── gps/
│   └── 2026-08-06_14-30-05/            <- date & time the flight ran
│       ├── IMG_0000_08-06_14:30:05:123.raw
│       ├── IMG_0001_08-06_14:30:05:456.raw
│       ├── ...
│       └── photos_position.csv
└── no_gps/
    └── 2026-08-06_15-02-11/
        ├── IMG_0000_08-06_15:02:11:789.raw
        ├── ...
        └── frames.csv
```

The parent (`~/Documents/flights/gps` or `~/Documents/flights/no_gps`) can be
overridden with `--output-dir`; the date/time subfolder is always created
automatically for each run.

## `capture_flight_frames.py`

Tags every frame with the most recently received GPS position and attitude
from the flight controller over MAVLink (Orange Cube or any other
ArduPilot/pymavlink-compatible FC — the flight controller model doesn't
matter, only that it speaks MAVLink).

### Save modes (`--format`)

| Mode | What happens per frame | Speed |
|------|------------------------|-------|
| `raw` (default) | Only writes the raw `.raw` file. No processing in the loop. Debayer/calibrate later offline (e.g. with `process_raw.py`). | Fastest — highest achievable FPS |
| `jpg` | Runs the full ISP pipeline (debayer + calibration, if `master_dark.bin`/`gain_map.bin` are found) in-loop and writes a `.jpg`. | Slower, bounded by processing time per frame |

In `jpg` mode the intermediate `.raw` file is deleted after conversion by
default; pass `--keep-raw` to keep both.

### Requirements

- `pymavlink`, `numpy`
- `v4l2-ctl` available on `PATH` (part of `v4l-utils`)
- For `--format jpg` only: `opencv-python`, and a local checkout of the
  `ar0234_isp` repo (the script imports `process()` from
  `ar0234_isp/process/process_raw.py` — no logic is duplicated here)

### Recording length (`--frames`)

- `--frames 0` (default): no limit, runs until `Ctrl+C`.
- `--frames N` (N > 0): stops after exactly N frames.

### GPS/attitude tagging

Unlike `CameraService.py` in Aligatorpy, which synchronizes each shot to a
specific `CAMERA_FEEDBACK` MAVLink message, this script just tags each frame
with the most recently received `GLOBAL_POSITION_INT` / `ATTITUDE` message at
capture time. That's accurate enough for building a test dataset.

### Usage

```bash
# raw mode, unlimited frames until Ctrl+C
python3 capture_flight_frames.py --mav-device /dev/ttyACM0 --format raw

# jpg mode, exactly 300 frames
python3 capture_flight_frames.py --mav-device /dev/ttyACM0 --format jpg \
    --frames 300 --isp-repo ~/isp_npp/Antek_isp/iza/ar0234_isp
```

### Key arguments

- `--cam-device` — V4L2 device path (default `/dev/video0`)
- `--exposure`, `--gain` — passed straight to `v4l2-ctl -c exposure=... -c analogue_gain=...`
  before recording starts; verify the actual control names on your hardware
  with `v4l2-ctl -d /dev/video0 --list-ctrls`, they're driver-dependent.
- `--warmup-frames` — frames captured and discarded right after setting
  exposure/gain, so the sensor has time to actually apply the new settings
  before real recording starts (default 3).
- `--mav-device`, `--mav-baud` — MAVLink connection to the flight controller.
- `--output-dir` — parent folder for flight folders (default `~/Documents/flights/gps`).
- `--isp-repo` — path to the `ar0234_isp` checkout (only used in `jpg` mode).
- `--log-every` — how often to print FPS/GPS status to the console.

CSV columns: `Filename, Index, Lat, Lon, Alt, Roll, Pitch, Yaw` — the same
schema as `photos_position.csv` in Aligatorpy's `CameraService.py`, so
`ImageMosaicService` can be reused later once the frames are debayered
(swap the `.raw` filenames for the debayered `.jpg`/`.png` names in the CSV).

### Known limitation

If `master_dark.bin` / `gain_map.bin` don't exist yet, `process()` from
`ar0234_isp` currently raises instead of falling back to an uncalibrated
debayer — so `--format jpg` requires calibration data to be present. If you
want to fly `jpg` mode without calibration, that's a small change to add
(a `calibrate=False` fallback) — just ask.

## `capture_frames_no_gps.py`

A stripped-down variant with **no MAVLink dependency and no GPS/attitude
tagging** — just fires off raw frames as fast as `v4l2-ctl` allows. Useful
for bench testing, indoor runs, or any time the flight controller isn't
connected and you just need raw camera data quickly. Always saves `.raw`
only — no `jpg` mode.

### Requirements

- `numpy`
- `v4l2-ctl` available on `PATH`

### Usage

```bash
# Unlimited frames, until Ctrl+C
python3 capture_frames_no_gps.py

# Exactly 200 frames
python3 capture_frames_no_gps.py --frames 200
```

### Key arguments

Same camera-related arguments as `capture_flight_frames.py`
(`--cam-device`, `--exposure`, `--gain`, `--warmup-frames`, `--frames`,
`--log-every`), plus `--output-dir` (default `~/Documents/flights/no_gps`).

CSV columns: `Filename, Index, Timestamp` — no GPS/attitude columns, since
there's no flight controller connection to get them from.

## `debayer_flight_frames.py`

Post-processes a flight folder produced by either capture script — debayers
(and optionally calibrates/denoises) every `.raw` frame into a `.jpg`, using
the existing pipeline from `ar0234_isp/process/process_raw.py` (no logic is
duplicated here, it's imported directly).

Works on both folder types:

```
~/Documents/flights/gps/{date_time}/      (has photos_position.csv)
~/Documents/flights/no_gps/{date_time}/   (has frames.csv)
```

Output goes into a `debayered/` subfolder inside the flight folder:

```
{date_time}/
├── IMG_0000_....raw
├── ...
├── photos_position.csv          <- untouched original
└── debayered/
    ├── IMG_0000_....jpg
    ├── ...
    └── photos_position.csv      <- copy with Filename updated to the .jpg names
```

That way the debayered CSV can be pointed straight at `ImageMosaicService`
(or any other tool expecting real image files) without further editing. If
the flight folder has no CSV, frames are still processed — just no CSV is
written.

Re-running is safe: frames whose `.jpg` already exists in `debayered/` are
skipped (pass `--overwrite` to redo everything), and a single corrupt or
unreadable frame is reported and skipped rather than stopping the batch.

### Requirements

- `opencv-python`, `numpy`
- A local checkout of the `ar0234_isp` repo

### Usage

```bash
# GPS-tagged flight, with calibration (needs master_dark.bin / gain_map.bin
# to exist under ar0234_isp/calib)
python3 debayer_flight_frames.py ~/Documents/flights/gps/2026-08-06_14-30-05

# no_gps flight, no calibration data available
python3 debayer_flight_frames.py ~/Documents/flights/no_gps/2026-08-06_15-02-11 \
    --no-calibrate

# Redo everything with a different bayer pattern / ISP repo location
python3 debayer_flight_frames.py ~/Documents/flights/gps/2026-08-06_14-30-05 \
    --overwrite --bayer-code GR --isp-repo ~/isp_npp/Antek_isp/iza/ar0234_isp
```

### Key arguments

- `flight_dir` — the specific flight folder to process (positional, required).
- `--isp-repo` — path to the `ar0234_isp` checkout.
- `--bayer-code` — `GR`/`BG`/`RG`/`GB`, passed to `process()` (default `GB`,
  matching the default in `ar0234_isp`'s own `process_raw.py`).
- `--calibrate` / `--no-calibrate` — apply dark/gain calibration if available
  (default: on). `process()` currently raises if calibration is requested
  but `master_dark.bin`/`gain_map.bin` aren't found — use `--no-calibrate`
  for frames captured without calibration data.
- `--target-mean`, `--sat-boost` — passed through to the AWB/gamma step.
- `--denoise {bilateral,nlm}`, `--denoise-strength` — optional denoise pass.
- `--output-subdir` — name of the output folder inside `flight_dir` (default
  `debayered`).
- `--overwrite` — reprocess frames whose `.jpg` already exists.
