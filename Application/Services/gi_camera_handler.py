import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import threading
import signal
import numpy as np
from typing import Tuple

# ── Configuration ────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 1280, 800
FPS_SOURCE = 60
FORMAT = "GRAY8"  # OV9281 is monochrome
FPS_STREAM = 60  # streaming branch framerate (after crop)
FPS_LOW = 10  # low‑rate processing branch
SHM_SOCKET_PATH = "/tmp/cam_stream"
SHM_SIZE = 12000000


# ─────────────────────────────────────────────────────────────────────────────


class CameraPipeline:
    def __init__(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()

        # Thread‑safe storage for the latest frame & timestamp of each branch
        self._lock_10fps = threading.Lock()
        self._frame_10fps: np.ndarray | None = None
        self._ts_10fps: float | None = None

        self._lock_120fps = threading.Lock()
        self._frame_120fps: np.ndarray | None = None
        self._ts_120fps: float | None = None

        self._lock_valve = threading.Lock()
        self._base_time_ns: int = 0

        self.pipeline = self._build_pipeline()
        self._connect_bus()
        self.valve_10fps = self.pipeline.get_by_name("valve_10fps")
        self.valve_120fps = self.pipeline.get_by_name("valve_120fps")

    # ── Pipeline construction ─────────────────────────────────────────────────
    def _build_pipeline(self) -> Gst.Pipeline:
        pipeline_str = f"""
            libcamerasrc
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_SOURCE}/1
                ! tee name=source_tee

            source_tee.
                ! queue name=stream_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! videocrop top=40 bottom=40
                ! videorate
                ! video/x-raw, framerate={FPS_STREAM}/1
                ! shmsink socket-path={SHM_SOCKET_PATH}
                    shm-size={SHM_SIZE}
                    sync=false
                    wait-for-connection=false

            source_tee.
                ! queue name=local_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! tee name=local_tee

            local_tee.
                ! queue name=low_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! valve name=valve_10fps drop=false
                ! videorate drop-only=true
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_LOW}/1
                ! appsink name=sink_10fps
                    emit-signals=true max-buffers=1 drop=true sync=false

            local_tee.
                ! queue name=high_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! valve name=valve_120fps drop=true
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_SOURCE}/1
                ! appsink name=sink_120fps
                    emit-signals=true max-buffers=1 drop=true sync=false
        """
        pipeline = Gst.parse_launch(pipeline_str)

        sink_10 = pipeline.get_by_name("sink_10fps")
        sink_10.connect("new-sample", self._on_frame_10fps)
        sink_120 = pipeline.get_by_name("sink_120fps")
        sink_120.connect("new-sample", self._on_frame_120fps)

        return pipeline

    # ── Appsink callbacks ─────────────────────────────────────────────────────
    def _pull_frame(self, sink) -> Tuple[np.ndarray | None, float | None]:
        """
        Pull one sample from an appsink and return (numpy_array, capture_ts_sec).
        Returns (None, None) on EOS / flush.
        """
        sample = sink.emit("pull-sample")
        if sample is None:
            return None, None

        buf = sample.get_buffer()
        if buf.pts == Gst.CLOCK_TIME_NONE:
            return None, None

        # Read safely without per-frame setup or race conditions
        ts_sec = (self._base_time_ns + buf.pts) / 1e9

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None, None
        try:
            # Copy buffer data into a NumPy array before memory unmapping
            data = np.frombuffer(mapinfo.data, dtype=np.uint8).copy()
        finally:
            buf.unmap(mapinfo)

        return data, ts_sec

    def _on_frame_10fps(self, sink) -> Gst.FlowReturn:
        data, ts = self._pull_frame(sink)
        if data is None:
            return Gst.FlowReturn.OK
        with self._lock_10fps:
            self._frame_10fps = data
            self._ts_10fps = ts
        return Gst.FlowReturn.OK

    def _on_frame_120fps(self, sink) -> Gst.FlowReturn:
        data, ts = self._pull_frame(sink)
        if data is None:
            return Gst.FlowReturn.OK
        with self._lock_120fps:
            self._frame_120fps = data
            self._ts_120fps = ts
        return Gst.FlowReturn.OK

    # ── Public: retrieve latest image from the active branch ──────────────────
    def get_image(self) -> Tuple[np.ndarray | None, float | None, str | None]:
        with self._lock_valve:
            valve_10_open = not self.valve_10fps.get_property("drop")
            valve_120_open = not self.valve_120fps.get_property("drop")

        if valve_10_open:
            with self._lock_10fps:
                if self._frame_10fps is None:
                    return None, None, "No frame received yet (10 fps)"
                return self._frame_10fps, self._ts_10fps, None

        elif valve_120_open:
            with self._lock_120fps:
                if self._frame_120fps is None:
                    return None, None, "No frame received yet (120 fps)"
                return self._frame_120fps, self._ts_120fps, None

        else:
            return None, None, "No active local branch"

    # ── Valve control ─────────────────────────────────────────────────────────
    def set_10fps_active(self, active: bool):
        with self._lock_valve:
            if active:
                self.valve_120fps.set_property("drop", True)
                self.valve_10fps.set_property("drop", False)
            else:
                self.valve_10fps.set_property("drop", True)

    def set_120fps_active(self, active: bool):
        with self._lock_valve:
            if active:
                self.valve_10fps.set_property("drop", True)
                self.valve_120fps.set_property("drop", False)
            else:
                self.valve_120fps.set_property("drop", True)

    # ── Bus handlers ──────────────────────────────────────────────────────────
    def _connect_bus(self):
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::eos", self._on_eos)
        bus.connect("message::warning", self._on_warning)
        bus.connect("message::state-changed", self._on_state_changed)

    def _on_state_changed(self, bus, msg):
        # Only check messages coming from the top-level pipeline
        if msg.src == self.pipeline:
            old, new, pending = msg.parse_state_changed()
            if new == Gst.State.PLAYING:
                # Capture the base_time safely right as the pipeline transitions on the main thread
                self._base_time_ns = self.pipeline.get_base_time()

    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[ERROR] {err.message}")
        if dbg:
            print(f"[DEBUG] {dbg}")
        self.stop()

    def _on_eos(self, bus, msg):
        print("[EOS] stream ended")
        self.stop()

    def _on_warning(self, bus, msg):
        warn, dbg = msg.parse_warning()
        print(f"[WARN] {warn.message}")

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        self.loop.run()

    def stop(self):
        print("[pipeline] stopping …")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()


# ── Demo / test ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    cam = CameraPipeline()


    def demo_toggle():
        import time
        time.sleep(3)
        frame, ts, err = cam.get_image()
        size = frame.size if frame is not None else 0
        ts_val = ts if ts is not None else 0.0
        print(f"[demo] 10fps  frame: size={size}, ts={ts_val:.6f}s, err={err}")

        cam.set_120fps_active(True)
        time.sleep(3)
        frame, ts, err = cam.get_image()
        size = frame.size if frame is not None else 0
        ts_val = ts if ts is not None else 0.0
        print(f"[demo] 120fps frame: size={size}, ts={ts_val:.6f}s, err={err}")

        cam.stop()


    threading.Thread(target=demo_toggle, daemon=True).start()
    cam.start()