import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import threading
import signal
import numpy as np
from typing import Tuple

# ── Configuration ────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 640, 400
FPS_SOURCE = 200
FORMAT = "GRAY16_LE"
FPS_STREAM = 30
FPS_LOW = 10


class CameraPipeline:
    def __init__(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()

        # ── Frame storage ──────────────────────────────────────────────────────
        self._lock_10fps = threading.Lock()
        self._frame_10fps = None
        self._ts_10fps = None

        self._lock_120fps = threading.Lock()
        self._frame_120fps = None
        self._ts_120fps = None

        self._lock_valve = threading.Lock()
        self._base_time_ns = 0

        self.pipeline = self._build_pipeline()
        self._connect_bus()

        self.valve_10fps = self.pipeline.get_by_name("valve_10fps")
        self.valve_120fps = self.pipeline.get_by_name("valve_120fps")

    # ── Pipeline ──────────────────────────────────────────────────────────────
    def _build_pipeline(self):
        pipeline_str = f"""
            libcamerasrc
                ! video/x-raw,
                    format={FORMAT},
                    width={WIDTH},
                    height={HEIGHT},
                    framerate={FPS_SOURCE}/1
                ! tee name=source_tee

            source_tee.
                ! queue leaky=downstream max-size-buffers=1
                ! fakesink sync=false

            source_tee.
                ! queue name=local_q leaky=downstream max-size-buffers=1
                ! tee name=local_tee

            local_tee.
                ! queue name=low_q leaky=downstream max-size-buffers=1
                ! valve name=valve_10fps drop=false
                ! videorate drop-only=true
                ! video/x-raw,
                    format={FORMAT},
                    width={WIDTH},
                    height={HEIGHT},
                    framerate={FPS_LOW}/1
                ! appsink name=sink_10fps
                    emit-signals=true
                    max-buffers=1
                    drop=true
                    sync=false
                    async=false
                    enable-last-sample=false

            local_tee.
                ! queue name=high_q leaky=downstream max-size-buffers=1
                ! valve name=valve_120fps drop=true
                ! video/x-raw,
                    format={FORMAT},
                    width={WIDTH},
                    height={HEIGHT},
                    framerate={FPS_SOURCE}/1
                ! appsink name=sink_120fps
                    emit-signals=true
                    max-buffers=1
                    drop=true
                    sync=false
                    async=false
                    enable-last-sample=false
        """

        pipeline = Gst.parse_launch(pipeline_str)

        pipeline.get_by_name("sink_10fps").connect(
            "new-sample", self._on_frame_10fps
        )
        pipeline.get_by_name("sink_120fps").connect(
            "new-sample", self._on_frame_120fps
        )

        return pipeline

    # ── Timing ───────────────────────────────────────────────────────────────
    def _get_running_time_ns(self):
        clock = self.pipeline.get_clock()
        if clock is None:
            return 0
        return clock.get_time() - self._base_time_ns

    # ── ZERO-COPY FRAME EXTRACTION ────────────────────────────────────────────
    def _pull_frame(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return None, None

        buf = sample.get_buffer()

        # timestamp
        if buf.pts != Gst.CLOCK_TIME_NONE:
            ts = (self._base_time_ns + buf.pts) / 1e9
        else:
            ts = self._get_running_time_ns() / 1e9

        caps = sample.get_caps()
        struct = caps.get_structure(0)

        width = struct.get_int("width")[1]
        height = struct.get_int("height")[1]

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None, None

        try:
            frame = np.ndarray(
                shape=(height, width),
                dtype=np.uint16,
                buffer=mapinfo.data
            )

            return frame, ts

        finally:
            buf.unmap(mapinfo)

    def _on_frame_10fps(self, sink):
        frame, ts = self._pull_frame(sink)
        if frame is None:
            return Gst.FlowReturn.OK

        with self._lock_10fps:
            self._frame_10fps = frame
            self._ts_10fps = ts

        return Gst.FlowReturn.OK

    def _on_frame_120fps(self, sink):
        frame, ts = self._pull_frame(sink)
        if frame is None:
            return Gst.FlowReturn.OK

        with self._lock_120fps:
            self._frame_120fps = frame
            self._ts_120fps = ts

        return Gst.FlowReturn.OK

    # ── Public API ───────────────────────────────────────────────────────────
    def get_image(self):
        with self._lock_valve:
            v10 = not self.valve_10fps.get_property("drop")
            v120 = not self.valve_120fps.get_property("drop")

        if v10:
            with self._lock_10fps:
                return self._frame_10fps, self._ts_10fps, None

        if v120:
            with self._lock_120fps:
                return self._frame_120fps, self._ts_120fps, None

        return None, None, "No active branch"

    # ── Control ───────────────────────────────────────────────────────────────
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

    # ── Bus ───────────────────────────────────────────────────────────────────
    def _connect_bus(self):
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::eos", self._on_eos)
        bus.connect("message::warning", self._on_warning)
        bus.connect("message::state-changed", self._on_state_changed)

    def _on_state_changed(self, bus, msg):
        if msg.src == self.pipeline:
            old, new, _ = msg.parse_state_changed()
            if new == Gst.State.PLAYING:
                self._base_time_ns = self.pipeline.get_base_time()

    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print("[ERROR]", err.message)
        if dbg:
            print("[DEBUG]", dbg)
        self.stop()

    def _on_eos(self, bus, msg):
        print("[EOS]")
        self.stop()

    def _on_warning(self, bus, msg):
        warn, _ = msg.parse_warning()
        print("[WARN]", warn.message)

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        self.loop.run()

    def stop(self):
        print("[pipeline] stopping")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()


# ── Demo ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import time

    cam = CameraPipeline()

    def demo():
        while True:
            cam.set_10fps_active(True)
            time.sleep(2)
            f, t, _ = cam.get_image()
            print("10fps:", None if f is None else f.shape, t)

            cam.set_120fps_active(True)
            time.sleep(2)
            f, t, _ = cam.get_image()
            print("60fps:", None if f is None else f.shape, t)

    threading.Thread(target=demo, daemon=True).start()
    cam.start()
