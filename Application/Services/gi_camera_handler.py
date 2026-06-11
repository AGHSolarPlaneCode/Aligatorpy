import gi
import os   # for socket cleanup
import time

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import signal
import numpy as np

# ── Configuration ────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 640, 400
FPS_SOURCE = 240
FORMAT = "GRAY16_LE"
FPS_STREAM = 30
FPS_LOW = 10


class CameraPipeline:
    WIDTH = WIDTH
    HEIGHT = HEIGHT

    def __init__(self):
        Gst.init(None)

        if not Gst.ElementFactory.find("shmsink"):
            raise RuntimeError("shmsink plugin not found. Install gstreamer1.0-plugins-bad or -ugly")

        self.loop = GLib.MainLoop()
        self.shm_socket_path = "/tmp/camera_stream"

        # ── Frame storage (główny wątek — bez locków) ───────────────────────────
        self._frame_10fps = None
        self._ts_10fps = None
        self._frame_120fps = None
        self._ts_120fps = None
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
                ! queue name=shm_q leaky=downstream max-size-buffers=1
                ! videorate drop-only=true
                ! video/x-raw,
                    format={FORMAT},
                    width={WIDTH},
                    height={HEIGHT},
                    framerate={FPS_STREAM}/1
                ! shmsink
                    socket-path={self.shm_socket_path}
                    shm-size=2000000
                    wait-for-connection=false
                    sync=false

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

        self._frame_10fps = frame
        self._ts_10fps = ts

        return Gst.FlowReturn.OK

    def _on_frame_120fps(self, sink):
        frame, ts = self._pull_frame(sink)
        if frame is None:
            return Gst.FlowReturn.OK

        self._frame_120fps = frame
        self._ts_120fps = ts

        return Gst.FlowReturn.OK

    def _pump(self) -> None:
        GLib.MainContext.default().iteration(False)

    def get_image(self):
        self._pump()
        v10 = not self.valve_10fps.get_property("drop")
        v120 = not self.valve_120fps.get_property("drop")

        if v10:
            return self._frame_10fps, self._ts_10fps, None

        if v120:
            return self._frame_120fps, self._ts_120fps, None

        return None, None, "No active branch"

    # ── Control ───────────────────────────────────────────────────────────────
    def set_10fps_active(self, active: bool):
        if active:
            self.valve_120fps.set_property("drop", True)
            self.valve_10fps.set_property("drop", False)
        else:
            self.valve_10fps.set_property("drop", True)

    def set_120fps_active(self, active: bool):
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
    def start(self) -> None:
        """Uruchamia pipeline w głównym wątku — GLib obsługiwany w get_image()."""
        self.pipeline.set_state(Gst.State.PLAYING)
        signal.signal(signal.SIGINT, lambda *_: self.stop())

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame, _, _ = self.get_image()
            if frame is not None:
                return True
            time.sleep(0.1)
        return False

    def run(self) -> None:
        """Blokująca pętla GLib — tylko dla uruchomienia standalone (__main__)."""
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        self.loop.run()

    def stop(self):
        print("[pipeline] stopping")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

        try:
            os.remove(self.shm_socket_path)
            print(f"[pipeline] removed socket {self.shm_socket_path}")
        except FileNotFoundError:
            pass   # already gone, no problem
        except Exception as e:
            print(f"[pipeline] warning: could not remove socket: {e}")


# ── Demo ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    cam = CameraPipeline()
    cam.start()
    mode_10 = True
    last_switch = time.monotonic()

    try:
        while True:
            if time.monotonic() - last_switch >= 2:
                mode_10 = not mode_10
                if mode_10:
                    cam.set_10fps_active(True)
                    label = "10fps"
                else:
                    cam.set_120fps_active(True)
                    label = "120fps"
                f, t, _ = cam.get_image()
                print(label + ":", None if f is None else f.shape, t)
                last_switch = time.monotonic()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
