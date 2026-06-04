import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import signal
import threading

# ── Configuration ────────────────────────────────────────────────────────────
WIDTH, HEIGHT   = 1280, 800
FPS_SOURCE      = 120
FORMAT          = "GRAY8"           # OV9281 is monochrome
FPS_MAIN        = 15                # streaming branch framerate
FPS_LOW         = 10                # low‑rate processing branch
UDP_HOST        = "0.0.0.0"
UDP_PORT        = 5000
# ─────────────────────────────────────────────────────────────────────────────


class CameraPipeline:
    def __init__(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()
        self.pipeline = self._build_pipeline()
        self._connect_bus()
        # Valve references for runtime control
        self.valve_10fps  = self.pipeline.get_by_name("valve_10fps")
        self.valve_120fps = self.pipeline.get_by_name("valve_120fps")

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
                ! videoconvert
                ! videorate
                ! video/x-raw, framerate={FPS_MAIN}/1
                ! avenc_h264
                ! h264parse
                ! rtph264pay config-interval=1 pt=96
                ! udpsink host={UDP_HOST} port={UDP_PORT} sync=false

            source_tee.
                ! queue name=local_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! tee name=local_tee

            local_tee.
                ! queue name=low_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! videorate
                ! video/x-raw, framerate={FPS_LOW}/1
                ! valve name=valve_10fps drop=false
                ! appsink name=sink_10fps
                    emit-signals=true max-buffers=1 drop=true sync=false

            local_tee.
                ! queue name=high_q
                    max-size-buffers=2 max-size-bytes=0 max-size-time=0
                    leaky=downstream
                ! valve name=valve_120fps drop=true
                ! appsink name=sink_120fps
                    emit-signals=true max-buffers=1 drop=true sync=false
        """
        pipeline = Gst.parse_launch(pipeline_str)

        # Connect callbacks for both sinks
        sink_10 = pipeline.get_by_name("sink_10fps")
        sink_10.connect("new-sample", self._on_frame_10fps)
        sink_120 = pipeline.get_by_name("sink_120fps")
        sink_120.connect("new-sample", self._on_frame_120fps)

        return pipeline

    # ── Appsink callbacks ────────────────────────────────────────────────────
    def _on_frame_10fps(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        pts_ns = buf.pts
        if pts_ns != Gst.CLOCK_TIME_NONE:
            pts_sec = pts_ns / 1e9
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            w = struct.get_int("width").value
            h = struct.get_int("height").value
            print(f"[10fps] {w}x{h}  pts={pts_sec:.6f}s (monotonic)")
        else:
            print("[10fps] No PTS on buffer")

        return Gst.FlowReturn.OK

    def _on_frame_120fps(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        pts_ns = buf.pts
        if pts_ns != Gst.CLOCK_TIME_NONE:
            pts_sec = pts_ns / 1e9
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            w = struct.get_int("width").value
            h = struct.get_int("height").value
            print(f"[120fps] {w}x{h}  pts={pts_sec:.6f}s (monotonic)")
        else:
            print("[120fps] No PTS on buffer")

        return Gst.FlowReturn.OK

    # ── Bus handlers ─────────────────────────────────────────────────────────
    def _connect_bus(self):
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error",   self._on_error)
        bus.connect("message::eos",     self._on_eos)
        bus.connect("message::warning", self._on_warning)

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
        warn, _ = msg.parse_warning()
        print(f"[WARN] {warn.message}")

    # ── Public controls ──────────────────────────────────────────────────────
    def set_10fps_active(self, active: bool):
        self.valve_10fps.set_property("drop", not active)
        state = "ACTIVE" if active else "DROPPED"
        print(f"[valve] 10fps branch → {state}")

    def set_120fps_active(self, active: bool):
        self.valve_120fps.set_property("drop", not active)
        state = "ACTIVE" if active else "DROPPED"
        print(f"[valve] 120fps branch → {state}")

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        print("[pipeline] PLAYING")
        print(f"  camera : libcamerasrc → {WIDTH}x{HEIGHT} @ {FPS_SOURCE}fps")
        print(f"  streaming : {FPS_MAIN}fps → udp://{UDP_HOST}:{UDP_PORT}")
        print(f"  local 10fps : ACTIVE")
        print(f"  local 120fps: DROPPED")
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        self.loop.run()

    def stop(self):
        print("[pipeline] stopping …")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()


# ── Demo / test ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    cam = CameraPipeline()

    # Toggle branches after a few seconds to verify both timestamps
    def demo_toggle():
        import time
        time.sleep(5)
        cam.set_120fps_active(True)    # enable high‑rate branch
        time.sleep(5)
        cam.set_10fps_active(False)    # disable low‑rate branch
        time.sleep(5)
        cam.set_120fps_active(False)   # disable all local processing
        time.sleep(2)
        cam.stop()

    threading.Thread(target=demo_toggle, daemon=True).start()
    cam.start()
