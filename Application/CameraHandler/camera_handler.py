import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import threading
import signal
from typing import Tuple

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

        # Thread‑safe storage for the latest frame & timestamp of each branch
        self._lock_10fps = threading.Lock()
        self._frame_10fps = None   # raw GRAY8 bytes
        self._ts_10fps = None      # monotonic capture time (seconds)

        self._lock_120fps = threading.Lock()
        self._frame_120fps = None
        self._ts_120fps = None

        self.pipeline = self._build_pipeline()
        self._connect_bus()
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

    # ── Appsink callbacks (store frame + timestamp, no printing) ─────────────
    def _on_frame_10fps(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        pts_ns = buf.pts
        if pts_ns == Gst.CLOCK_TIME_NONE:
            return Gst.FlowReturn.OK

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if ok:
            data = bytes(mapinfo.data)          # copy raw GRAY8 bytes
            pts_sec = pts_ns / 1e9
            with self._lock_10fps:
                self._frame_10fps = data
                self._ts_10fps = pts_sec
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _on_frame_120fps(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        pts_ns = buf.pts
        if pts_ns == Gst.CLOCK_TIME_NONE:
            return Gst.FlowReturn.OK

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if ok:
            data = bytes(mapinfo.data)
            pts_sec = pts_ns / 1e9
            with self._lock_120fps:
                self._frame_120fps = data
                self._ts_120fps = pts_sec
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    # ── Public: retrieve latest image from the active branch ─────────────────
    def get_image(self) -> Tuple[bytes | None, float | None, str | None]:
        """
        Returns the latest frame from the currently active local branch.

        Returns:
            (frame, timestamp, error)
            - frame (bytes or None): raw GRAY8 image data (1280*800 bytes)
            - timestamp (float or None): monotonic capture time in seconds
            - error (str or None): error description if frame unavailable
        """
        # Determine which branch is active (drop == False)
        if not self.valve_10fps.get_property("drop"):
            # 10 fps branch is active
            with self._lock_10fps:
                if self._frame_10fps is None:
                    return None, None, "No frame received yet (10fps)"
                return self._frame_10fps, self._ts_10fps, None

        elif not self.valve_120fps.get_property("drop"):
            # 120 fps branch is active
            with self._lock_120fps:
                if self._frame_120fps is None:
                    return None, None, "No frame received yet (120fps)"
                return self._frame_120fps, self._ts_120fps, None

        else:
            return None, None, "No active local branch"

    # ── Valve control with mutual exclusivity ────────────────────────────────
    def set_10fps_active(self, active: bool):
        if active:
            # Ensure only 10fps is open
            self.valve_120fps.set_property("drop", True)
            self.valve_10fps.set_property("drop", False)
            print("[valve] 10fps branch → ACTIVE   (120fps closed)")
        else:
            self.valve_10fps.set_property("drop", True)
            print("[valve] 10fps branch → DROPPED")

    def set_120fps_active(self, active: bool):
        if active:
            # Ensure only 120fps is open
            self.valve_10fps.set_property("drop", True)
            self.valve_120fps.set_property("drop", False)
            print("[valve] 120fps branch → ACTIVE   (10fps closed)")
        else:
            self.valve_120fps.set_property("drop", True)
            print("[valve] 120fps branch → DROPPED")

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

    def demo_toggle():
        import time
        # Wait for a few frames to arrive
        time.sleep(3)
        frame, ts, err = cam.get_image()
        print(f"[demo] 10fps frame: size={len(frame) if frame else 0}, ts={ts}, err={err}")

        # Switch to high‑rate branch
        cam.set_120fps_active(True)
        time.sleep(3)
        frame, ts, err = cam.get_image()
        print(f"[demo] 120fps frame: size={len(frame) if frame else 0}, ts={ts}, err={err}")

        # Stop all local processing
        cam.set_120fps_active(False)
        time.sleep(1)
        frame, ts, err = cam.get_image()
        print(f"[demo] no branch: err={err}")

        cam.stop()

    threading.Thread(target=demo_toggle, daemon=True).start()
    cam.start()
