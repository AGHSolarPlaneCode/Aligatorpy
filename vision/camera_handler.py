"""
CameraPipeline — strumień OV9281 (mono, global shutter) przez GStreamer.

Trzy gałęzie z tee:
  - streaming (shmsink)        : podgląd na laptopie, działa równolegle, niezależnie
  - search   (valve, 30 fps)   : faza SEARCH — get_image() w trybie search
  - decode   (valve, 120 fps)  : faza OOK   — get_image() w trybie decode

Znacznik czasu klatki: (base_time + buf.pts)/1e9 — zegar monotoniczny GStreamera.
Mózg dodaje do niego CLOCK_OFFSET przy interpolacji telemetrii.

get_image() zwraca (płaski 1D uint8, ts_sec, err). Konsument robi reshape do (H, W).
"""
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import threading
import signal
import numpy as np
from typing import Tuple

# ── Konfiguracja ──
WIDTH, HEIGHT = 1280, 800
FPS_SOURCE = 120
FORMAT = "GRAY8"          # OV9281 monochromatyczna
FPS_STREAM = 60           # gałąź podglądu (po cropie)
FPS_SEARCH = 30           # gałąź SEARCH
FPS_DECODE = 120          # gałąź OOK
SHM_SOCKET_PATH = "/tmp/cam_stream"
SHM_SIZE = 12000000


class CameraPipeline:
    def __init__(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()

        self._lock_search = threading.Lock()
        self._frame_search = None
        self._ts_search = None

        self._lock_decode = threading.Lock()
        self._frame_decode = None
        self._ts_decode = None

        self._lock_valve = threading.Lock()
        self._base_time_ns = 0

        self.pipeline = self._build_pipeline()
        self._connect_bus()
        self.valve_search = self.pipeline.get_by_name("valve_search")
        self.valve_decode = self.pipeline.get_by_name("valve_decode")

    def _build_pipeline(self) -> Gst.Pipeline:
        pipeline_str = f"""
            libcamerasrc
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_SOURCE}/1
                ! tee name=source_tee

            source_tee.
                ! queue name=stream_q max-size-buffers=2 leaky=downstream
                ! videocrop top=40 bottom=40
                ! videorate
                ! video/x-raw, framerate={FPS_STREAM}/1
                ! shmsink socket-path={SHM_SOCKET_PATH} shm-size={SHM_SIZE}
                    sync=false wait-for-connection=false

            source_tee.
                ! queue name=local_q max-size-buffers=2 leaky=downstream
                ! tee name=local_tee

            local_tee.
                ! queue name=search_q max-size-buffers=2 leaky=downstream
                ! valve name=valve_search drop=true
                ! videorate drop-only=true
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_SEARCH}/1
                ! appsink name=sink_search emit-signals=true max-buffers=1 drop=true sync=false

            local_tee.
                ! queue name=decode_q max-size-buffers=2 leaky=downstream
                ! valve name=valve_decode drop=true
                ! video/x-raw, format={FORMAT}, width={WIDTH}, height={HEIGHT},
                  framerate={FPS_DECODE}/1
                ! appsink name=sink_decode emit-signals=true max-buffers=1 drop=true sync=false
        """
        pipeline = Gst.parse_launch(pipeline_str)
        pipeline.get_by_name("sink_search").connect("new-sample", self._on_search)
        pipeline.get_by_name("sink_decode").connect("new-sample", self._on_decode)
        return pipeline

    def _pull(self, sink) -> Tuple:
        sample = sink.emit("pull-sample")
        if sample is None:
            return None, None
        buf = sample.get_buffer()
        if buf.pts == Gst.CLOCK_TIME_NONE:
            return None, None
        ts_sec = (self._base_time_ns + buf.pts) / 1e9
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None, None
        try:
            data = np.frombuffer(mi.data, dtype=np.uint8).copy()
        finally:
            buf.unmap(mi)
        return data, ts_sec

    def _on_search(self, sink):
        d, ts = self._pull(sink)
        if d is not None:
            with self._lock_search:
                self._frame_search, self._ts_search = d, ts
        return Gst.FlowReturn.OK

    def _on_decode(self, sink):
        d, ts = self._pull(sink)
        if d is not None:
            with self._lock_decode:
                self._frame_decode, self._ts_decode = d, ts
        return Gst.FlowReturn.OK

    def get_image(self) -> Tuple:
        """Zwraca (płaski 1D uint8, ts, err) z aktywnej gałęzi (search albo decode)."""
        with self._lock_valve:
            search_on = not self.valve_search.get_property("drop")
            decode_on = not self.valve_decode.get_property("drop")
        if decode_on:
            with self._lock_decode:
                if self._frame_decode is None:
                    return None, None, "brak klatki (decode)"
                return self._frame_decode, self._ts_decode, None
        if search_on:
            with self._lock_search:
                if self._frame_search is None:
                    return None, None, "brak klatki (search)"
                return self._frame_search, self._ts_search, None
        return None, None, "brak aktywnej gałęzi"

    def set_search_active(self, active: bool):
        with self._lock_valve:
            if active:
                self.valve_decode.set_property("drop", True)
                self.valve_search.set_property("drop", False)
            else:
                self.valve_search.set_property("drop", True)

    def set_decode_active(self, active: bool):
        with self._lock_valve:
            if active:
                self.valve_search.set_property("drop", True)
                self.valve_decode.set_property("drop", False)
            else:
                self.valve_decode.set_property("drop", True)

    def _connect_bus(self):
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::eos", self._on_eos)
        bus.connect("message::state-changed", self._on_state)

    def _on_state(self, bus, msg):
        if msg.src == self.pipeline:
            _, new, _ = msg.parse_state_changed()
            if new == Gst.State.PLAYING:
                self._base_time_ns = self.pipeline.get_base_time()

    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[cam ERROR] {err.message} | {dbg}")
        self.stop()

    def _on_eos(self, bus, msg):
        print("[cam EOS]")
        self.stop()

    def start(self):
        """Blokujące — uruchom w osobnym wątku. get_image() woła się z innego wątku."""
        self.pipeline.set_state(Gst.State.PLAYING)
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        self.loop.run()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
