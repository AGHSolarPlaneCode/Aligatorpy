"""
MatekService — warstwa FC (MAVLink) z JEDNYM wątkiem RX (demux).

Powód refaktora: jeden UART do FC + wątek telemetrii w tle. Gdyby kilka wątków
wołało recv_match na tym samym master, kradłyby sobie wiadomości. Tu JEDYNYM
konsumentem master.recv_match jest wątek _rx_loop; rozdziela on wiadomości po
typie do struktur, z których czytają pozostałe metody.

Stempel czasu telemetrii: time.monotonic() — ta sama domena co znaczniki klatek
GStreamera (po dodaniu CLOCK_OFFSET, które robi Mózg).
"""
from __future__ import annotations

import threading
import queue
import time
import math
import bisect
from collections import deque, namedtuple
from typing import Any, List, Dict, Optional, Tuple

from pymavlink import mavutil

from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger


TelemetrySample = namedtuple("TelemetrySample", "ts lat lon alt roll pitch yaw")


def _wrap_pi(a: float) -> float:
    """Zawija kąt do (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class MatekService:
    BOTTLE_SERVO_CHANNEL = 9
    BEACON_SERVO_CHANNEL = 8
    PWM_DROP_SERVO = 800

    def __init__(self, device: str = cfg.mav.device, baud: int = cfg.mav.baud,    #  bez configa na sztywno
                 tele_buffer: int = 150):
        self.logger = get_logger(__name__)
        self.device = device
        self.baud = baud

        self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
        self.master.wait_heartbeat()
        self.logger.info("Connected to system %s component %s",
                         self.master.target_system, self.master.target_component)
        self._mode_map = self.master.mode_mapping()

        # ── struktury wypełniane przez wątek RX ──
        self._lock = threading.Lock()
        self._tele_buf: deque = deque(maxlen=tele_buffer)   # TelemetrySample, rosnące ts
        self._latest_gps: Optional[Tuple[float, float, float]] = None
        self._acks: Dict[int, int] = {}                     # command_id -> result
        self._reached: set = set()                          # skumulowane MISSION_ITEM_REACHED
        self._mission_current: Optional[int] = None
        self._hb: Optional[Tuple[int, int]] = None          # (base_mode, custom_mode)
        self._proto_q: queue.Queue = queue.Queue()          # wiadomości protokołu misji
        self._upload_lock = threading.Lock()

        # ── start wątku RX ──
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ================================================================== #
    #  WĄTEK RX — jedyny konsument master.recv_match
    # ================================================================== #
    def _rx_loop(self) -> None:
        ap_comp = 1   # autopilot component id
        while self._rx_running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                self.logger.warning("RX recv error: %s", e)
                continue
            if msg is None:
                continue

            t = msg.get_type()

            if t == "ATTITUDE":
                with self._lock:
                    gps = self._latest_gps
                    if gps is not None:
                        self._tele_buf.append(TelemetrySample(
                            time.monotonic(), gps[0], gps[1], gps[2],
                            msg.roll, msg.pitch, msg.yaw))

            elif t == "GLOBAL_POSITION_INT":
                with self._lock:
                    self._latest_gps = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)

            elif t == "COMMAND_ACK":
                with self._lock:
                    self._acks[msg.command] = msg.result

            elif t == "MISSION_ITEM_REACHED":
                with self._lock:
                    self._reached.add(msg.seq)

            elif t == "MISSION_CURRENT":
                with self._lock:
                    self._mission_current = msg.seq

            elif t == "HEARTBEAT":
                if msg.get_srcComponent() == ap_comp:
                    with self._lock:
                        self._hb = (msg.base_mode, msg.custom_mode)

            elif t in ("MISSION_COUNT", "MISSION_REQUEST", "MISSION_REQUEST_INT",
                       "MISSION_ITEM_INT", "MISSION_ITEM", "MISSION_ACK"):
                self._proto_q.put(msg)

    def stop_rx(self) -> None:
        self._rx_running = False
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)

    # ================================================================== #
    #  TELEMETRIA — interpolacja do czasu klatki
    # ================================================================== #
    def get_telemetry_at(self, ts: float) -> Optional[TelemetrySample]:
        """
        Interpolacja liniowa bufora telemetrii do czasu ts [s, monotonic].
        yaw interpolowany po najkrótszym łuku (obsługa zawijania ±pi).
        Zwraca None gdy ts poza zakresem bufora.
        """
        with self._lock:
            buf = list(self._tele_buf)
        if len(buf) < 2:
            return None
        times = [s.ts for s in buf]
        if ts < times[0] or ts > times[-1]:
            return None

        i = bisect.bisect_left(times, ts)
        if i == 0:
            return buf[0]
        if times[i] == ts:
            return buf[i]
        a, b = buf[i - 1], buf[i]
        span = b.ts - a.ts
        f = 0.0 if span <= 0 else (ts - a.ts) / span

        lat = a.lat + f * (b.lat - a.lat)
        lon = a.lon + f * (b.lon - a.lon)
        alt = a.alt + f * (b.alt - a.alt)
        roll = a.roll + f * (b.roll - a.roll)
        pitch = a.pitch + f * (b.pitch - a.pitch)
        yaw = a.yaw + f * _wrap_pi(b.yaw - a.yaw)    # najkrótszy łuk
        return TelemetrySample(ts, lat, lon, alt, roll, pitch, _wrap_pi(yaw))

    def get_latest_telemetry(self) -> Optional[TelemetrySample]:
        with self._lock:
            return self._tele_buf[-1] if self._tele_buf else None

    def get_current_coordinates(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            return self._latest_gps

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        s = self.get_latest_telemetry()
        return (s.roll, s.pitch, s.yaw) if s else None

    # ================================================================== #
    #  STAN — czytany z demuxa
    # ================================================================== #
    def is_armed(self) -> bool:
        with self._lock:
            hb = self._hb
        if hb is None:
            return False
        return (hb[0] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def get_current_mode(self) -> str:
        with self._lock:
            hb = self._hb
        if hb is None:
            return "UNKNOWN"
        for name, mid in self._mode_map.items():
            if mid == hb[1]:
                return name
        return "UNKNOWN"

    def reached_seqs(self) -> set:
        with self._lock:
            return set(self._reached)

    def clear_reached(self) -> None:
        with self._lock:
            self._reached.clear()

    # ================================================================== #
    #  KOMENDY — TX + poll demuxa
    # ================================================================== #
    def set_mode(self, mode_name: str, timeout: float = 5.0) -> bool:
        mode_id = self._mode_map.get(mode_name)
        if mode_id is None:
            self.logger.error("Unknown mode: %s (dostępne: %s)",
                              mode_name, list(self._mode_map))
            return False
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        end = time.time() + timeout
        while time.time() < end:
            if self.get_current_mode() == mode_name:
                self.logger.info("Tryb ustawiony: %s", mode_name)
                return True
            time.sleep(0.05)
        self.logger.warning("Zmiana trybu nie powiodła się (obecny: %s)",
                            self.get_current_mode())
        return False

    def _wait_ack(self, command: int, timeout: float = 5.0) -> Optional[int]:
        end = time.time() + timeout
        with self._lock:
            self._acks.pop(command, None)   # wyczyść stary
        while time.time() < end:
            with self._lock:
                if command in self._acks:
                    return self._acks[command]
            time.sleep(0.02)
        return None

    def arm(self) -> bool:
        if self.is_armed():
            self.logger.info("Już uzbrojony")
            return True
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        res = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if res != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.logger.error("Arm odrzucony: %s", res)
            return False
        time.sleep(1.0)
        if self.is_armed():
            self.logger.info("Uzbrojony")
            return True
        self.logger.warning("ACK OK, ale dron nie uzbrojony")
        return False

    def disarm(self) -> bool:
        if not self.is_armed():
            return True
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
        res = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        return res == mavutil.mavlink.MAV_RESULT_ACCEPTED

    def wait_item_reached(self, target_seqs: set, timeout: float) -> Optional[int]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lock:
                hit = self._reached & set(target_seqs)
            if hit:
                return min(hit)
            time.sleep(0.05)
        return None

    def set_current_waypoint(self, index: int, timeout: float = 5.0) -> bool:
        self.master.mav.mission_set_current_send(
            self.master.target_system, self.master.target_component, index)
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._mission_current == index:
                    return True
            time.sleep(0.05)
        self.logger.warning("Nie potwierdzono current waypoint %s", index)
        return False

    def mission_start(self) -> None:
        """MAV_CMD_MISSION_START — start/wznowienie misji AUTO."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

    def goto(self, lat: float, lon: float, alt: float) -> None:
        """Lot do punktu w trybie GUIDED (SET_POSITION_TARGET_GLOBAL_INT, tylko pozycja)."""
        type_mask = 0b0000111111111000   # ignoruj vel/accel/yaw, użyj pozycji
        self.master.mav.set_position_target_global_int_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7), int(lon * 1e7), float(alt),
            0, 0, 0, 0, 0, 0, 0, 0)

    def set_speed(self, speed_mps: float, speed_type: int = 1, throttle: float = -1.0) -> None:
        """MAV_CMD_DO_CHANGE_SPEED. speed_type: 0=airspeed, 1=groundspeed."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            speed_type, float(speed_mps), float(throttle), 0, 0, 0, 0)

    def set_message_rate(self, message_id: int, hz: float) -> None:
        interval_us = int(1e6 / hz)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0)

    def request_streams(self, attitude_hz: float = 50, gps_hz: float = 15,
                        mission_current_hz: float = 5) -> None:
        """Zażądaj strumieni potrzebnych do buforu telemetrii i nawigacji."""
        self.set_message_rate(30, attitude_hz)            # ATTITUDE
        self.set_message_rate(33, gps_hz)                 # GLOBAL_POSITION_INT
        self.set_message_rate(42, mission_current_hz)     # MISSION_CURRENT

    # ================================================================== #
    #  MISJA — protokół przez _proto_q (pod _upload_lock)
    # ================================================================== #
    def _drain_proto(self) -> None:
        try:
            while True:
                self._proto_q.get_nowait()
        except queue.Empty:
            pass

    def _proto_get(self, types, timeout: float = 5.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                msg = self._proto_q.get(timeout=max(0.0, end - time.time()))
            except queue.Empty:
                return None
            if msg.get_type() in types:
                return msg
        return None

    def get_mission(self) -> List[Dict[str, Any]]:
        with self._upload_lock:
            self._drain_proto()
            self.master.mav.mission_request_list_send(
                self.master.target_system, self.master.target_component)
            cnt = self._proto_get(("MISSION_COUNT",))
            if not cnt:
                self.logger.warning("Brak MISSION_COUNT")
                return []
            mission = []
            for i in range(cnt.count):
                self.master.mav.mission_request_int_send(
                    self.master.target_system, self.master.target_component, i, 0)
                item = self._proto_get(("MISSION_ITEM_INT", "MISSION_ITEM"))
                if not item:
                    self.logger.warning("Brak MISSION_ITEM %s", i)
                    break
                mission.append({
                    "seq": item.seq, "command": item.command, "frame": item.frame,
                    "current": item.current, "autocontinue": item.autocontinue,
                    "param1": item.param1, "param2": item.param2, "param3": item.param3,
                    "param4": item.param4, "param5": item.x, "param6": item.y,
                    "param7": item.z,
                })
            return mission

    def set_waypoints(self, waypoints: List[Dict[str, Any]]) -> bool:
        """
        Wgrywa misję. Punkt 0 (Home) dodawany automatycznie i ignorowany w misji.
        WAYPOINT: param1 (hold) z wp.get('param1', 0). Czyta protokół z _proto_q.
        """
        command_map = {
            "WAYPOINT": 16, "SET_SERVO": 183, "TAKEOFF": 22,
            "NAV_LOITER_UNLIM": mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            "NAV_LOITER_TIME": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        }
        home = {"command": 16, "frame": 0, "current": 1, "autocontinue": 1,
                "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                "param5": 0, "param6": 0, "param7": 0}
        all_wps = [home] + waypoints

        with self._upload_lock:
            self._drain_proto()
            with self._lock:
                current_idx = self._mission_current if self._mission_current else 1
            if current_idx >= len(all_wps):
                current_idx = len(all_wps) - 1

            self.master.mav.mission_count_send(
                self.master.target_system, self.master.target_component,
                len(all_wps), 0)

            for _ in range(len(all_wps)):
                req = self._proto_get(("MISSION_REQUEST", "MISSION_REQUEST_INT"))
                if not req:
                    self.logger.warning("Brak MISSION_REQUEST")
                    return False
                idx = req.seq
                wp = all_wps[idx]
                is_current = 1 if idx == current_idx else 0

                if "param5" in wp:   # surowe dane z get_mission
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, wp["frame"], wp["command"], is_current, wp["autocontinue"],
                        wp["param1"], wp["param2"], wp["param3"], wp["param4"],
                        int(wp["param5"]), int(wp["param6"]), wp["param7"])
                    continue

                cmd = command_map.get(wp["command"], wp["command"])
                if cmd == 16:        # WAYPOINT (param1 = hold, z dict albo 0)
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, is_current, 1,
                        wp.get("param1", 0), wp["acr"], 0, 0,
                        int(wp["lat"] * 1e7), int(wp["lon"] * 1e7), wp["alt"])
                elif cmd == 183:     # SET_SERVO
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, mavutil.mavlink.MAV_FRAME_MISSION,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1,
                        wp["channel"], wp["pwm"], 0, 0, 0, 0, 0)
                elif cmd == 22:      # TAKEOFF
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, is_current, 1,
                        0, 0, 0, 0, 0, 0, wp["alt"])
                elif cmd == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, is_current, 1,
                        0, 0, wp.get("radius", 0), wp.get("yaw", 0),
                        int(wp["lat"] * 1e7), int(wp["lon"] * 1e7), wp["alt"])
                elif cmd == mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME:
                    if "time" not in wp:
                        self.logger.error("NAV_LOITER_TIME wymaga 'time'")
                        return False
                    self.master.mav.mission_item_int_send(
                        self.master.target_system, self.master.target_component,
                        idx, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, is_current, 1,
                        wp["time"], 0, wp.get("radius", 0), wp.get("yaw", 0),
                        int(wp["lat"] * 1e7), int(wp["lon"] * 1e7), wp["alt"])
                else:
                    self.logger.error("Nieznana komenda: %s", wp["command"])
                    return False

            ack = self._proto_get(("MISSION_ACK",))
            if ack:
                self.logger.info("Mission ACK: %s", ack.type)
            return True

    def append_waypoints(self, new_wps: List[Dict[str, Any]]) -> bool:
        mission = self.get_mission()
        if mission:
            mission = mission[1:]    # pomiń Home
        mission.extend(new_wps)
        return self.set_waypoints(mission)

    def clear_mission(self) -> None:
        self.master.mav.mission_clear_all_send(
            self.master.target_system, self.master.target_component)
        self._proto_get(("MISSION_ACK",), timeout=5)
        self.logger.info("Misja wyczyszczona")

    def close(self) -> None:
        self.stop_rx()
        if hasattr(self, "master"):
            self.master.close()
            self.logger.info("MAVLink zamknięty")
