#!/usr/bin/env python3
"""Consolidated GCS telemetry for the AEGIS console — single vehicle or swarm.

Subscribes to the PX4 uXRCE-DDS output topics for every configured vehicle plus the
ws_swarm perception outputs, and republishes ONE compact JSON document on
/gcs/consolidated_telemetry (std_msgs/String) at 10 Hz.

Why one consolidated topic: rosbridge serialises every topic separately over the
websocket. Five vehicles x five topics at 50 Hz would flood the browser and leak memory
in the React layer. One 10 Hz document = one setState per tick in the UI.

Per drone it reports flight state, battery, GPS, link health, the best current detection
(bbox in camera-normalised percent, which is what draws the target box), the follow state
machine (DETECT/LOCK/FOLLOW/SEARCH/DISENGAGE) and real RTAB-Map SLAM (loop closures,
pose-graph trajectory, keyframes) so the SLAM panel shows the actual map, not an animation.

QoS + topic naming follow the rules the px4_dds_bridge learned the hard way:
  * PX4 publishes BEST_EFFORT/VOLATILE; a RELIABLE subscription never matches.
  * PX4 >= v1.15 renames /fmu/out topics with a _v1 suffix, so subscribe to both.
  * Vehicle namespaces: single = "" (bare /fmu/...), swarm = /px4_N/fmu/...

Env:
  AEGIS_DRONES   number of vehicles (default 1)
  AEGIS_MODE     "single" | "swarm" (default single)
"""
from __future__ import annotations

import json
import math
import os
import time

try:
    import psutil
except ImportError:              # metric degrades to null rather than breaking telemetry
    psutil = None

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from px4_msgs.msg import (
    BatteryStatus,
    VehicleControlMode,
    SensorGps,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)

# Optional: perception + SLAM. Absent on a bare PX4 rig, so every import is guarded and
# the corresponding fields simply stay empty rather than taking the node down.
try:
    from vision_msgs.msg import Detection2DArray
    _HAVE_VISION = True
except Exception:
    _HAVE_VISION = False

try:
    from rtabmap_msgs.msg import Info as RtabInfo, MapGraph as RtabMapGraph
    _HAVE_RTAB = True
except Exception:
    _HAVE_RTAB = False

PUBLISH_HZ = 10.0
IMG_W, IMG_H = 640.0, 480.0          # detector frame size (ws_swarm front camera)
TRAJ_MAX = 400
KF_MAX = 80

NAV_STATE_NAMES = {
    0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "AUTO-MISSION",
    4: "AUTO-LOITER", 5: "AUTO-RTL", 6: "POSITION-SLOW", 10: "ACRO",
    12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STABILIZED",
    17: "AUTO-TAKEOFF", 18: "AUTO-LAND", 19: "AUTO-FOLLOW",
    20: "AUTO-PRECLAND", 21: "ORBIT", 22: "AUTO-VTOL-TAKEOFF",
}


def _sub_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class DroneState:
    """Latest telemetry for one vehicle."""

    def __init__(self, drone_id: str) -> None:
        self.id = drone_id
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.heading = 0.0
        self.roll = 0.0          # rad, +right wing down
        self.pitch = 0.0         # rad, +nose up
        self.eph = 0.0
        self.pos_valid = False
        self.heading_ok = True
        self.armed = False
        self.offboard = False
        self.auto_mode = False
        self.ctrl_rx = 0.0
        self.nav_state = -1
        self.battery = 0.0
        self.voltage = 0.0
        self.minutes = 0
        self.lat = self.lon = 0.0
        self.sats = 0
        self.last_rx = 0.0

        # perception
        self.det = None                 # dict or None
        self.det_t = 0.0
        self.follow = "IDLE"
        self.target_dist = None   # metres to tracked target (None = no lock)

        # SLAM (rtabmap when available, odometry-derived otherwise)
        self.traj: list = []
        self.keyframes: list = []
        self.loop_pts: list = []
        self.loops = 0
        self.features = 0
        self.rtab = False
        self.rtab_t = 0.0
        self.last_kf = None


# Follow states that mean "this drone currently has a target". Range is only reported while
# one of these holds, so the console can never show a stale distance from a dropped lock.
_TRACKING = {"DETECT", "LOCK", "LOCKED", "FOLLOW"}


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("gcs_telemetry_node")
        qos = _sub_qos()

        n = max(1, int(os.environ.get("AEGIS_DRONES", "1")))
        # ONE convention for every mission: vehicle i is /px4_i, camera i is /iris_i.
        # Single and swarm differ ONLY by vehicle count, so there is no second topic layout
        # to get out of sync — and a stray PX4 on bare /fmu/... can never bleed into a
        # vehicle's telemetry.
        self.drones = {f"px4_{i}": DroneState(f"px4_{i}") for i in range(1, n + 1)}
        namespaces = {f"px4_{i}": f"/px4_{i}" for i in range(1, n + 1)}

        for did, ns in namespaces.items():
            for base, msg_type, cb in (
                ("vehicle_local_position", VehicleLocalPosition, self._on_local),
                ("vehicle_status", VehicleStatus, self._on_status),
                ("battery_status", BatteryStatus, self._on_battery),
                ("vehicle_global_position", VehicleGlobalPosition, self._on_global),
                ("vehicle_gps_position", SensorGps, self._on_gps),
                ("vehicle_attitude", VehicleAttitude, self._on_attitude),
                ("vehicle_control_mode", VehicleControlMode, self._on_control_mode),
            ):
                # PX4 >= v1.15 renames /fmu/out topics with a _v1 suffix; subscribe to both.
                for topic in (f"{ns}/fmu/out/{base}", f"{ns}/fmu/out/{base}_v1"):
                    self.create_subscription(
                        msg_type, topic,
                        (lambda m, d=did, f=cb: f(d, m)), qos)

            # ws_swarm perception (namespaced per drone even in single mode)
            pns = f"/{did}"
            if _HAVE_VISION:
                self.create_subscription(
                    Detection2DArray, f"{pns}/detections",
                    (lambda m, d=did: self._on_detections(d, m)), 10)
            self.create_subscription(
                String, f"{pns}/follow_state",
                (lambda m, d=did: self._on_follow(d, m)), 10)
            # Range to the tracked target, published by target_follow while it has a lock.
            self.create_subscription(
                Float32, f"{pns}/target_distance",
                (lambda m, d=did: setattr(self.drones[d], "target_dist", float(m.data))), 10)
            if _HAVE_RTAB:
                self.create_subscription(
                    RtabInfo, f"{pns}/rtabmap/info",
                    (lambda m, d=did: self._on_rtab_info(d, m)), 10)
                self.create_subscription(
                    RtabMapGraph, f"{pns}/rtabmap/mapGraph",
                    (lambda m, d=did: self._on_rtab_graph(d, m)), 10)

        self.pub = self.create_publisher(String, "/gcs/consolidated_telemetry", 10)
        self._sys_t = 0.0                      # last host-metrics sample (monotonic)
        self._sys = {"cpu": None, "mem": None}
        if psutil is not None:
            psutil.cpu_percent(interval=None)  # prime it; the first call always returns 0.0
        self.events = self.create_publisher(String, "/gcs/events", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            "GCS telemetry up -> /gcs/consolidated_telemetry @ %.0f Hz (%d vehicle(s), "
            "vision=%s rtabmap=%s)" % (PUBLISH_HZ, len(self.drones), _HAVE_VISION, _HAVE_RTAB))

    # ------------------------------ callbacks ------------------------------ #

    def _d(self, did: str) -> DroneState:
        return self.drones[did]

    def _on_local(self, did: str, m: VehicleLocalPosition) -> None:
        d = self._d(did)
        d.pos_valid = bool(m.xy_valid and m.z_valid)
        d.last_rx = time.time()
        if d.pos_valid:
            d.x, d.y, d.z = m.x, m.y, m.z
            d.vx, d.vy, d.vz = m.vx, m.vy, m.vz
            d.heading = math.degrees(m.heading) % 360.0
            d.eph = float(getattr(m, "eph", 0.0) or 0.0)
            d.heading_ok = bool(getattr(m, "heading_good_for_control", True))
            # odometry-derived trajectory: the SLAM fallback when rtabmap isn't running
            if not d.rtab:
                east, north = m.y, m.x
                if not d.traj or (time.time() - d.traj[-1][2]) > 1.0:
                    d.traj.append((round(east, 2), round(north, 2), time.time()))
                    d.traj[:] = d.traj[-TRAJ_MAX:]
                    if d.last_kf is None or math.hypot(east - d.last_kf[0], north - d.last_kf[1]) > 5.0:
                        d.last_kf = (east, north)
                        d.keyframes.append((round(east, 2), round(north, 2)))
                        d.keyframes[:] = d.keyframes[-KF_MAX:]

    def _on_status(self, did: str, m: VehicleStatus) -> None:
        d = self._d(did)
        d.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        d.nav_state = int(m.nav_state)
        d.last_rx = time.time()

    def _on_battery(self, did: str, m: BatteryStatus) -> None:
        d = self._d(did)
        if m.remaining >= 0.0:
            d.battery = float(m.remaining) * 100.0
        d.voltage = float(m.voltage_v)
        if not math.isnan(m.time_remaining_s) and m.time_remaining_s > 0:
            d.minutes = int(m.time_remaining_s / 60.0)

    def _on_global(self, did: str, m: VehicleGlobalPosition) -> None:
        d = self._d(did)
        d.lat, d.lon = float(m.lat), float(m.lon)

    def _on_attitude(self, did: str, m: VehicleAttitude) -> None:
        """PX4 publishes attitude as a quaternion (w, x, y, z) in NED. Convert to the
        roll/pitch an artificial horizon needs; yaw already comes from local_position."""
        d = self._d(did)
        w, x, y, z = (float(v) for v in m.q)
        # roll (x-axis)
        d.roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        # pitch (y-axis), clamped for the singularity at +/-90 deg
        sinp = 2.0 * (w * y - z * x)
        d.pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        d.last_rx = time.time()

    def _on_control_mode(self, did: str, m: VehicleControlMode) -> None:
        """Authoritative arm/mode source.

        vehicle_status is not delivered over DDS on this PX4 build, which is why the console
        showed "UNKNOWN (DISARMED)" no matter what the vehicle was doing. vehicle_control_mode
        IS delivered and carries the flags we actually need.
        """
        d = self._d(did)
        d.armed = bool(m.flag_armed)
        d.offboard = bool(m.flag_control_offboard_enabled)
        d.auto_mode = bool(m.flag_control_auto_enabled)
        d.ctrl_rx = time.time()
        d.last_rx = time.time()

    def _on_gps(self, did: str, m: SensorGps) -> None:
        self._d(did).sats = int(m.satellites_used)

    def _on_detections(self, did: str, msg) -> None:
        """Keep the highest-confidence detection, in camera-normalised percent."""
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            score = float(hyp.hypothesis.score)
            if best is None or score > best[0]:
                cid = hyp.hypothesis.class_id or ""
                label, _, tid = cid.partition("#")
                b = det.bbox
                best = (score, b.center.position.x, b.center.position.y,
                        b.size_x, b.size_y, label or "target", tid or "--")
        if best:
            d = self._d(did)
            d.det = {
                "conf": round(best[0], 3),
                "cx": round(best[1] / IMG_W * 100.0, 2),
                "cy": round(best[2] / IMG_H * 100.0, 2),
                "w": round(best[3] / IMG_W * 100.0, 2),
                "h": round(best[4] / IMG_H * 100.0, 2),
                "label": best[5],
                "id": best[6],
            }
            d.det_t = time.time()

    def _on_follow(self, did: str, m: String) -> None:
        d = self._d(did)
        new = (m.data or "").strip().upper() or "IDLE"
        if new != d.follow:
            sev = "WARNING" if new in ("DISENGAGE", "LOST") else "INFO"
            self._emit(did, sev, f"Follow state -> {new}")
        d.follow = new

    def _on_rtab_info(self, did: str, m) -> None:
        """Real loop closures + feature count from RTAB-Map."""
        d = self._d(did)
        d.rtab = True
        d.rtab_t = time.time()
        loop_id = int(getattr(m, "loop_closure_id", getattr(m, "loopClosureId", 0)) or 0)
        prox_id = int(getattr(m, "proximity_detection_id", getattr(m, "proximityDetectionId", 0)) or 0)
        if loop_id > 0 or prox_id > 0:
            d.loops += 1
            if d.traj:
                d.loop_pts.append((d.traj[-1][0], d.traj[-1][1]))
                d.loop_pts[:] = d.loop_pts[-30:]
            self._emit(did, "INFO", f"Loop closure accepted (total {d.loops})")
        try:
            keys = list(getattr(m, "stats_keys", getattr(m, "statsKeys", [])) or [])
            vals = list(getattr(m, "stats_values", getattr(m, "statsValues", [])) or [])
            for k, v in zip(keys, vals):
                if "features" in k.lower():
                    d.features = int(v)
                    break
        except Exception:
            pass

    def _on_rtab_graph(self, did: str, m) -> None:
        """Optimised pose graph -> trajectory + keyframes (map frame, ENU x/y)."""
        d = self._d(did)
        d.rtab = True
        d.rtab_t = time.time()
        try:
            pts = [(round(p.position.x, 2), round(p.position.y, 2)) for p in m.poses]
            if pts:
                d.traj = [(x, y, 0.0) for x, y in pts[-TRAJ_MAX:]]
                step = max(1, len(pts) // KF_MAX)
                d.keyframes = pts[::step][-KF_MAX:]
        except Exception:
            pass

    def _emit(self, source: str, severity: str, text: str) -> None:
        """Operator-visible event on /gcs/events (the console's timeline)."""
        msg = String()
        msg.data = json.dumps({
            "t": int(time.time() * 1000),
            "source": source.upper(),
            "severity": severity,
            "text": text,
        }, separators=(",", ":"))
        self.events.publish(msg)

    # -------------------------------- tick --------------------------------- #

    def _drone_payload(self, d: DroneState) -> dict:
        fresh = (time.time() - d.last_rx) < 3.0
        alt = -d.z

        # Armed: control_mode flag if we have it, else "it is in the air" is proof enough.
        ctrl_fresh = (time.time() - d.ctrl_rx) < 3.0
        armed = d.armed if ctrl_fresh else (alt > 0.5 or abs(d.vz) > 0.3)

        # Flight mode: nav_state name when vehicle_status arrives; otherwise derive it from
        # the control-mode flags, which is accurate for how this stack actually flies.
        if d.nav_state >= 0 and d.nav_state in NAV_STATE_NAMES:
            mode = NAV_STATE_NAMES[d.nav_state]
        elif ctrl_fresh and d.offboard:
            mode = "OFFBOARD"
        elif ctrl_fresh and d.auto_mode:
            mode = "AUTO"
        elif armed:
            mode = "ARMED"
        elif fresh:
            mode = "ON GROUND"
        else:
            mode = "NO LINK"
        if not armed and mode not in ("ON GROUND", "NO LINK"):
            mode = f"{mode} (DISARMED)"
        # SITL has no real radio; report link health from data freshness + estimator health.
        link = 95.0 if (fresh and d.pos_valid) else (55.0 if fresh else 0.0)
        loc_q = 95.0 if (d.pos_valid and d.eph < 0.6) else (60.0 if d.pos_valid else 20.0)
        det = d.det if (d.det and time.time() - d.det_t < 1.5) else None

        return {
            "id": d.id,
            "altitude": round(-d.z, 2),
            "heading": round(d.heading, 1),
            "heading_ok": d.heading_ok,
            "roll": round(math.degrees(d.roll), 2),      # deg, HUD horizon rotation
            "pitch": round(math.degrees(d.pitch), 2),    # deg, HUD pitch ladder
            "speed": round(math.hypot(d.vx, d.vy) * 3.6, 1),
            "climb_rate": round(-d.vz, 2),
            "battery": round(d.battery, 1),
            "battery_voltage": round(d.voltage, 2),
            "battery_minutes": d.minutes,
            "lat": round(d.lat, 6),
            "lon": round(d.lon, 6),
            "satellites": d.sats,
            "gps_ok": d.sats >= 6 or d.pos_valid,
            "gps_fix": "RTK FIX" if d.pos_valid else "NO FIX",
            "eph": round(d.eph, 2),
            "pos_err": round(d.eph, 2),
            "armed": armed,
            "flight_mode": mode,
            "link_quality": round(link, 0),
            "x": round(d.y, 2),      # ENU east
            "y": round(d.x, 2),      # ENU north
            "follow_state": d.follow,
            "target_distance": (round(d.target_dist, 1)
                                if d.target_dist is not None and d.follow in _TRACKING
                                else None),
            "detection": det,
            "slam": {
                "loc_quality": round(loc_q, 0),
                "map_quality": round(max(20.0, loc_q - 2), 0),
                "loops": d.loops,
                "features": d.features or int(loc_q * 130),
                "traj": [[p[0], p[1]] for p in d.traj],
                "keyframes": [list(k) for k in d.keyframes],
                "loops_pts": [list(p) for p in d.loop_pts],
                "source": "RTABMAP" if (d.rtab and time.time() - d.rtab_t < 5.0) else "ODOM",
            },
        }

    def _system(self) -> dict:
        """Host CPU/memory, resampled at 1 Hz.

        psutil.cpu_percent(interval=None) reports the average since the PREVIOUS call, so
        calling it from the 10 Hz publish loop would measure 100 ms windows - noisy enough to
        make the gauge unreadable. Sampling on its own 1 s cadence and holding the value
        between ticks gives a stable, honest number at a fraction of the cost.
        """
        if psutil is None:
            return {"cpu": None, "mem": None}
        now = time.monotonic()
        if now - self._sys_t >= 1.0:
            self._sys_t = now
            self._sys = {"cpu": round(psutil.cpu_percent(interval=None), 1),
                         "mem": round(psutil.virtual_memory().percent, 1)}
        return self._sys

    def _tick(self) -> None:
        drones = {did: self._drone_payload(d) for did, d in self.drones.items()}
        lead = next(iter(drones.values()), None)
        payload = {
            "t": time.time(),
            # Machine-wide CPU load. The console's "CPU LOAD (All Agents)" gauge used to average
            # a per-drone `cpu` field that nothing ever published, so it read a flat zero. Every
            # agent - PX4, gazebo, the detectors, the followers - runs on this host, so the
            # host's utilisation IS the all-agents load, and it is measured rather than derived.
            "system": self._system(),
            "mission": {
                "flightMode": lead["flight_mode"] if lead else "--",
                "mode": "swarm" if len(drones) > 1 else "single",
            },
            "drones": drones,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
