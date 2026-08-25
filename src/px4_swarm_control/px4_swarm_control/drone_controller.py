#!/usr/bin/env python3
"""Per-drone PX4 offboard executor.

Stage 1 goal: prove ONE drone end-to-end. It is written namespace-first, so
running a second copy with `drone_id:=px4_2` gives a second independent drone
with zero code changes.

Responsibilities (deliberately narrow):
  * stream OffboardControlMode + TrajectorySetpoint at a fixed rate (PX4 needs
    this heartbeat or it drops offboard),
  * arm and switch to offboard once a few setpoints have been sent,
  * follow the single active DroneTask (takeoff / goto / hold / rtl / land),
  * publish a compact DroneStatus for the rest of the swarm.

It does NOT plan, coordinate, or talk to the LLM. Higher layers send DroneTask.
"""

import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from swarm_msgs.msg import DroneTask, DroneStatus


# PX4 uses NED internally; our tasks use ENU (ROS convention). Convert at the edge.
def enu_to_ned(x, y, z):
    return (y, x, -z)


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # --- parameters (namespace-ready) ---
        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.takeoff_alt = float(self.declare_parameter('default_takeoff_alt', 3.0).value)

        # Runtime safety envelope, enforced on EVERY setpoint (not just at plan
        # time). Mirrors mission_interface.schema bounds; overridable per drone.
        self.geofence_radius = float(self.declare_parameter('geofence_radius', 200.0).value)
        self.max_altitude = float(self.declare_parameter('max_altitude', 120.0).value)
        # Cruise speed: the published setpoint is RAMPED toward the target at this
        # rate rather than jumping to it. That gives real speed control in offboard
        # position mode, and keeps a formation coherent in transit (all drones
        # advance at the same rate instead of arriving at different times).
        self.cruise_speed = float(self.declare_parameter('cruise_speed', 3.0).value)

        # PX4 SITL sets MAV_SYS_ID = instance + 1, and the DDS namespace is
        # px4_<instance>. So VehicleCommand.target_system for px4_N is N+1.
        # target_system:=0 (default) auto-derives; set >0 to override.
        ts_param = int(self.declare_parameter('target_system', 0).value)
        self.target_system = ts_param if ts_param > 0 else self._derive_target_system()

        ns = f'/{self.drone_id}'

        # PX4 uXRCE-DDS bridge topics are best-effort; match that QoS exactly.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- PX4 publishers ---
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', px4_qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', px4_qos)
        self.pub_command = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', px4_qos)

        # --- PX4 subscribers ---
        # This PX4 (v1.17) uses message-versioned topics; feedback lives on the
        # "_v1" names. VehicleLocalPosition is wire-compatible with our pinned
        # px4_msgs and delivers reliably. VehicleStatus is NOT wire-compatible
        # (firmware added fields px4_msgs lacks), so it cannot be subscribed
        # without dropped samples + RTPS errors. We therefore derive health from
        # position + offboard state instead of arming_state/nav_state.
        self.create_subscription(
            VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_v1',
            self._on_local_position, px4_qos)

        # --- swarm interface ---
        self.create_subscription(DroneTask, f'{ns}/task', self._on_task, 10)
        self.pub_status = self.create_publisher(DroneStatus, f'{ns}/status', 10)

        # --- state ---
        self.local_pos = VehicleLocalPosition()
        self.setpoint_count = 0
        self.offboard_active = False
        # Active target in NED. Default: hold at origin altitude until first task.
        self.target_ned = [0.0, 0.0, -self.takeoff_alt]
        # Ramped setpoint actually published (None until first position fix).
        self.sp_ned = None
        self.task_speed = 0.0        # per-task override; 0 = use self.cruise_speed
        self.task_yaw = float('nan')  # NaN = face direction of travel
        # Safety override, TIME-LIMITED so it can never deadlock the drone.
        # A higher-priority task blocks lower-priority ones, but only until
        # override_ttl elapses. safety_manager REFRESHES its override while the
        # hazard persists; when it stops refreshing, normal control resumes.
        self.override_ttl = float(self.declare_parameter('override_ttl', 3.0).value)
        self.override_priority = 0
        self.override_until = 0.0
        self._clamp_warned = False
        self.target_yaw = 0.0
        self.action = 'hold'

        period = 1.0 / self.rate_hz
        self.create_timer(period, self._control_loop)
        self.create_timer(1.0, self._publish_status)  # 1 Hz status is plenty

        self.get_logger().info(f'drone_controller up for {self.drone_id} (ns={ns})')

    def _derive_target_system(self):
        """px4_N -> MAV_SYS_ID N+1 (PX4 SITL convention).

        Use only the TRAILING number so 'px4_1' -> instance 1 (not 41 from the
        '4' in 'px4'); MAV_SYS_ID is then 2.
        """
        m = re.search(r'(\d+)$', self.drone_id)
        return (int(m.group(1)) if m else 0) + 1

    # ---------- subscriptions ----------
    def _on_local_position(self, msg):
        self.local_pos = msg

    def _on_task(self, msg: DroneTask):
        """Latest task wins. Safety tasks arrive on the same topic with high priority."""
        # Refuse lower-priority tasks while a safety override is live (not expired).
        now = self._now_s()
        active = self.override_priority if now < self.override_until else 0
        if msg.priority < active:
            self.get_logger().warn(
                f'[{self.drone_id}] ignoring "{msg.action}" (priority {msg.priority}) '
                f'— safety override {active} active')
            return
        if msg.priority > DroneTask.PRIORITY_NORMAL:
            self.override_priority = msg.priority
            self.override_until = now + self.override_ttl
        else:
            self.override_priority = 0
            self.override_until = 0.0

        if msg.action != self.action:      # log only on change (safety refreshes often)
            self.get_logger().info(f'[{self.drone_id}] task -> {msg.action}')
        self.action = msg.action
        self.task_speed = float(getattr(msg, 'cruise_speed', 0.0) or 0.0)
        self.task_yaw = float(getattr(msg, 'yaw', float('nan')))

        if msg.action == 'takeoff':
            alt = msg.target_altitude if msg.target_altitude > 0.0 else self.takeoff_alt
            # Hold current horizontal position (already NED), climb to alt.
            self.target_ned = [self.local_pos.x, self.local_pos.y, -alt]
        elif msg.action == 'goto':
            self.target_ned = list(enu_to_ned(msg.target.x, msg.target.y, msg.target_altitude))
        elif msg.action == 'hold':
            # Freeze at current position.
            self.target_ned = [self.local_pos.x, self.local_pos.y, self.local_pos.z]
        elif msg.action == 'rtl':
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        elif msg.action == 'land':
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ---------- main loop ----------
    def _control_loop(self):
        # PX4 requires the offboard heartbeat regardless of state.
        self._publish_offboard_mode()

        # Open-loop arm + offboard engage, RETRIED until actually airborne.
        # This PX4 build does not deliver vehicle_status over DDS, so we cannot
        # read arming_state; instead we detect success via altitude. A one-shot
        # arm is unreliable: the command may be dropped, or PX4 may simply not be
        # armable at that instant (fresh boot / EKF not ready). So we keep
        # re-sending SET_MODE(offboard)+ARM every ~0.5 s (after a 1 s setpoint
        # warm-up) as long as we want to fly but are still on the ground.
        # Re-sending when already armed/offboard is harmless (idempotent).
        altitude_up = -self.local_pos.z

        want_flight = self.action in ('takeoff', 'goto', 'hold')
        if want_flight and altitude_up < 1.0:
            warmup = int(1.0 * self.rate_hz)
            resend_every = max(1, int(0.5 * self.rate_hz))
            if self.setpoint_count >= warmup and self.setpoint_count % resend_every == 0:
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # offboard
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.offboard_active = True

        # rtl/land hand control back to PX4 flight modes; stop streaming setpoints.
        if self.action not in ('rtl', 'land'):
            self._publish_setpoint()

        self.setpoint_count += 1

    # ---------- PX4 message helpers ----------
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard.publish(msg)

    def _clamp_to_envelope(self, ned):
        """Continuously enforce the safety envelope on every setpoint.

        Safety rides WITH the mission: rather than refusing missions up front, any
        commanded position is clamped into the geofence / altitude ceiling here, at
        the control rate, whatever the planner asked for.
        """
        n, e, d = ned
        clamped = False
        horiz = (n * n + e * e) ** 0.5
        if horiz > self.geofence_radius:
            scale = self.geofence_radius / horiz
            n, e, clamped = n * scale, e * scale, True
        if -d > self.max_altitude:      # d is NED down; -d is altitude
            d, clamped = -self.max_altitude, True
        if clamped and not self._clamp_warned:
            self.get_logger().warn(
                f'[{self.drone_id}] setpoint clamped to safety envelope '
                f'(geofence {self.geofence_radius} m, ceiling {self.max_altitude} m)')
            self._clamp_warned = True
        elif not clamped:
            self._clamp_warned = False
        return [n, e, d]

    def _publish_setpoint(self):
        goal = self._clamp_to_envelope(self.target_ned)

        # Start the ramp at the current position so we never jump on first command.
        if self.sp_ned is None:
            self.sp_ned = [self.local_pos.x, self.local_pos.y, self.local_pos.z]

        # Advance the setpoint toward the goal at cruise speed (speed control).
        speed = self.task_speed if self.task_speed > 0.0 else self.cruise_speed
        step = speed / self.rate_hz
        dn, de, dd = (goal[0] - self.sp_ned[0], goal[1] - self.sp_ned[1],
                      goal[2] - self.sp_ned[2])
        dist = (dn * dn + de * de + dd * dd) ** 0.5
        if dist <= step or dist < 1e-6:
            self.sp_ned = list(goal)
        else:
            f = step / dist
            self.sp_ned = [self.sp_ned[0] + dn * f, self.sp_ned[1] + de * f,
                           self.sp_ned[2] + dd * f]

        # Yaw: explicit if given, else face the direction of travel.
        if self.task_yaw == self.task_yaw:          # not NaN
            self.target_yaw = self.task_yaw
        elif (dn * dn + de * de) ** 0.5 > 0.5:      # only turn for real motion
            self.target_yaw = math.atan2(de, dn)    # NED: yaw 0 = North

        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(self.sp_ned[0]), float(self.sp_ned[1]), float(self.sp_ned[2])]
        msg.yaw = float(self.target_yaw)
        self.pub_setpoint.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_command.publish(msg)

    # ---------- status out ----------
    def _publish_status(self):
        s = DroneStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.drone_id = self.drone_id
        # nav_state/arming_state come from VehicleStatus, which is not wire-
        # compatible with our pinned px4_msgs (see subscriber note). Left as 0
        # (unknown); health is conveyed via offboard_active + position instead.
        s.nav_state = 0
        s.arming_state = 0
        s.offboard_active = self.offboard_active
        # NED -> ENU for the shared position.
        s.position = Point(x=float(self.local_pos.y),
                           y=float(self.local_pos.x),
                           z=float(-self.local_pos.z))
        s.battery_remaining = -1.0  # TODO(stage2): wire BatteryStatus topic
        self.pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
