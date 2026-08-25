#!/usr/bin/env python3
"""Local safety / failsafe supervisor.

Runs alongside the mission rather than gating it: missions are never refused up
front, but these hazards are watched continuously and override the drones when
they occur.

  * heartbeat stale     -> hold, then RTL if it stays stale
  * inter-drone too close -> repulsion: the yielding drone backs away until clear
  * explicit abort      -> RTL on all known drones

Overrides are published as DroneTask with PRIORITY_SAFETY and are REFRESHED while
the hazard persists. drone_controller expires an override that stops being
refreshed, so control returns automatically once the hazard clears — a safety
hold can never deadlock a drone.

Separation uses a SHARED world frame (drone_origins), since each drone reports
position in its own local frame.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String
from geometry_msgs.msg import Point
from swarm_msgs.msg import Heartbeat, DroneTask, DroneStatus
from px4_msgs.msg import VehicleLocalPosition

# PX4 DDS publishes best-effort; match it for the estimator-validity subscription.
_PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.hb_timeout = float(self.declare_parameter('heartbeat_timeout', 3.0).value)
        self.rtl_timeout = float(self.declare_parameter('rtl_timeout', 6.0).value)
        # Must stay BELOW the validator's minimum formation spacing, otherwise a
        # legal formation (slots exactly `spacing` apart) would sit inside the
        # separation guard and be pushed around forever. `formation_min_spacing`
        # mirrors schema.MIN_SPACING; the invariant is checked at startup below so a
        # misconfiguration surfaces as a warning instead of silent formation thrash.
        self.min_separation = float(self.declare_parameter('min_separation', 2.0).value)
        self.low_batt = float(self.declare_parameter('low_battery_frac', 0.20).value)
        self.formation_min_spacing = float(
            self.declare_parameter('formation_min_spacing', 2.5).value)
        if self.min_separation >= self.formation_min_spacing:
            self.get_logger().warn(
                f'min_separation {self.min_separation} m >= formation floor '
                f'{self.formation_min_spacing} m: legal formations may trip avoidance. '
                f'Set min_separation below the formation spacing floor.')

        # Shared-frame origins: ['px4_1:0:0', 'px4_2:3:0', ...] (id:north:east).
        self.origins = {}
        for s in (self.declare_parameter(
                'drone_origins', [], ParameterDescriptor(dynamic_typing=True)).value or []):
            p = s.split(':')
            if len(p) == 3:
                self.origins[p[0]] = (float(p[1]), float(p[2]))

        self.tracked = {}     # drone_id -> {'last_seen', 'held', 'rtl'}
        self.task_pubs = {}
        self.world = {}       # drone_id -> (north, east, up) shared frame
        self._sep_warned = set()

        self.create_subscription(Heartbeat, '/swarm/heartbeat', self._on_heartbeat, 10)
        self.create_subscription(String, '/mission/abort', self._on_abort, 10)

        # 2 Hz: fast enough to refresh overrides well inside drone_controller's TTL.
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'safety_node up (heartbeat, abort, separation >= {self.min_separation} m)')

    # ---------- inputs ----------
    def _on_heartbeat(self, msg: Heartbeat):
        now = self._now()
        if msg.drone_id not in self.tracked:
            # First sighting: pre-create publisher + status sub so an override is
            # never lost to a discovery race when a hazard actually fires.
            self.tracked[msg.drone_id] = {'last_seen': now, 'held': False, 'rtl': False}
            self.task_pubs.setdefault(
                msg.drone_id, self.create_publisher(DroneTask, f'/{msg.drone_id}/task', 10))
            self.create_subscription(
                DroneStatus, f'/{msg.drone_id}/status', self._on_status, 10)
            # Estimator-degradation watch: PX4 EKF sets xy_valid=false when the local
            # position estimate is untrustworthy -> hold until it recovers.
            self.create_subscription(
                VehicleLocalPosition, f'/{msg.drone_id}/fmu/out/vehicle_local_position_v1',
                (lambda d: lambda m: self._on_localpos(d, m))(msg.drone_id), _PX4_QOS)
            self.get_logger().info(f'tracking {msg.drone_id}')
        rec = self.tracked[msg.drone_id]
        rec['last_seen'] = now
        if rec['held'] or rec['rtl']:
            self.get_logger().info(f'[{msg.drone_id}] heartbeat recovered')
        rec['held'] = False
        rec['rtl'] = False

    def _on_status(self, msg: DroneStatus):
        o_n, o_e = self.origins.get(msg.drone_id, (0.0, 0.0))
        # DroneStatus.position is local ENU (x=East, y=North, z=Up)
        self.world[msg.drone_id] = (msg.position.y + o_n, msg.position.x + o_e,
                                    msg.position.z)
        # Low-battery failsafe: bring the drone home before it drops out of the sky.
        rec = self.tracked.get(msg.drone_id)
        if (rec and not rec['rtl'] and 0.0 <= msg.battery_remaining < self.low_batt):
            self.get_logger().error(
                f'[{msg.drone_id}] battery {msg.battery_remaining:.0%} < '
                f'{self.low_batt:.0%} -> RTL')
            rec['rtl'] = True
            self._send(msg.drone_id, 'rtl')

    def _on_localpos(self, drone_id, m):
        """Estimator degradation: hold if PX4 says the local position is invalid."""
        rec = self.tracked.get(drone_id)
        if rec and not rec['rtl'] and not m.xy_valid and not rec['held']:
            self.get_logger().error(f'[{drone_id}] estimator xy invalid -> HOLD')
            rec['held'] = True
            self._send(drone_id, 'hold')

    def _on_abort(self, msg: String):
        self.get_logger().warn(f'ABORT received: "{msg.data}" -> RTL all drones')
        for drone_id, rec in self.tracked.items():
            self._send(drone_id, 'rtl')
            rec['rtl'] = True

    # ---------- watchdogs ----------
    def _tick(self):
        self._check_timeouts()
        self._check_separation()

    def _check_timeouts(self):
        now = self._now()
        for drone_id, rec in self.tracked.items():
            age = now - rec['last_seen']
            if age > self.rtl_timeout:
                if not rec['rtl']:
                    self.get_logger().error(f'[{drone_id}] stale {age:.1f}s -> RTL')
                    rec['rtl'] = True
                self._send(drone_id, 'rtl')      # refresh while stale
            elif age > self.hb_timeout:
                if not rec['held']:
                    self.get_logger().warn(f'[{drone_id}] stale {age:.1f}s -> HOLD')
                    rec['held'] = True
                self._send(drone_id, 'hold')     # refresh while stale

    def _check_separation(self):
        """Repulsion avoidance: push the YIELDING drone directly AWAY from the
        intruder (same altitude), refreshed while breached.

        Holding is not avoidance — a held drone can be flown into. Instead the
        later-id drone actively backs off along the line joining the pair until
        separation is restored, at which point the override stops refreshing and
        normal control resumes. This increases distance every cycle, so it can
        neither collide (unlike hold-one-continue-other) nor deadlock (unlike
        hold-both). Only one of each pair yields, so they don't chase each other.
        """
        ids = sorted(d for d in self.world if not self.tracked.get(d, {}).get('rtl'))
        # drone -> intruder it must back away from (nearest breaching partner)
        yield_from = {}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                dist = math.dist(self.world[a], self.world[b])
                if dist < self.min_separation:
                    prev = yield_from.get(b)
                    if prev is None or dist < math.dist(self.world[b], self.world[prev]):
                        yield_from[b] = a
                    if (a, b) not in self._sep_warned:
                        # Routine avoidance, NOT a failure — log at INFO so it does
                        # not look like an error. (Chronic breaches would still show
                        # up as a steady stream of these.)
                        self.get_logger().info(
                            f'avoidance: {a}<->{b} {dist:.2f} m -> {b} easing clear')
                        self._sep_warned.add((a, b))
        for b, a in yield_from.items():
            self._send_avoid(b, a)
        if not yield_from:
            self._sep_warned.clear()

    def _send_avoid(self, drone, intruder):
        """Command `drone` to a point just outside min_separation, directly away
        from `intruder`, at its current altitude."""
        bn, be, bu = self.world[drone]
        an, ae, _ = self.world[intruder]
        vn, ve = bn - an, be - ae
        norm = math.hypot(vn, ve)
        if norm < 1e-3:                         # exactly overlapping: pick a fixed axis
            vn, ve, norm = 1.0, 0.0, 1.0
        reach = self.min_separation + 1.5       # back off to a safe margin
        wn, we = bn + vn / norm * reach, be + ve / norm * reach
        o_n, o_e = self.origins.get(drone, (0.0, 0.0))
        # world NED -> this drone's local ENU goto (x=East, y=North, z=up), same alt
        self._send(drone, 'goto', Point(x=float(we - o_e), y=float(wn - o_n), z=float(bu)))

    # ---------- output ----------
    def _send(self, drone_id, action, target=None):
        pub = self.task_pubs.get(drone_id)
        if pub is None:
            pub = self.create_publisher(DroneTask, f'/{drone_id}/task', 10)
            self.task_pubs[drone_id] = pub
        task = DroneTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.drone_id = drone_id
        task.action = action
        task.priority = DroneTask.PRIORITY_SAFETY
        if target is not None:
            task.target = target
            task.target_altitude = target.z
        pub.publish(task)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
