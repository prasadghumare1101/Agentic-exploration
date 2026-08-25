#!/usr/bin/env python3
"""Target follow — lock one tracked target and pursue it with image-space visual servoing.

Fixes over the previous revision:
  * LOCK BY ID PERSISTENCE, not by confidence. A frozen CSRT seed score can never clear a
    0.55 gate, so the old node never locked at all and sat in contact-freeze/search forever.
  * BLIND-YAW BUDGET. The old open-loop yaw on a stale ex ran at full HFOV gain for the whole
    lost_timeout (~69 deg), sweeping the target out of frame and self-perpetuating the loss.
    Now: decayed, budgeted, and it HOLDS rather than sweeps once the budget is spent.
  * DOWN-CAM CONTROL LAW (camera_mount='down'). Under a downward camera ex/ey are ground
    offsets, not bearing/range. Translate on both axes; do not yaw to centre.
  * SINGLE confidence gate. min_confidence is the only score threshold in the follower.
  * Search no longer strands the drone: contact freeze is bounded, and a contact that never
    locks is logged loudly instead of silently holding forever.
"""

import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String

from vision_msgs.msg import Detection2DArray
from swarm_msgs.msg import DroneTask
from px4_msgs.msg import VehicleLocalPosition


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class TargetFollow(Node):
    def __init__(self):
        super().__init__('target_follow')

        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.lost_timeout = float(self.declare_parameter('lost_timeout', 3.0).value)

        # ---- SINGLE score gate. There is no second "lock_confidence" gate any more: a lock is
        # earned by an id persisting for lock_frames consecutive detections, which is a far
        # better signal of a real target than a frozen tracker seed score. ----
        self.min_conf = float(self.declare_parameter('min_confidence', 0.30).value)
        self.lock_frames = int(self.declare_parameter('lock_frames', 3).value)
        self.lock_window = float(self.declare_parameter('lock_window', 1.5).value)

        self.follow_alt = float(self.declare_parameter('follow_altitude', 0.0).value)
        det_topic = self.declare_parameter(
            'detections_topic', f'/{self.drone_id}/detections').value
        self.mode = str(self.declare_parameter('follow_mode', 'image').value)

        # 'down'    -> ex/ey are ground offsets; translate, hold heading.
        # 'forward' -> ex is bearing, bbox area is range proxy; yaw + forward.
        self.mount = str(self.declare_parameter('camera_mount', 'down').value).lower()

        self.img_w = float(self.declare_parameter('image_width', 640).value)
        self.img_h = float(self.declare_parameter('image_height', 480).value)
        self.hfov = float(self.declare_parameter('hfov_deg', 80.0).value) * math.pi / 180.0
        # intrinsics for the down-cam ground projection (must match the detector's)
        self.fx = float(self.declare_parameter('fx', 554.0).value)
        self.fy = float(self.declare_parameter('fy', 554.0).value)

        self.yaw_deadband = float(self.declare_parameter('yaw_deadband', 0.06).value)
        self.max_yaw_step = float(self.declare_parameter('max_yaw_step', 0.10).value)
        self.yaw_gain = float(self.declare_parameter('yaw_gain', 0.5).value)

        self.strafe_gain = float(self.declare_parameter('strafe_gain', 3.0).value)
        self.max_strafe = float(self.declare_parameter('max_strafe', 2.0).value)
        self.strafe_sign = float(self.declare_parameter('strafe_sign', 1.0).value)

        # down-cam: metres of goto-offset per metre of measured ground error, and clamp
        self.pos_gain = float(self.declare_parameter('pos_gain', 0.6).value)
        self.max_pos_step = float(self.declare_parameter('max_pos_step', 2.5).value)
        self.pos_deadband = float(self.declare_parameter('pos_deadband', 0.4).value)  # m

        self.log_vec = bool(self.declare_parameter('log_vectors', True).value)

        self.desired_area = float(self.declare_parameter('desired_area', 0.05).value)
        self.fwd_gain = float(self.declare_parameter('forward_gain', 10.0).value)
        self.max_step = float(self.declare_parameter('max_step', 1.2).value)
        self.backoff_step = float(self.declare_parameter('backoff_step', 0.6).value)

        # Cruise speed, m/s, stamped onto every task this node emits. drone_controller ramps
        # its published setpoint toward the goal at this rate (step = speed / rate_hz), so it
        # is a real speed limit rather than a per-step distance like max_step. Previously left
        # unset, which silently fell back to the controller's own default; setting it here
        # makes follow speed explicit and tunable per drone.
        self.cruise_speed = float(self.declare_parameter('cruise_speed', 3.0).value)

        # Range readout for the operator console. Image-space servoing does not need a metric
        # distance - it closes on bbox area - so this is display-only and never feeds control.
        # Pinhole model: a target of real height h projects to bh pixels at distance h*fy/bh.
        # target_height_m is the assumed real height of the tracked object (1.7 = person).
        self.target_h = float(self.declare_parameter('target_height_m', 1.7).value)

        self.alt_gain = float(self.declare_parameter('alt_gain', 0.0).value)
        self.smooth_a = float(self.declare_parameter('smooth_alpha', 0.35).value)
        self.reassoc_px = float(self.declare_parameter('reassoc_px', 0.25).value)

        self.lead_time = float(self.declare_parameter('lead_time', 0.8).value)
        self.max_jump = float(self.declare_parameter('max_jump', 35.0).value)

        self.search = bool(self.declare_parameter('search', False).value)
        # How close to the commanded waypoint counts as "arrived", and therefore as free to
        # yaw-scan. Matches the coordinator's own arrival tolerance.
        self.scan_radius = float(self.declare_parameter('scan_radius', 2.0).value)
        self.search_yaw_rate = float(self.declare_parameter('search_yaw_rate', 0.3).value)
        self.contact_freeze = float(self.declare_parameter('contact_freeze', 2.0).value)

        # ---- BLIND-YAW BUDGET: total radians we are allowed to rotate open-loop on a stale
        # error before we give up and simply HOLD. Without this the node sweeps the target out
        # of its own field of view and can never recover. ----
        self.blind_yaw_max = float(self.declare_parameter('blind_yaw_max', 0.30).value)

        self._nav_target = None     # (north, east) the mission last sent us, world-local ENU
        self._nav_action = ''
        self.last_contact = -1e9
        self.locked_id = None
        self.last_seen = -1e9
        self.pose = None
        self.last_cx = None
        self.last_cy = None
        self.ex_f = None
        self.ey_f = None
        self.area_f = None
        self.dist_f = None      # smoothed range estimate, metres (display only)
        self.last_ex = None
        self.last_ey = None
        self._blind_yaw = 0.0
        self._disengaged_logged = False
        self._no_lock_logged = -1e9

        # candidate-id persistence counters for the lock
        self._cand_id = None
        self._cand_hits = 0
        self._cand_t0 = -1e9

        self.last_world = None
        self.last_time = None
        self.vel = (0.0, 0.0)

        k = self._inst()
        self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{k}/fmu/out/vehicle_local_position_v1',
            self._on_pose,
            _QOS,
        )
        self.create_subscription(Detection2DArray, det_topic, self._on_det, 10)
        # Mission traffic on our own task topic. Coordinator tasks carry a mission_id; the
        # ones this node publishes do not, which is how the two are told apart.
        self.create_subscription(DroneTask, f'/{self.drone_id}/task', self._on_nav_task, 10)
        self.create_subscription(
            String,
            f'/{self.drone_id}/follow_reset',
            lambda m: self._reset('operator reset'),
            10,
        )
        self.pub = self.create_publisher(DroneTask, f'/{self.drone_id}/task', 10)
        # Follow-state beacon for the ops console (Detect/Lock/Follow/Hold/Search/Disengage).
        # Published only on change; consumers (dashboard_bridge) treat LOCK/FOLLOW as locked.
        self.state_pub = self.create_publisher(String, f'/{self.drone_id}/follow_state', 10)
        # Estimated metres to the tracked target, for the console. Display-only.
        self.dist_pub = self.create_publisher(Float32, f'/{self.drone_id}/target_distance', 10)
        self._fstate = ''
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'target_follow up for {self.drone_id} '
            f'(mode={self.mode}, mount={self.mount}, lock={self.lock_frames} frames)'
        )

    def _inst(self):
        m = re.search(r'(\d+)$', str(self.drone_id))
        return int(m.group(1)) if m else 1

    def _on_pose(self, m):
        self.pose = (m.x, m.y, -m.z, m.heading)

    @staticmethod
    def _parse(d):
        hyp = d.results[0]
        cid = hyp.hypothesis.class_id
        tid = None
        if '#' in cid:
            try:
                tid = int(cid.split('#')[1])
            except ValueError:
                tid = None
        b = d.bbox
        world = (hyp.pose.pose.position.y, hyp.pose.pose.position.x)
        return (tid, hyp.hypothesis.score, b.center.position.x, b.center.position.y,
                b.size_x, b.size_y, world)

    def _on_det(self, msg):
        if not msg.detections or self.pose is None:
            return
        self.last_contact = self._now()

        cand = {}
        for d in msg.detections:
            if not d.results:
                continue
            tid, score, cx, cy, bw, bh, world = self._parse(d)
            if tid is None or tid < 0 or score < self.min_conf:
                continue
            cand[tid] = (score, cx, cy, bw, bh, world)

        if not cand:
            return

        target_id = self._select_target(cand)
        if target_id is None:
            return

        score, cx, cy, bw, bh, world = cand[target_id]
        self.last_seen = self._now()
        self.last_cx, self.last_cy = cx, cy
        self._blind_yaw = 0.0          # fresh measurement -> reopen the blind-yaw budget

        if self.mode == 'world':
            self._update_velocity(world, self.last_seen)
            self._pursue_world(world)
        elif self.mount == 'down':
            self._pursue_down(score, cx, cy)
        else:
            self._pursue_forward(score, cx, cy, bw, bh)

    # ---------------- target selection ----------------
    def _select_target(self, cand):
        now = self._now()

        if self.locked_id is not None:
            if self.locked_id in cand:
                return self.locked_id
            # id churned (detector re-seeded and minted a new id). Re-associate spatially.
            if self.last_cx is not None:
                gate = self.reassoc_px * self.img_w
                best, bd = None, gate
                for tid, c in cand.items():
                    d = math.hypot(c[1] - self.last_cx, c[2] - self.last_cy)
                    if d <= bd:
                        best, bd = tid, d
                if best is not None:
                    self.get_logger().info(
                        f'[{self.drone_id}] re-associated lock #{self.locked_id} -> #{best} '
                        f'({bd:.0f} px)')
                    self.locked_id = best
                    return best
            return None

        # ---- no lock yet: earn it with id persistence, NOT with a high score gate ----
        best_id = max(cand, key=lambda t: cand[t][0])
        if self._cand_id != best_id or (now - self._cand_t0) > self.lock_window:
            self._cand_id = best_id
            self._cand_hits = 1
            self._cand_t0 = now
        else:
            self._cand_hits += 1

        if self._cand_hits >= self.lock_frames:
            self.locked_id = best_id
            self._reset_servo()
            self._disengaged_logged = False
            self.get_logger().info(
                f'[{self.drone_id}] LOCKED target #{self.locked_id} '
                f'(conf={cand[best_id][0]:.2f}, {self._cand_hits} frames)')
            self._emit_state('LOCK')
            return self.locked_id

        if now - self._no_lock_logged > 2.0:
            self._no_lock_logged = now
            self.get_logger().info(
                f'[{self.drone_id}] contact #{best_id} conf={cand[best_id][0]:.2f} '
                f'({self._cand_hits}/{self.lock_frames} toward lock)')
        self._emit_state('DETECT')
        return None

    # ---------------- DOWNWARD-CAMERA servo: translate, do not yaw ----------------
    def _pursue_down(self, score, cx, cy):
        pn, pe, up, yaw = self.pose
        if up < 1.0:
            return

        # pixel offset -> ground offset in metres, using the same intrinsics as the detector
        gx = (cx - self.img_w / 2.0) / self.fx * up      # right of the airframe
        gy = (cy - self.img_h / 2.0) / self.fy * up      # image-down == airframe rear

        a = self.smooth_a
        self.ex_f = gx if self.ex_f is None else a * gx + (1 - a) * self.ex_f
        self.ey_f = gy if self.ey_f is None else a * gy + (1 - a) * self.ey_f

        err = math.hypot(self.ex_f, self.ey_f)
        if err < self.pos_deadband:
            right, fwd = 0.0, 0.0
        else:
            right = self.pos_gain * self.ex_f
            fwd = -self.pos_gain * self.ey_f              # image-down is behind the airframe
            s = math.hypot(right, fwd)
            if s > self.max_pos_step:
                right *= self.max_pos_step / s
                fwd *= self.max_pos_step / s

        target_up = self.follow_alt if self.follow_alt > 0.5 else up

        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        tn = pn + fwd * cy_ + right * (-sy_)
        te = pe + fwd * sy_ + right * (cy_)

        t = self._task('goto')
        t.target = Point(x=float(te), y=float(tn), z=float(target_up))
        t.target_altitude = float(target_up)
        t.yaw = float(yaw)                                # hold heading; do not spin
        self.pub.publish(t)

        self.last_ex = (cx - self.img_w / 2.0) / self.img_w
        self.last_ey = (cy - self.img_h / 2.0) / self.img_h

        if self.log_vec:
            self.get_logger().info(
                f'[{self.drone_id}] #{self.locked_id} ground=({self.ex_f:+.2f},'
                f'{self.ey_f:+.2f})m -> fwd={fwd:+.2f} right={right:+.2f} '
                f'alt={up:.1f} conf={score:.2f}',
                throttle_duration_sec=0.5)

    # ---------------- FORWARD-CAMERA servo (bearing + apparent-size range) ----------------
    def _pursue_forward(self, score, cx, cy, bw, bh):
        pn, pe, up, yaw = self.pose

        ex = (cx - self.img_w / 2.0) / self.img_w
        ey = (cy - self.img_h / 2.0) / self.img_h
        area = (bw * bh) / (self.img_w * self.img_h)

        a = self.smooth_a
        self.ex_f = ex if self.ex_f is None else a * ex + (1 - a) * self.ex_f
        self.ey_f = ey if self.ey_f is None else a * ey + (1 - a) * self.ey_f
        self.area_f = area if self.area_f is None else a * area + (1 - a) * self.area_f

        # Operator range readout (display only - control below still runs on area error).
        # Smoothed with the same alpha as the servo terms so the number on screen is as
        # steady as the box it comes from.
        if bh > 1.0:
            dist = self.target_h * self.fy / bh
            self.dist_f = dist if self.dist_f is None else a * dist + (1 - a) * self.dist_f
            self.dist_pub.publish(Float32(data=float(self.dist_f)))

        if abs(self.ex_f) < self.yaw_deadband:
            dyaw = 0.0
        else:
            dyaw = self.ex_f * self.hfov * self.yaw_gain
            dyaw *= max(0.4, 1.0 - 0.5 * abs(self.ex_f))
            dyaw = max(-self.max_yaw_step, min(self.max_yaw_step, dyaw))
        new_yaw = yaw + dyaw

        raw_fwd = self.fwd_gain * (self.desired_area - self.area_f)
        edge_scale = max(0.0, 1.0 - 2.0 * abs(self.ex_f))
        fwd = raw_fwd * edge_scale
        fwd = max(-self.backoff_step, min(self.max_step, fwd))

        if abs(self.ex_f) < self.yaw_deadband:
            lat = 0.0
        else:
            lat = self.strafe_sign * self.strafe_gain * self.ex_f
            lat = max(-self.max_strafe, min(self.max_strafe, lat))

        target_up = up - self.alt_gain * self.ey_f
        if self.follow_alt > 0.5:
            target_up = self.follow_alt

        cy_, sy_ = math.cos(new_yaw), math.sin(new_yaw)
        tn = pn + fwd * cy_ + lat * (-sy_)
        te = pe + fwd * sy_ + lat * (cy_)

        t = self._task('goto')
        t.target = Point(x=float(te), y=float(tn), z=float(target_up))
        t.target_altitude = float(target_up)
        t.yaw = float(new_yaw)
        self.pub.publish(t)

        self.last_ex = self.ex_f
        self.last_ey = self.ey_f

        if self.log_vec:
            self.get_logger().info(
                f'[{self.drone_id}] #{self.locked_id} ex={self.ex_f:+.3f} '
                f'area={self.area_f:.4f} -> dyaw={dyaw:+.3f} lat={lat:+.2f} fwd={fwd:+.2f} '
                f'conf={score:.2f}', throttle_duration_sec=0.5)

    # ---------------- world mode ----------------
    def _update_velocity(self, world, now):
        if world == (0.0, 0.0):
            return
        if self.last_world is not None and self.last_time is not None:
            dt = now - self.last_time
            if 0.05 < dt < 1.0:
                vn = (world[0] - self.last_world[0]) / dt
                ve = (world[1] - self.last_world[1]) / dt
                a = 0.5
                self.vel = (a * vn + (1 - a) * self.vel[0],
                            a * ve + (1 - a) * self.vel[1])
        self.last_world = world
        self.last_time = now

    def _pursue_world(self, world):
        n, e = world
        if e == 0.0 and n == 0.0:
            return
        if self.pose is not None and \
                math.hypot(n - self.pose[0], e - self.pose[1]) > self.max_jump:
            return

        n_lead = n + self.vel[0] * self.lead_time
        e_lead = e + self.vel[1] * self.lead_time
        up = self.follow_alt if self.follow_alt > 0.5 else (self.pose[2] if self.pose else 4.0)

        t = self._task('goto')
        t.target = Point(x=float(e_lead), y=float(n_lead), z=float(up))
        t.target_altitude = float(up)
        t.yaw = float(self.pose[3]) if self.pose else 0.0   # was NaN
        self.pub.publish(t)

    # ---------------- tick ----------------
    def _tick(self):
        if self.locked_id is None:
            self._search()
            return

        if self.last_seen <= 0 or self.pose is None:
            return

        gap = self._now() - self.last_seen
        if gap > self.lost_timeout:
            self._reset('target lost')
            return

        if gap <= 0.4:
            return

        pn, pe, up, yaw = self.pose

        # ---- COAST: hold station on the last commanded point. Under a down-cam we never
        # yaw blind at all — rotating the airframe cannot recover a translational error and
        # only smears the search pattern.
        if self.mount == 'down' or self.last_ex is None or \
                abs(self.last_ex) <= self.yaw_deadband or \
                self._blind_yaw >= self.blind_yaw_max:
            t = self._task('goto')
            t.target = Point(x=float(pe), y=float(pn), z=float(up))
            t.target_altitude = float(up)
            t.yaw = float(yaw)
            self.pub.publish(t)
            return

        # ---- BUDGETED, DECAYED blind yaw (forward mount only). Confidence in a stale error
        # falls off with the gap; total open-loop rotation is capped by blind_yaw_max.
        decay = max(0.0, 1.0 - gap / self.lost_timeout)
        dyaw = self.last_ex * self.hfov * self.yaw_gain * decay
        dyaw = max(-self.max_yaw_step, min(self.max_yaw_step, dyaw))
        self._blind_yaw += abs(dyaw)

        t = self._task('goto')
        t.target = Point(x=float(pe), y=float(pn), z=float(up))
        t.target_altitude = float(up)
        t.yaw = float(yaw + dyaw)
        self.pub.publish(t)
        self._emit_state('FOLLOW')

    def _station_keeping(self):
        """True when the mission is NOT currently flying this drone somewhere.

        Yaw-scanning is only safe while the drone is parked. If it scanned mid-leg it would
        fight the mission for the same drone: every scan message re-commands the current
        position, so the vehicle would sit and spin instead of covering its search area.
        Holding, or having arrived at the commanded waypoint, means the area sweep is done
        moving and the camera can sweep instead.
        """
        if self._nav_action in ('hold', 'wait'):
            return True
        if self._nav_action == 'goto' and self._nav_target is not None:
            pn, pe, _, _ = self.pose
            tn, te = self._nav_target
            return math.hypot(pn - tn, pe - te) <= self.scan_radius
        # takeoff / rtl / land, or nothing commanded yet: the mission owns the drone. Scanning
        # here republishes the CURRENT altitude every tick, which - being the newest task at
        # equal priority - overrides the climb and pins the vehicle to the ground.
        return False

    def _search(self):
        if not self.search or self.pose is None:
            return
        if self._now() - self.last_contact < self.contact_freeze:
            return
        if not self._station_keeping():
            return                              # the mission is flying it; let the sweep run
        self._emit_state('SEARCH')
        pn, pe, up, yaw = self.pose
        t = self._task('goto')
        t.target = Point(x=float(pe), y=float(pn), z=float(up))
        t.target_altitude = float(up)
        t.yaw = float(yaw + self.search_yaw_rate * 0.2)
        # NORMAL, not NAV: a task above NORMAL arms a 3 s override in drone_controller that
        # blocks every lower-priority task, and this republishes every tick - which would
        # refresh that override forever and permanently lock the mission out of its own drone.
        t.priority = DroneTask.PRIORITY_NORMAL
        self.pub.publish(t)

    def _on_nav_task(self, m):
        """Track what the MISSION last told this drone to do (our own tasks carry no id)."""
        if not m.mission_id:
            return
        self._nav_action = m.action
        self._nav_target = ((m.target.y, m.target.x)
                            if m.action == 'goto' else None)

    def _emit_state(self, s):
        if s != self._fstate:
            self._fstate = s
            m = String()
            m.data = s
            self.state_pub.publish(m)

    def _reset_servo(self):
        self.last_cx = None
        self.last_cy = None
        self.ex_f = None
        self.ey_f = None
        self.area_f = None
        self.dist_f = None      # smoothed range estimate, metres (display only)
        self.last_ex = None
        self.last_ey = None
        self._blind_yaw = 0.0
        self.last_world = None
        self.last_time = None
        self.vel = (0.0, 0.0)

    def _reset(self, why):
        if self.locked_id is not None and not self._disengaged_logged:
            self.get_logger().info(
                f'[{self.drone_id}] DISENGAGE (#{self.locked_id}): {why} -> yielding to nav')
            self._disengaged_logged = True
        self._emit_state('DISENGAGE')
        self.locked_id = None
        self.last_seen = -1e9
        self._cand_id = None
        self._cand_hits = 0
        self._reset_servo()

    def _task(self, action):
        t = DroneTask()
        t.header.stamp = self.get_clock().now().to_msg()
        t.drone_id = self.drone_id
        t.action = action
        t.priority = DroneTask.PRIORITY_FOLLOW
        t.cruise_speed = self.cruise_speed
        t.yaw = float('nan')
        return t

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()