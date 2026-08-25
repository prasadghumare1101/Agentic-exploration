#!/usr/bin/env python3
"""Occupancy mapper — online 2-D world map from the swarm's lidars.

ONE shared node. It fuses every drone's 360-degree LaserScan with that drone's
EKF pose (position + heading) into a single occupancy grid published on
`/swarm/map`. This is the "online mapping + localization" half of autonomous
navigation: the local_planner (C2) plans obstacle-free paths on this grid.

Frames
------
World grid is ENU (x=East, y=North), centred on the shared world origin. A drone's
pose arrives in its own local NED frame (x=North, y=East, heading=yaw CW from
North); we lift it to the world with its drone_origins offset, then ray-cast each
lidar return to a world cell.

Design choices (kept deliberately small + fast)
-----------------------------------------------
* numpy int8 grid, marked in-place -> O(rays) per scan, O(W*H) memory, no per-cell
  Python loops on the hot path.
* obstacles-only map (mark hits occupied); we do NOT ray-trace free space, which
  halves the work and is all the planner needs. Easy to extend if you want it.
* one node, plain callbacks -> no locks, no cross-process shared state.
"""

import math
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition

# Gazebo sensors and the PX4 DDS bridge both publish BEST_EFFORT; a default (reliable)
# subscription silently receives nothing from them. Match it.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

FREE, OCC, UNKNOWN = 0, 100, -1


class OccupancyMapper(Node):

    def __init__(self):
        super().__init__('occupancy_mapper')

        self.known_drones = list(self.declare_parameter('known_drones', ['px4_1']).value)
        self.res = float(self.declare_parameter('resolution', 0.5).value)      # m/cell
        self.size = int(self.declare_parameter('grid_cells', 200).value)       # W=H cells
        pub_hz = float(self.declare_parameter('publish_rate', 2.0).value)
        # A drone's lidar sees its team-mates; mapping them as obstacles would block the
        # swarm's own navigation. Discard any hit within this radius of a known drone.
        self.team_radius = float(self.declare_parameter('teammate_radius', 2.0).value)
        # A grounded drone's horizontal lidar grazes the ground plane and paints a
        # spurious ring of "obstacles"; only map from drones that are actually airborne.
        self.min_map_alt = float(self.declare_parameter('min_map_alt', 1.5).value)
        # When a multirotor pitches to fly, its body-fixed horizontal lidar tilts down and
        # hits the ground ~15-30 m ahead. Cap the mapping range below that so those phantom
        # hits are excluded while genuine near obstacles are still mapped.
        self.max_range = float(self.declare_parameter('max_map_range', 12.0).value)

        # Shared-frame origins ['px4_1:0:0', ...] (id:north:east) — same convention as
        # the coordinator, so the map lines up with formation/mission coordinates.
        self.origins = {}
        for s in (self.declare_parameter(
                'drone_origins', [], ParameterDescriptor(dynamic_typing=True)).value or []):
            p = s.split(':')
            if len(p) == 3:
                self.origins[p[0]] = (float(p[1]), float(p[2]))

        # int8 grid, row-major [north][east]; world (0,0) sits at the grid centre.
        self.grid = np.full((self.size, self.size), UNKNOWN, dtype=np.int8)
        self.half = self.size * self.res / 2.0          # metres from centre to edge
        self.pose = {}                                  # drone -> (north, east, yaw) world

        for d in self.known_drones:
            k = self._instance(d)
            self.create_subscription(VehicleLocalPosition,
                                     f'/px4_{k}/fmu/out/vehicle_local_position_v1',
                                     self._pose_cb(d), SENSOR_QOS)
            self.create_subscription(LaserScan, f'/iris_{k}/scan',
                                     self._scan_cb(d), SENSOR_QOS)

        self.pub = self.create_publisher(OccupancyGrid, '/swarm/map', 1)
        self.create_timer(1.0 / pub_hz, self._publish)
        self.get_logger().info(
            f'occupancy_mapper up: {self.size}x{self.size} @ {self.res} m/cell')

    # ---------- helpers ----------
    @staticmethod
    def _instance(drone_id):
        """'px4_2' -> 2. Use only the TRAILING number, or 'px4_1' would give 41
        (the '4' in 'px4') and subscribe to a non-existent /iris_41."""
        m = re.search(r'(\d+)$', drone_id)
        return int(m.group(1)) if m else 1

    def _cell(self, north, east):
        """World (north, east) metres -> (row, col) or None if off-grid."""
        col = int((east + self.half) / self.res)
        row = int((north + self.half) / self.res)
        if 0 <= row < self.size and 0 <= col < self.size:
            return row, col
        return None

    # ---------- inputs ----------
    def _pose_cb(self, drone):
        o_n, o_e = self.origins.get(drone, (0.0, 0.0))
        def cb(m):
            # Only trust a VALID EKF fix. During init the estimate can jump hundreds of
            # metres; mapping lidar hits against that glitched pose paints permanent phantom
            # obstacles far from the drone. Skip until xy is valid.
            if not m.xy_valid:
                return
            # VehicleLocalPosition is NED (x=North, y=East, z=Down); lift to world + altitude.
            self.pose[drone] = (m.x + o_n, m.y + o_e, m.heading, -m.z)
        return cb

    def _scan_cb(self, drone):
        def cb(scan):
            p = self.pose.get(drone)
            if p is None or p[3] < self.min_map_alt:      # skip grounded drones (ground ring)
                return
            pn, pe, yaw = p[0], p[1], p[2]
            teammates = [(tn, te) for d2, (tn, te, _, _) in self.pose.items() if d2 != drone]
            rmax = min(self.max_range, 0.95 * scan.range_max)   # cap range (drop far ground hits)
            ang = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < rmax and math.isfinite(r):
                    # ray bearing in world = drone heading + ray angle (both CW from North)
                    b = yaw + ang
                    hn, he = pn + r * math.cos(b), pe + r * math.sin(b)
                    # drop hits on our own team-mates so the swarm isn't self-blocking
                    if not any(math.hypot(hn - tn, he - te) < self.team_radius
                               for tn, te in teammates):
                        cell = self._cell(hn, he)
                        if cell is not None:
                            self.grid[cell] = OCC
                ang += scan.angle_increment
        return cb

    # ---------- output ----------
    def _clear_teammates(self):
        """Erase the swarm's own footprint every cycle: cells within team_radius of any
        drone are set FREE. Handles both the startup race (marks made before poses were
        known) and moving drones (they leave a clear trail, not an obstacle wall)."""
        rad = int(self.team_radius / self.res)
        for pn, pe, _, _ in self.pose.values():
            c = self._cell(pn, pe)
            if c is None:
                continue
            r0, c0 = c
            self.grid[max(0, r0 - rad):r0 + rad + 1,
                      max(0, c0 - rad):c0 + rad + 1] = FREE

    def _decay_far(self):
        """Keep only obstacles the swarm can CURRENTLY sense: clear (to UNKNOWN) any OCC
        cell beyond lidar range of every drone. The avoidance map is intentionally LOCAL/
        rolling — real obstacles are re-mapped as drones approach them, while stale marks
        (pose-glitch phantoms, or areas already flown past) can never persist far away and
        block long-range planning. This is what makes long-range avoidance robust."""
        if not self.pose:
            return
        occ = np.argwhere(self.grid == OCC)
        if len(occ) == 0:
            return
        wn = -self.half + (occ[:, 0] + 0.5) * self.res      # world N of each occ cell
        we = -self.half + (occ[:, 1] + 0.5) * self.res      # world E
        keep2 = (self.max_range + 3.0) ** 2
        near = np.zeros(len(occ), dtype=bool)
        for pn, pe, _, _ in self.pose.values():
            near |= (wn - pn) ** 2 + (we - pe) ** 2 <= keep2
        far = occ[~near]
        self.grid[far[:, 0], far[:, 1]] = UNKNOWN

    def _publish(self):
        self._clear_teammates()
        self._decay_far()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.res
        msg.info.width = self.size
        msg.info.height = self.size
        msg.info.origin.position.x = -self.half   # ENU: x=East
        msg.info.origin.position.y = -self.half   #      y=North
        msg.data = self.grid.flatten().tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
