#!/usr/bin/env python3
"""Local planner — autonomous obstacle-aware navigation for one drone.

Turns "go to this point" into "find a collision-free path there on the live map and
follow it". It plans A* over the shared OccupancyGrid (built by occupancy_mapper),
inflated by the drone's radius, and steers the drone with ordinary `goto` tasks —
so it reuses the existing control path and never fights the mission/agentic layer.

Only active while a goal is set on `/px4_N/nav_goal`; idle otherwise (zero cost,
zero interference). Re-plans at 1 Hz so it reacts as the map fills in — true
navigation of unknown / partially-known space, not fixed waypoints.

Frames: DroneStatus.position and the goal are local ENU (x=East, y=North); we lift
to the world with drone_origins to index the world grid, and lower back to local
ENU for the goto. All grid metadata (resolution/size/origin) comes from the map
message, so this node has no hard-coded map assumptions.
"""

import heapq
import math
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

from nav_msgs.msg import OccupancyGrid
from swarm_msgs.msg import DroneStatus, DroneTask

OCC = 100


def astar(blocked, start, goal, w=1.0):
    """4/8-connected A* on a boolean `blocked` grid. Returns [(r,c), ...] or None.

    blocked[r][c] True = impassable. Cost = Euclidean step, heuristic = Euclidean.
    `w` weights the heuristic (>1 = greedier / much faster on large mostly-open grids,
    paths slightly suboptimal but still obstacle-free) — needed for long-range planning.
    """
    if blocked[goal] or blocked[start]:
        return None
    h, w_ = blocked.shape
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    openq = [(0.0, start)]
    came, g = {}, {start: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        cr, cc = cur
        for dr, dc, step in nbrs:
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < h and 0 <= nc < w_ and not blocked[nr, nc]:
                ng = g[cur] + step
                if ng < g.get((nr, nc), 1e18):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    f = ng + w * math.hypot(nr - goal[0], nc - goal[1])
                    heapq.heappush(openq, (f, (nr, nc)))
    return None


class LocalPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')

        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.reach = float(self.declare_parameter('waypoint_tol', 1.5).value)  # m to advance
        # Obstacle inflation and carrot look-ahead are in METRES, converted to cells
        # against the live map resolution — so the same planner works on the coarse
        # 0.5 m lidar grid (/swarm/map) and the fine ~0.05 m rtabmap SLAM grid alike.
        self.inflate_m = float(self.declare_parameter('inflate_radius_m', 1.5).value)
        self.lookahead_m = float(self.declare_parameter('lookahead_m', 20.0).value)
        # Greedy A* heuristic weight: keeps long-range planning fast on the large,
        # mostly-open avoidance grid (paths stay obstacle-free, just slightly non-optimal).
        self.heuristic_w = float(self.declare_parameter('heuristic_weight', 2.0).value)
        # Map source. Default = the shared lidar world grid. With SLAM nav we point this
        # at the drone's own rtabmap grid (/{drone}/map) and lift it into the shared world
        # frame via drone_origins (map_frame_offset), so A* plans on the SLAM map.
        self.map_topic = self.declare_parameter('map_topic', '/swarm/map').value
        self.map_frame_offset = bool(self.declare_parameter('map_frame_offset', False).value)

        o = {}
        for s in (self.declare_parameter(
                'drone_origins', [], ParameterDescriptor(dynamic_typing=True)).value or []):
            p = s.split(':')
            if len(p) == 3:
                o[p[0]] = (float(p[1]), float(p[2]))
        self.o_n, self.o_e = o.get(self.drone_id, (0.0, 0.0))

        self.blocked = None          # inflated obstacle mask (numpy bool)
        self.info = None             # OccupancyGrid.info (resolution/size/origin)
        self.world = None            # own (north, east)
        self.goal = None             # (north, east, alt) world, or None when idle

        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 1)
        self.create_subscription(DroneStatus, f'/{self.drone_id}/status', self._on_pose, 10)
        self.create_subscription(Point, f'/{self.drone_id}/nav_goal', self._on_goal, 10)
        self.pub = self.create_publisher(DroneTask, f'/{self.drone_id}/task', 10)

        self.create_timer(1.0, self._step)      # re-plan at 1 Hz
        self.get_logger().info(f'local_planner up for {self.drone_id}')

    # ---------- inputs ----------
    def _on_map(self, m):
        info = m.info
        if self.map_frame_offset:
            # SLAM grid arrives in the drone's own map frame (origin at its spawn); lift it
            # into the shared world frame so poses/goals index the same cells the planner
            # already reasons in. px4_1's origin is (0,0), so this is a no-op for it.
            info.origin.position.x += self.o_e
            info.origin.position.y += self.o_n
        self.info = info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        occ = grid == OCC
        n_inflate = max(1, int(round(self.inflate_m / info.resolution)))
        blocked = occ.copy()                      # inflate obstacles by drone radius
        for _ in range(n_inflate):
            b = blocked.copy()
            b[1:, :] |= blocked[:-1, :]; b[:-1, :] |= blocked[1:, :]
            b[:, 1:] |= blocked[:, :-1]; b[:, :-1] |= blocked[:, 1:]
            blocked = b
        self.blocked = blocked

    def _on_pose(self, m):
        self.world = (m.position.y + self.o_n, m.position.x + self.o_e)

    def _on_goal(self, m):
        # goal arrives in LOCAL ENU (x=East, y=North); lift to the world frame for planning
        self.goal = (m.y + self.o_n, m.x + self.o_e, m.z)
        self.get_logger().info(f'[{self.drone_id}] nav_goal set (world n={self.goal[0]:.1f} '
                               f'e={self.goal[1]:.1f})')

    # ---------- world <-> grid ----------
    def _cell(self, north, east):
        r = int((north - self.info.origin.position.y) / self.info.resolution)
        c = int((east - self.info.origin.position.x) / self.info.resolution)
        if 0 <= r < self.info.height and 0 <= c < self.info.width:
            return r, c
        return None

    def _world(self, cell):
        r, c = cell
        north = self.info.origin.position.y + (r + 0.5) * self.info.resolution
        east = self.info.origin.position.x + (c + 0.5) * self.info.resolution
        return north, east

    def _edge_cell_toward(self, gn, ge):
        """Farthest FREE in-grid cell along the ray from the drone toward the world goal
        (gn, ge). Lets the planner make progress toward a goal beyond the current map:
        it heads to the map edge in the goal's direction, avoiding mapped obstacles, and
        the map fills in as it flies. Returns None if the edge toward the goal is blocked."""
        dn, de = gn - self.world[0], ge - self.world[1]
        dist = math.hypot(dn, de)
        if dist < 1e-6:
            return None
        step = self.info.resolution
        # stay inside the grid (49% of half-extent) and walk inward until a free cell
        reach = min(dist, 0.49 * self.info.width * step, 0.49 * self.info.height * step)
        d = reach
        while d > step:
            wn = self.world[0] + dn / dist * d
            we = self.world[1] + de / dist * d
            cell = self._cell(wn, we)
            if cell is not None and not self.blocked[cell]:
                return cell
            d -= step
        return None

    # ---------- plan + follow (1 Hz) ----------
    def _step(self):
        if self.goal is None or self.blocked is None or self.world is None:
            return
        gn, ge, alt = self.goal
        if math.hypot(gn - self.world[0], ge - self.world[1]) <= self.reach:
            self.get_logger().info(f'[{self.drone_id}] nav_goal reached')
            self.goal = None
            return
        start = self._cell(*self.world)
        if start is None:
            return
        # The drone's own footprint is by definition traversable — clear a small disk
        # around it so a spurious occupancy reading under/around the drone can never
        # freeze A* (blocked[start]). Robustness against residual false-positives.
        sr, sc = start
        rad = max(1, int(round(self.inflate_m / self.info.resolution)))
        self.blocked[max(0, sr - rad):sr + rad + 1, max(0, sc - rad):sc + rad + 1] = False
        goal = self._cell(gn, ge)
        if goal is None:
            # Goal lies OUTSIDE the mapped grid (long-range / unexplored). Aim at the
            # farthest free cell along the ray toward it, so the drone flies TOWARD the
            # goal (avoiding mapped obstacles) and the map grows to eventually contain it.
            goal = self._edge_cell_toward(gn, ge)
            if goal is None:
                return
        path = astar(self.blocked, start, goal, w=self.heuristic_w)
        if not path:
            self.get_logger().warn(f'[{self.drone_id}] no path to goal (blocked?)',
                                   throttle_duration_sec=5.0)
            return
        # steer toward a lookahead point on the path (carrot pursuit); look-ahead in
        # metres -> cells against this map's resolution
        lookahead = max(1, int(round(self.lookahead_m / self.info.resolution)))
        wp = path[min(lookahead, len(path) - 1)]
        wn, we = self._world(wp)
        self._send_goto(we - self.o_e, wn - self.o_n, alt)   # world -> local ENU

    def _send_goto(self, east, north, alt):
        t = DroneTask()
        t.header.stamp = self.get_clock().now().to_msg()
        t.drone_id = self.drone_id
        t.action = 'goto'
        t.target = Point(x=float(east), y=float(north), z=float(alt))
        t.target_altitude = float(alt)
        t.yaw = float('nan')
        # NAV owns the drone while a goal is active (beats mission/agentic NORMAL via the
        # controller's 3 s override TTL); it yields automatically once the goal is reached.
        t.priority = DroneTask.PRIORITY_NAV
        self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
