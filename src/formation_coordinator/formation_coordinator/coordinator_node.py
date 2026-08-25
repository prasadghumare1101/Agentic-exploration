#!/usr/bin/env python3
"""Mission plan executor.

Receives a validated MissionPlan and runs it phase by phase: expands each phase's
primitives (including formation slot geometry) into per-drone DroneTasks, watches
the drones' reported positions, and advances when the phase completes (all
arrived) or its timeout expires.

It does NOT stream setpoints and does not know PX4 exists — it decides "which
drone does what, and when the next step starts". drone_controller does the
flying; safety_manager can override at any time with PRIORITY_SAFETY.

Frames: DroneTask.target and DroneStatus.position are ENU in each drone's LOCAL
frame. Formation anchors are given in the SHARED world frame, so slots are
computed in world NED then offset into each drone's local frame via drone_origins.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

from swarm_msgs.msg import MissionPlan, DroneTask, DroneStatus, FormationGoal, Heartbeat
from formation_coordinator import formations

# primitive -> DroneTask.action understood by drone_controller
_ACTION = {'takeoff': 'takeoff', 'goto': 'goto', 'formation': 'goto',
           'hold': 'hold', 'wait': 'hold', 'land': 'land', 'rtl': 'rtl'}


class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator_node')

        self.known_drones = list(self.declare_parameter('known_drones', ['px4_1']).value)
        self.tolerance = float(self.declare_parameter('arrival_tolerance', 1.5).value)
        self.default_alt = float(self.declare_parameter('default_altitude', 3.0).value)
        # Safety's separation floor, mirrored here so deconfliction can be tied to it
        # (see altitude_layer below). Keep in sync with safety_manager.min_separation.
        self.min_separation = float(self.declare_parameter('min_separation', 2.0).value)
        # Altitude-layer deconfliction: each drone flies in its own altitude band
        # (index * this offset). Separation is measured in 3D, so distinct bands mean
        # two drones can never breach min_separation even when their horizontal paths
        # cross mid-move -- which is what otherwise makes the safety guard fight a
        # formation and stall it. Default 2.5 m (>= min_separation) guarantees that.
        # Set 0.0 for a level formation (horizontal spacing + avoidance only).
        self.altitude_layer = float(self.declare_parameter('altitude_layer', 2.5).value)
        if 0.0 < self.altitude_layer < self.min_separation:
            # A positive band smaller than the separation floor would NOT guarantee
            # 3D deconfliction; raise it so the guarantee always holds.
            self.get_logger().warn(
                f'altitude_layer {self.altitude_layer} < min_separation '
                f'{self.min_separation}; raising to {self.min_separation} to '
                f'guarantee 3D deconfliction')
            self.altitude_layer = self.min_separation

        # Agentic mode: for FORMATION phases, publish a squad-level FormationGoal and
        # let the drone_agents self-assign slots (distributed auction) instead of the
        # coordinator dictating each slot. The coordinator still sequences phases and
        # gates advance on slot COVERAGE (every slot filled by SOME drone), which also
        # makes the phase tolerant of the agents redistributing after a drop. Every
        # other primitive (takeoff/goto/hold/rtl) stays centrally assigned. Off by
        # default -> unchanged, fully central behaviour.
        self.agentic = bool(self.declare_parameter('agentic', False).value)
        self.slot_cover = None   # list[(north,east)] world slots to cover, or None
        # Liveness for agentic self-heal: size a formation to the drones that are
        # actually alive, so a dropped drone's slot is not endlessly waited on.
        self.hb_timeout = float(self.declare_parameter('heartbeat_timeout', 3.0).value)
        self.last_hb = {}        # drone_id -> t last heartbeat

        # Shared-frame origin offsets: ['px4_1:0:0', 'px4_2:3:0', ...] (id:north:east).
        # dynamic_typing: an empty-list default would otherwise be inferred BYTE_ARRAY.
        self.drone_origins = {}
        for s in (self.declare_parameter(
                'drone_origins', [], ParameterDescriptor(dynamic_typing=True)).value or []):
            p = s.split(':')
            if len(p) == 3:
                self.drone_origins[p[0]] = (float(p[1]), float(p[2]))

        # Eager publishers so the first task of a plan is never lost to discovery.
        self.task_pubs = {d: self.create_publisher(DroneTask, f'/{d}/task', 10)
                          for d in self.known_drones}
        self.goal_pub = self.create_publisher(FormationGoal, '/swarm/formation_goal', 10)
        self.navgoal_pubs = {}   # drone -> Point publisher on /{d}/nav_goal (mission->planner)
        self.positions = {}   # drone -> (x, y, z) ENU local, from DroneStatus
        for d in self.known_drones:
            self.create_subscription(
                DroneStatus, f'/{d}/status',
                (lambda dd: (lambda m: self.positions.__setitem__(
                    dd, (m.position.x, m.position.y, m.position.z))))(d), 10)

        self.create_subscription(MissionPlan, '/mission_plan', self._on_plan, 10)
        self.create_subscription(
            Heartbeat, '/swarm/heartbeat',
            lambda m: self.last_hb.__setitem__(m.drone_id, self._now()), 10)

        # execution state
        self.mission_id = ''
        self.phases = []
        self.phase_idx = -1
        self.phase_started = 0.0
        self.arrival = {}     # drone -> (x|None, y|None, z|None) local ENU
        self.arrived = set()  # drones that have reached this phase's target (latched)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'coordinator up; known drones: {self.known_drones}')

    # ---------- plan intake ----------
    def _on_plan(self, msg: MissionPlan):
        try:
            plan = json.loads(msg.plan_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'bad plan_json: {e}')
            return
        self.mission_id = msg.mission_id
        self.phases = self._expand_patterns(plan.get('phases', []))
        self.get_logger().info(
            f'plan {self.mission_id}: {len(self.phases)} phase(s) — starting')
        self.phase_idx = -1
        self._next_phase()

    # ---------- declarative pattern expansion ----------
    def _expand_patterns(self, phases):
        """Replace declarative `survey` phases with concrete formation legs.

        The planner states INTENT (area, spacing) and the geometry is computed
        HERE, deterministically: the same intent always yields identical
        waypoints, which a model hand-computing trigonometry cannot guarantee —
        and it guarantees the area is actually covered.
        """
        out = []
        for ph in phases:
            if not isinstance(ph, dict):
                out.append(ph)
                continue
            prim = ph.get('primitive')
            if prim in ('orbit', 'spiral'):
                out.extend(self._expand_ring(ph, prim))
                continue
            if prim != 'survey':
                out.append(ph)
                continue
            p = ph.get('params') or {}
            cx, cy = float(p['center'][0]), float(p['center'][1])
            w, h, sp = float(p['width']), float(p['height']), float(p['spacing'])
            alt = float(p['altitude'])
            shape = p.get('shape', 'wedge')
            fspacing = float(p.get('formation_spacing', 8.0))
            th = math.radians(float(p.get('heading', 0.0)))
            cos_t, sin_t = math.cos(th), math.sin(th)

            legs = []
            lanes = max(1, int(h / sp) + 1)
            for i in range(lanes):
                oy = -h / 2.0 + min(i * sp, h)
                ends = [(-w / 2.0, oy), (w / 2.0, oy)]
                if i % 2:                       # alternate direction => lawnmower
                    ends.reverse()
                for ex, ey in ends:
                    legs.append((round(cx + ex * cos_t - ey * sin_t, 3),
                                 round(cy + ex * sin_t + ey * cos_t, 3)))

            base = ph.get('name', 'survey')
            for i, (gx, gy) in enumerate(legs):
                # face the direction of travel where there is a next leg
                if i + 1 < len(legs):
                    hdg = math.degrees(math.atan2(legs[i + 1][0] - gx, legs[i + 1][1] - gy))
                else:
                    hdg = float(p.get('heading', 0.0))
                out.append({
                    'name': f'{base}_leg{i}',
                    'primitive': 'formation',
                    'params': {'shape': shape, 'spacing': fspacing,
                               'heading': hdg, 'anchor': [gx, gy, alt]},
                    'drones': ph.get('drones'),
                    'advance_when': ph.get('advance_when', 'all_arrived'),
                    'timeout': ph.get('timeout', 60.0),
                })
            self.get_logger().info(
                f'survey "{base}" {w}x{h} m spacing {sp} -> {len(legs)} legs in {shape}')
        return out

    def _expand_ring(self, ph, kind):
        """ORBIT (fixed radius) / SPIRAL (radius grows per turn) -> formation legs.

        Declarative like `survey`: the planner gives centre/radius/turns and the
        ring geometry is computed here, deterministically.
        """
        p = ph.get('params') or {}
        cx, cy = float(p['center'][0]), float(p['center'][1])
        alt = float(p['altitude'])
        turns = int(p.get('turns', 1))
        n = int(p.get('points_per_turn', 12))
        shape = p.get('shape', 'circle')
        fspacing = float(p.get('formation_spacing', 8.0))
        r0 = float(p.get('radius', p.get('start_radius', 10.0)))
        growth = float(p.get('growth', 0.0)) if kind == 'spiral' else 0.0

        legs, base = [], ph.get('name', kind)
        for t in range(turns):
            for k in range(n):
                frac = k / n
                r = r0 + growth * (t + frac)
                a = 2.0 * math.pi * frac
                legs.append((round(cx + r * math.cos(a), 3),
                             round(cy + r * math.sin(a), 3)))
        if kind == 'orbit':                     # close the ring
            legs.append((round(cx + r0, 3), round(cy, 3)))

        out = []
        for i, (gx, gy) in enumerate(legs):
            if i + 1 < len(legs):
                hdg = math.degrees(math.atan2(legs[i + 1][0] - gx, legs[i + 1][1] - gy))
            else:
                hdg = 0.0
            out.append({
                'name': f'{base}_p{i}', 'primitive': 'formation',
                'params': {'shape': shape, 'spacing': fspacing,
                           'heading': hdg, 'anchor': [gx, gy, alt],
                           'speed': p.get('speed')},
                'drones': ph.get('drones'),
                'advance_when': ph.get('advance_when', 'all_arrived'),
                'timeout': ph.get('timeout', 30.0),
            })
        self.get_logger().info(
            f'{kind} "{base}" r={r0} turns={turns} -> {len(legs)} legs in {shape}')
        return out

    # ---------- phase sequencing ----------
    def _next_phase(self):
        self.phase_idx += 1
        if self.phase_idx >= len(self.phases):
            self.get_logger().info(f'plan {self.mission_id} COMPLETE')
            self.phases = []
            self.phase_idx = -1
            return
        ph = self.phases[self.phase_idx]
        self.phase_started = self._now()
        self.arrival = {}
        self.arrived = set()
        self.slot_cover = None
        name = ph.get('name', f'phase{self.phase_idx}')
        self.get_logger().info(f'-> phase {self.phase_idx} "{name}"')

        # Agentic FORMATION: broadcast the goal, let drone_agents self-assign, and
        # gate on slot coverage instead of per-drone identity.
        if self.agentic and ph.get('primitive') == 'formation':
            self._publish_formation_goal(ph)
            return

        # navigate: hand the target to each drone's local_planner (/{d}/nav_goal) instead
        # of a direct goto, so the drone plans an obstacle-aware path. Requires nav:=true.
        if ph.get('primitive') == 'navigate':
            self._publish_nav_goals(ph)
            return

        # Per-drone NAVIGATE assignments: each agent routes through its OWN local_planner
        # (obstacle-aware) to its own target, so an independent-sector mission has every
        # drone planning around obstacles on its map. Needs nav:=true. Same arrival gate.
        nav_drones = set()
        for a in (ph.get('assignments') or []):
            if a.get('primitive') == 'navigate' and a.get('drone'):
                d = a['drone']
                t = (a.get('params') or {}).get('target', [0.0, 0.0, self.default_alt])
                tx, ty, tz = float(t[0]), float(t[1]), float(t[2])
                pub = self.navgoal_pubs.get(d)
                if pub is None:
                    pub = self.create_publisher(Point, f'/{d}/nav_goal', 10)
                    self.navgoal_pubs[d] = pub
                pub.publish(Point(x=tx, y=ty, z=tz))
                self.arrival[d] = (tx, ty, tz)
                nav_drones.add(d)

        for drone, (task, target) in self._expand(ph).items():
            if drone in nav_drones:              # already handed to its planner above
                continue
            pub = self.task_pubs.get(drone)
            if pub is None:                      # drone outside known_drones
                pub = self.create_publisher(DroneTask, f'/{drone}/task', 10)
                self.task_pubs[drone] = pub
            pub.publish(task)
            if target is not None:
                self.arrival[drone] = target

    def _publish_nav_goals(self, ph):
        """Mission -> nav: publish each drone's target on /{d}/nav_goal so the local_planner
        drives it there (obstacle-aware). The phase advances on the SAME arrival gate as
        goto (drone within tolerance of the target). Needs the nav stack (nav:=true); with
        no planner running the drone won't move and the phase will time out."""
        params = ph.get('params') or {}
        drones = list(ph.get('drones') or self.known_drones)
        t = params.get('target', [0.0, 0.0, self.default_alt])
        tx, ty, tz = float(t[0]), float(t[1]), float(t[2])
        for d in drones:
            pub = self.navgoal_pubs.get(d)
            if pub is None:
                pub = self.create_publisher(Point, f'/{d}/nav_goal', 10)
                self.navgoal_pubs[d] = pub
            pub.publish(Point(x=tx, y=ty, z=tz))
            self.arrival[d] = (tx, ty, tz)      # advance when the drone reaches the goal

    def _publish_formation_goal(self, ph):
        """Agentic path: broadcast the formation intent and set a slot-coverage gate.

        We compute the same world slots the agents will, but only to CHECK coverage
        (any drone within tolerance of each slot) — we do not assign them; the agents
        do that via their auction.
        """
        params = ph.get('params') or {}
        want = list(ph.get('drones') or self.known_drones)
        # Size the formation to the drones that are actually alive (agentic self-heal):
        # if a drone has dropped, the survivors form a smaller shape and the coverage
        # gate expects only their slots. Fall back to `want` before any heartbeats.
        alive = [d for d in want if (self._now() - self.last_hb.get(d, -1e9)) <= self.hb_timeout]
        drones = alive or want
        shape = params.get('shape', 'wedge')
        spacing = float(params.get('spacing', 8.0))
        heading = float(params.get('heading', 0.0))
        anchor = params.get('anchor', [0.0, 0.0, self.default_alt])
        alt = float(anchor[2]) if len(anchor) > 2 else self.default_alt

        g = FormationGoal()
        g.header.stamp = self.get_clock().now().to_msg()
        g.mission_id = self.mission_id
        g.phase_idx = self.phase_idx
        g.shape = shape
        g.spacing = spacing
        g.heading = heading
        g.anchor = Point(x=float(anchor[0]), y=float(anchor[1]), z=float(alt))
        g.num_slots = len(drones)
        self.goal_pub.publish(g)

        # world slots (north, east) for the coverage check
        offs = formations.slot_offsets(shape, len(drones), spacing, math.radians(heading))
        self.slot_cover = [(float(anchor[1]) + dn, float(anchor[0]) + de) for dn, de in offs]

    def _tick(self):
        if not self.phases or self.phase_idx < 0:
            return
        ph = self.phases[self.phase_idx]
        elapsed = self._now() - self.phase_started
        timeout = float(ph.get('timeout', 30.0))

        if elapsed >= timeout:
            self.get_logger().warn(f'phase {self.phase_idx} timeout ({timeout}s) — advancing')
            self._next_phase()
            return
        if ph.get('primitive') == 'wait':
            if elapsed >= float((ph.get('params') or {}).get('duration', 0.0)):
                self._next_phase()
            return
        if ph.get('advance_when', 'all_arrived') != 'all_arrived':
            return
        # Agentic formation: advance when every slot is COVERED by some drone (we
        # don't care which — the agents decided that). Latched per slot so the auction
        # reshuffling underneath doesn't reset progress.
        if self.slot_cover is not None:
            for si, (sn, se) in enumerate(self.slot_cover):
                if si in self.arrived:
                    continue
                for d, pos in self.positions.items():
                    o_n, o_e = self.drone_origins.get(d, (0.0, 0.0))
                    if math.hypot((pos[1] + o_n) - sn, (pos[0] + o_e) - se) <= self.tolerance:
                        self.arrived.add(si)
                        break
            if len(self.arrived) == len(self.slot_cover):
                self.get_logger().info(
                    f'phase {self.phase_idx} complete (all slots covered)')
                self._next_phase()
            return
        if not self.arrival:
            if elapsed >= 2.0:      # nothing to arrive at (hold/land/rtl): settle
                self._next_phase()
            return
        # Latch arrivals: once a drone has reached its slot it stays "arrived" for
        # the phase, so a transient safety nudge (a brief separation 'ease clear'
        # that pushes it a metre off) cannot reset progress and stall the phase.
        for drone, tgt in self.arrival.items():
            if drone in self.arrived:
                continue
            pos = self.positions.get(drone)
            if pos is not None and all(
                    want is None or abs(got - want) <= self.tolerance
                    for got, want in zip(pos, tgt)):
                self.arrived.add(drone)
        if len(self.arrived) == len(self.arrival):
            self.get_logger().info(f'phase {self.phase_idx} complete (all arrived)')
            self._next_phase()

    # ---------- phase expansion ----------
    def _expand(self, ph):
        """-> {drone: (DroneTask, arrival_target|None)} for one phase."""
        drones = list(ph.get('drones') or self.known_drones)
        out = {}

        # explicit per-drone assignments take precedence
        assigns = ph.get('assignments') or []

        # Drones sharing a formation must be slotted TOGETHER. Slotting each one on its own
        # makes every drone read a ONE-slot formation and take the bare anchor, so "line,
        # 12 m spacing" collapsed to a single point with the drones merely stacked in their
        # altitude bands. Grouping first restores the requested shape and spacing, and lets
        # the nearest-slot matching in _assign_slots do its job across the whole group.
        slots = {}
        for members in self._formation_groups(assigns).values():
            slots.update(self._assign_slots(members[0][1], [d for d, _ in members]))

        for a in assigns:
            d = a.get('drone')
            if d:
                out[d] = self._make(d, a.get('primitive'), a.get('params') or {}, slots.get(d))

        if 'primitive' in ph:
            prim, params = ph['primitive'], (ph.get('params') or {})
            slots = self._assign_slots(params, drones) if prim == 'formation' else {}
            for d in drones:
                if d not in out:
                    out[d] = self._make(d, prim, params, slots.get(d))
        return out

    @staticmethod
    def _formation_groups(assigns):
        """-> {formation key: [(drone, params), ...]} for assignments sharing one formation.

        Two drones belong to the same formation when they were given the same shape, spacing,
        heading and anchor — which is exactly how a planner expresses "px4_2 and px4_3 fly a
        line together".
        """
        groups = {}
        for a in assigns:
            if a.get('primitive') != 'formation' or not a.get('drone'):
                continue
            p = a.get('params') or {}
            key = (p.get('shape', 'wedge'),
                   float(p.get('spacing', 8.0)),
                   float(p.get('heading', 0.0)),
                   tuple(float(v) for v in (p.get('anchor') or ())))
            groups.setdefault(key, []).append((a['drone'], p))
        return groups

    def _assign_slots(self, params, drones):
        """Match drones to formation slots by NEAREST first.

        Index-order assignment makes drones fly through each other (the apex slot
        can land on top of another drone's current position), which trips the
        separation guard. Greedy nearest-slot matching minimises travel and
        crossings. Returns {drone: Point} in each drone's LOCAL ENU frame.
        """
        shape = params.get('shape', 'wedge')
        spacing = float(params.get('spacing', 8.0))
        heading = math.radians(float(params.get('heading', 0.0)))
        anchor = params.get('anchor', [0.0, 0.0, self.default_alt])
        alt = float(anchor[2]) if len(anchor) > 2 else self.default_alt
        anchor_n, anchor_e = float(anchor[1]), float(anchor[0])

        offs = formations.slot_offsets(shape, len(drones), spacing, heading)
        world_slots = [(anchor_n + dn, anchor_e + de) for dn, de in offs]

        # current world positions (local ENU + origin offset)
        cur = {}
        for d in drones:
            p = self.positions.get(d)
            o_n, o_e = self.drone_origins.get(d, (0.0, 0.0))
            cur[d] = (p[1] + o_n, p[0] + o_e) if p is not None else None

        order = list(range(len(drones)))
        if all(cur[d] is not None for d in drones):
            pairs = sorted((math.dist(cur[d], world_slots[s]), di, s)
                           for di, d in enumerate(drones) for s in range(len(world_slots)))
            taken_d, taken_s, chosen = set(), set(), {}
            for _, di, s in pairs:
                if di in taken_d or s in taken_s:
                    continue
                chosen[di] = s
                taken_d.add(di)
                taken_s.add(s)
            order = [chosen.get(i, i) for i in range(len(drones))]

        out = {}
        for di, d in enumerate(drones):
            wn, we = world_slots[order[di]]
            o_n, o_e = self.drone_origins.get(d, (0.0, 0.0))
            out[d] = Point(x=float(we - o_e), y=float(wn - o_n), z=float(alt + self._layer(d)))
        return out

    def _layer(self, drone):
        """This drone's altitude-band offset (deterministic 3D deconfliction)."""
        idx = self.known_drones.index(drone) if drone in self.known_drones else 0
        return idx * self.altitude_layer

    def _make(self, drone, primitive, params, slot):
        task = DroneTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.mission_id = self.mission_id
        task.drone_id = drone
        task.priority = DroneTask.PRIORITY_NORMAL
        task.action = _ACTION.get(primitive, 'hold')
        task.cruise_speed = float(params.get('speed') or 0.0)   # 0 = controller default
        task.yaw = float('nan')                                 # NaN = face direction of travel
        layer = self._layer(drone)                              # keep each drone in its own band
        target = None

        if primitive == 'takeoff':
            alt = float(params.get('altitude', self.default_alt)) + layer
            task.target_altitude = alt
            target = (None, None, alt)
        elif primitive == 'goto':
            t = params.get('target', [0.0, 0.0, self.default_alt])
            z = float(t[2]) + layer
            task.target = Point(x=float(t[0]), y=float(t[1]), z=z)
            task.target_altitude = z
            target = (float(t[0]), float(t[1]), z)
        elif primitive == 'formation':
            p = slot if slot is not None else self._assign_slots(params, [drone])[drone]
            task.target = p
            task.target_altitude = p.z
            target = (p.x, p.y, p.z)
        return task, target

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
