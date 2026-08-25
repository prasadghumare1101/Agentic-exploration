#!/usr/bin/env python3
"""Per-drone swarm agent — decentralised formation self-assignment + self-healing.

In agentic mode the coordinator stops dictating exactly where each drone goes. For
a formation phase it broadcasts a FormationGoal (shape + anchor + spacing) and the
drone_agents decide AMONGST THEMSELVES who takes which slot, via a distributed
auction:

  1. every alive agent computes the SAME candidate slots for the current alive-set,
  2. each broadcasts its own travel COST to every slot (a SlotBid),
  3. each agent collects all bids and runs the SAME deterministic assignment
     (greedy nearest, collision-minimising) -> everyone converges on one layout
     with NO central assigner,
  4. each agent commands its OWN drone to the slot it won.

Self-healing: alive membership comes from the shared heartbeat. If a peer stops
beating, the survivors recompute slots for the smaller group and re-bid, so the
formation/coverage redistributes itself — emergent recovery, not a scripted
failover. Non-formation actions (takeoff/goto/hold/rtl) stay with the coordinator;
the agent only owns the "who goes where" decision for formations.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

from swarm_msgs.msg import FormationGoal, SlotBid, DroneStatus, DroneTask, Heartbeat
from formation_coordinator import formations


class DroneAgent(Node):

    def __init__(self):
        super().__init__('drone_agent')

        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.known_drones = list(self.declare_parameter('known_drones', ['px4_1']).value)
        self.hb_timeout = float(self.declare_parameter('heartbeat_timeout', 3.0).value)
        self.altitude_layer = float(self.declare_parameter('altitude_layer', 2.5).value)

        # Shared-frame origins ['px4_1:0:0', ...] (id:north:east) — same as coordinator.
        self.origins = {}
        for s in (self.declare_parameter(
                'drone_origins', [], ParameterDescriptor(dynamic_typing=True)).value or []):
            p = s.split(':')
            if len(p) == 3:
                self.origins[p[0]] = (float(p[1]), float(p[2]))

        self.world = None                 # own (north, east) in shared frame
        self.goal = None                  # latest FormationGoal
        self.bids = {}                    # drone_id -> FROZEN first costs[] for the auction
        self.last_hb = {}                 # drone_id -> t last heartbeat (liveness)
        self.auction_key = None           # (mission_id, phase_idx, n_alive) currently bidding on
        self.my_costs = None              # our own frozen bid for auction_key
        self.committed = False            # have we assigned+commanded for auction_key

        self.create_subscription(DroneStatus, f'/{self.drone_id}/status', self._on_status, 10)
        self.create_subscription(FormationGoal, '/swarm/formation_goal', self._on_goal, 10)
        self.create_subscription(SlotBid, '/swarm/slot_bids', self._on_bid, 10)
        self.create_subscription(Heartbeat, '/swarm/heartbeat', self._on_heartbeat, 10)

        self.pub_bid = self.create_publisher(SlotBid, '/swarm/slot_bids', 10)
        self.pub_task = self.create_publisher(DroneTask, f'/{self.drone_id}/task', 10)

        # 5 Hz: re-evaluate bids/assignment as positions, goals and liveness change.
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'drone_agent up for {self.drone_id} (agentic slot auction)')

    # ---------- inputs ----------
    def _on_status(self, msg):
        o_n, o_e = self.origins.get(self.drone_id, (0.0, 0.0))
        # DroneStatus.position is local ENU (x=East, y=North); lift into shared frame.
        self.world = (msg.position.y + o_n, msg.position.x + o_e)

    def _on_heartbeat(self, msg):
        self.last_hb[msg.drone_id] = self._now()

    def _on_goal(self, msg):
        self.goal = msg                   # auction state resets in _tick when the key changes

    def _on_bid(self, msg):
        g = self.goal
        if g is None or msg.mission_id != g.mission_id or msg.phase_idx != g.phase_idx:
            return                        # stale bid for another goal — ignore
        # Keep each drone's FIRST bid for this auction. Freezing bids is what makes the
        # assignment stable: without it, every agent recomputes as drones move and near-
        # equidistant slots flip back and forth (limit-cycle thrash). All agents keep the
        # same first bids -> the same deterministic assignment -> one stable layout.
        self.bids.setdefault(msg.drone_id, list(msg.costs))

    # ---------- auction ----------
    def _alive(self):
        """Drones we've heard from within the heartbeat window (self always alive)."""
        now = self._now()
        alive = {d for d, t in self.last_hb.items() if now - t <= self.hb_timeout}
        alive.add(self.drone_id)
        # keep only known drones, in deterministic order so every agent agrees
        return [d for d in self.known_drones if d in alive]

    def _world_slots(self, alive):
        g = self.goal
        anchor_n, anchor_e = g.anchor.y, g.anchor.x
        offs = formations.slot_offsets(g.shape, len(alive), g.spacing,
                                       math.radians(g.heading))
        return [(anchor_n + dn, anchor_e + de) for dn, de in offs]

    def _tick(self):
        if self.goal is None or self.world is None:
            return
        alive = self._alive()
        if self.drone_id not in alive:
            return
        n = len(alive)
        # Auction identity = (mission, phase, who's alive). It changes on a new phase
        # OR when the alive-set changes (a drop/recovery) -> that is what triggers a
        # fresh auction and self-heals the formation. Within one key the layout is FROZEN.
        key = (self.goal.mission_id, self.goal.phase_idx, n)
        if key != self.auction_key:
            self.auction_key = key
            self.bids = {}
            self.committed = False
            slots = self._world_slots(alive)
            self.my_costs = [math.dist(self.world, s) for s in slots]  # freeze our bid
            self.bids[self.drone_id] = self.my_costs

        if self.committed:
            return                        # stable: hold the assignment for this phase

        # (re)broadcast our frozen bid until we commit, so every agent receives it
        bid = SlotBid()
        bid.header.stamp = self.get_clock().now().to_msg()
        bid.mission_id = self.goal.mission_id
        bid.phase_idx = self.goal.phase_idx
        bid.drone_id = self.drone_id
        bid.costs = [float(c) for c in self.my_costs]
        self.pub_bid.publish(bid)

        # need a matching-length frozen bid from every alive drone before assigning
        usable = {d: self.bids[d] for d in alive
                  if d in self.bids and len(self.bids[d]) == n}
        if len(usable) < n:
            return

        # same greedy nearest assignment on every agent -> one consistent, stable layout
        order = sorted(alive)             # deterministic drone ordering
        pairs = sorted((usable[d][s], di, s)
                       for di, d in enumerate(order) for s in range(n))
        taken_d, taken_s, chosen = set(), set(), {}
        for _, di, s in pairs:
            if di in taken_d or s in taken_s:
                continue
            chosen[di] = s
            taken_d.add(di)
            taken_s.add(s)

        my_rank = order.index(self.drone_id)
        my_slot = chosen.get(my_rank)
        if my_slot is None:
            return

        # commit once and command our own drone to the slot we won
        self.committed = True
        slots = self._world_slots(alive)
        wn, we = slots[my_slot]
        o_n, o_e = self.origins.get(self.drone_id, (0.0, 0.0))
        alt = self.goal.anchor.z + my_rank * self.altitude_layer   # keep 3D deconfliction
        self._send_goto(Point(x=float(we - o_e), y=float(wn - o_n), z=float(alt)))
        self.get_logger().info(
            f'[{self.drone_id}] auction: {n} drones -> won slot {my_slot} '
            f'(phase {self.goal.phase_idx})')

    def _send_goto(self, target):
        task = DroneTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.mission_id = self.goal.mission_id
        task.drone_id = self.drone_id
        task.action = 'goto'
        task.target = target
        task.target_altitude = target.z
        task.yaw = float('nan')
        task.priority = DroneTask.PRIORITY_NORMAL
        self.pub_task.publish(task)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = DroneAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
