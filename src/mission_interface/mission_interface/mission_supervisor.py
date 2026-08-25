#!/usr/bin/env python3
"""Mission supervisor — the agentic loop that closes AI around the flying swarm.

The prompt_gateway plans a mission ONCE, up front. This node keeps watching while
it flies and RE-PLANS when reality diverges from the plan — today's trigger is a
drone dropping out mid-mission (its heartbeat goes stale). It hands the LLM the
original intent plus the new reality ("px4_2 is lost; only px4_1, px4_3 remain")
and publishes the revised MissionPlan, so the surviving drones autonomously re-task
to finish the job instead of the mission silently degrading.

This is the difference between "an LLM that plans" and "an agent": perceive
(telemetry) -> decide (LLM) -> act (new plan) -> keep watching.

Decentralised slot re-assignment (which drone flies where in the smaller group) is
handled by the drone_agents' auction; this node handles the MISSION-level rethink.
"""

import json
import threading
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_msgs.msg import MissionPlan, Heartbeat
from mission_interface import validator
from mission_interface.llm_client import LLMClient


class MissionSupervisor(Node):

    def __init__(self):
        super().__init__('mission_supervisor')

        self.default_alt = float(self.declare_parameter('default_altitude', 3.0).value)
        self.drones = list(self.declare_parameter('known_drones', ['px4_1']).value)
        self.hb_timeout = float(self.declare_parameter('heartbeat_timeout', 3.0).value)
        # Ignore drops once a mission is likely finished (no telemetry-driven replan
        # when idle). Refreshed every time a plan is published.
        self.active_window = float(self.declare_parameter('active_window', 180.0).value)

        self.llm = None                    # built lazily (needs HF creds)
        self.last_prompt = None            # original operator intent (NL)
        self.last_plan_t = -1e9
        self.last_hb = {}                  # drone_id -> t
        self.ever_seen = set()             # drones that have EVER been alive
        self.lost = set()                  # drones we've already reacted to
        self._replanning = False           # a replan thread is in flight

        self.create_subscription(String, '/mission/prompt', self._on_prompt, 10)
        self.create_subscription(MissionPlan, '/mission_plan', self._on_plan, 10)
        self.create_subscription(Heartbeat, '/swarm/heartbeat', self._on_hb, 10)
        self.pub = self.create_publisher(MissionPlan, '/mission_plan', 10)

        self.create_timer(1.0, self._watch)
        self.get_logger().info('mission_supervisor up; watching swarm health')

    # ---------- inputs ----------
    def _on_prompt(self, msg):
        raw = msg.data.strip()
        if not raw.startswith('{'):        # remember NL intent, not raw JSON plans
            self.last_prompt = raw
        self.lost.clear()                  # a new command starts a fresh mission

    def _on_plan(self, msg):
        self.last_plan_t = self._now()

    def _on_hb(self, msg):
        self.last_hb[msg.drone_id] = self._now()
        self.ever_seen.add(msg.drone_id)

    # ---------- the agentic loop ----------
    def _watch(self):
        now = self._now()
        if self._replanning:               # one replan at a time; don't stack calls
            return
        if self.last_prompt is None or (now - self.last_plan_t) > self.active_window:
            return                         # nothing active to supervise
        alive = {d for d in self.ever_seen if now - self.last_hb.get(d, -1e9) <= self.hb_timeout}
        newly_lost = (self.ever_seen - alive) - self.lost
        if not newly_lost:
            return
        self.lost |= newly_lost
        survivors = sorted(alive)
        if not survivors:
            self.get_logger().error('all drones lost — cannot replan')
            return
        self.get_logger().warn(
            f'drone(s) lost: {sorted(newly_lost)} — replanning for survivors {survivors}')
        # Run the LLM call OFF the executor thread: it can take tens of seconds, and
        # blocking here would starve heartbeat processing and make live drones look
        # stale on the next tick (a false "all drones lost").
        self._replanning = True
        threading.Thread(target=self._replan, args=(sorted(newly_lost), survivors),
                         daemon=True).start()

    def _replan(self, lost, survivors):
        try:
            if self.llm is None:
                self.llm = LLMClient(default_altitude=self.default_alt, drones=survivors)
            else:
                self.llm.drones = survivors     # re-plan for the reduced fleet
            situation = (
                f'{self.last_prompt}\n\n'
                f'[SUPERVISOR UPDATE] Drone(s) {lost} have failed mid-mission and are no '
                f'longer available. Only these drones remain: {survivors}. Re-plan the '
                f'mission to be completed by the remaining drones, keeping the original '
                f'intent and coverage. Scope every phase to the remaining drones.')
            plan, unsupported = self.llm.plan(situation)
            plan = validator.normalize_plan(plan)
            validator.clamp_plan(plan)
            ok, errors = validator.validate_plan(plan)
            if not ok:
                self.get_logger().error(f'replan rejected: {errors[:2]}')
                return
            out = MissionPlan()
            out.header.stamp = self.get_clock().now().to_msg()
            out.mission_id = plan.get('mission_id', 'reheal-' + uuid.uuid4().hex[:6])
            out.plan_json = json.dumps(plan)
            out.unsupported = [str(u) for u in unsupported]
            self.pub.publish(out)
            self.get_logger().info(
                f'republished healed plan {out.mission_id}: {len(plan["phases"])} phase(s) '
                f'for {survivors}')
        except Exception as e:
            self.get_logger().error(f'replan failed: {e}')
        finally:
            self._replanning = False

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
