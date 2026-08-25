#!/usr/bin/env python3
"""Mission entry point.

Accepts raw JSON (a plan) or a natural-language command on /mission/prompt, runs
it through the LLM planner (NL only) + the bounds validator, and publishes a
MissionPlan on /mission_plan.

The validator checks STRUCTURE and SAFETY BOUNDS — it is not an intent
whitelist. Anything composable from the primitives is allowed; unsafe numbers
are rejected, and capabilities the planner could not express are reported
(never silently swapped for a different mission).

    ros2 topic pub --once /mission/prompt std_msgs/String "data: 'take off to 4m'"
"""

import json
import re
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_msgs.msg import MissionPlan
from mission_interface import validator
from mission_interface.llm_client import LLMClient


# "drone 2", "uav3", "px4_1" — an operator naming a SPECIFIC vehicle.
_DRONE_REF = re.compile(r'\b(?:drone|uav|px4)[\s_-]*(\d+)\b', re.IGNORECASE)


def _names_specific_drones(prompt: str) -> bool:
    return bool(_DRONE_REF.search(prompt))


def _has_per_drone_scoping(plan: dict) -> bool:
    """True if any phase is scoped to particular drones."""
    return any(ph.get('assignments') or ph.get('drones')
               for ph in plan.get('phases', []) if isinstance(ph, dict))


class PromptGateway(Node):

    def __init__(self):
        super().__init__('prompt_gateway')

        self.default_alt = float(self.declare_parameter('default_altitude', 3.0).value)
        self.drones = list(self.declare_parameter('known_drones', ['px4_1']).value)
        # Built lazily on first NL prompt so JSON plans need no HF credentials.
        self.llm = None

        self.create_subscription(String, '/mission/prompt', self._on_prompt, 10)
        self.pub = self.create_publisher(MissionPlan, '/mission_plan', 10)
        self.get_logger().info('prompt_gateway up; listening on /mission/prompt')

    def _on_prompt(self, msg: String):
        raw = msg.data.strip()
        unsupported = []
        from_llm = not raw.startswith('{')

        if raw.startswith('{'):
            try:
                plan = json.loads(raw)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'bad JSON: {e}')
                return
            unsupported = plan.pop('unsupported', []) or []
        else:
            try:
                if self.llm is None:
                    self.llm = LLMClient(default_altitude=self.default_alt,
                                         drones=self.drones)
                plan, unsupported = self.llm.plan(raw)
            except Exception as e:
                self.get_logger().error(f'planning failed: {e}')
                return

            if _names_specific_drones(raw) and not _has_per_drone_scoping(plan):
                self.get_logger().error(
                    'plan REJECTED: prompt names specific drones but the plan has no '
                    'per-drone scoping ("drones"/"assignments") — it would command ALL '
                    'drones. Re-issue the command, or send a scoped JSON plan.')
                return

        # Degrade, don't reject: fix trivial placement variance, then CLAMP any
        # out-of-bounds value to its safe limit and fly the safe version.
        plan = validator.normalize_plan(plan)
        adjustments = validator.clamp_plan(plan)
        if adjustments:
            self.get_logger().warn(
                'clamped to safe limits (flying adjusted mission): ' + '; '.join(adjustments))
        ok, errors = validator.validate_plan(plan)

        # If it still fails and came from the LLM, ask the planner to FIX it once
        # (feed the errors back) instead of throwing the whole mission away.
        if not ok and from_llm:
            self.get_logger().warn(f'plan invalid {errors[:2]} — asking planner to repair')
            try:
                fixed, unsup2 = self.llm.repair(raw, plan, errors)
                fixed = validator.normalize_plan(fixed)
                ok2, errors2 = validator.validate_plan(fixed)
                if ok2:
                    plan, unsupported, ok = fixed, unsup2, True
                    self.get_logger().info('planner repaired the plan')
                else:
                    errors = errors2
            except Exception as e:
                self.get_logger().error(f'repair attempt failed: {e}')

        if not ok:
            self.get_logger().error(f'plan REJECTED after repair attempt: {errors}')
            return

        if unsupported:
            self.get_logger().warn(
                f'planner could not express: {unsupported} — flying the achievable part only')

        out = MissionPlan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.mission_id = plan.get('mission_id', uuid.uuid4().hex[:8])
        out.plan_json = json.dumps(plan)
        out.unsupported = [str(u) for u in unsupported]
        self.pub.publish(out)
        self.get_logger().info(
            f'published plan {out.mission_id}: {len(plan["phases"])} phase(s)')


def main(args=None):
    rclpy.init(args=args)
    node = PromptGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
