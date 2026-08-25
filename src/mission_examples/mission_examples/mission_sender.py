#!/usr/bin/env python3
"""Tiny helper to fire an example mission at the pipeline.

Reads a JSON mission file and publishes its contents (as a String) to
/mission/prompt, which prompt_gateway validates and forwards. Handy for
demonstrating mission start and mid-mission update without hand-typing JSON.

    ros2 run mission_examples mission_sender --ros-args -p file:=<abs_path.json>
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionSender(Node):

    def __init__(self):
        super().__init__('mission_sender')
        path = self.declare_parameter('file', '').value
        if not path:
            self.get_logger().error('set -p file:=<path to mission json>')
            rclpy.shutdown()
            sys.exit(1)

        with open(path, 'r') as f:
            payload = f.read().strip()

        self.payload = payload
        self.sent = False
        self.pub = self.create_publisher(String, '/mission/prompt', 10)
        # One-shot: give discovery a beat, publish, then stop spinning.
        self.create_timer(0.5, self._send)

    def _send(self):
        if self.sent:
            return
        self.sent = True
        self.pub.publish(String(data=self.payload))
        self.get_logger().info(f'sent mission:\n{self.payload}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MissionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
