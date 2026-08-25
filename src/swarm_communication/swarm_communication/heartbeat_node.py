#!/usr/bin/env python3
"""Heartbeat + status-sharing node (low bandwidth only).

One instance per drone. It emits a tiny Heartbeat on the shared /swarm/heartbeat
bus at a fixed rate, tagging it with a coarse health level derived from the local
DroneStatus. This is the ONLY always-on inter-drone traffic in Stage 1 — keep it
small. No mission logic, no control, no heavy computation lives here.
"""

import rclpy
from rclpy.node import Node

from swarm_msgs.msg import Heartbeat, DroneStatus


class HeartbeatNode(Node):

    def __init__(self):
        super().__init__('heartbeat_node')

        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.rate_hz = float(self.declare_parameter('rate_hz', 1.0).value)

        self.seq = 0
        self.status_level = Heartbeat.STATUS_UNKNOWN

        # Read local status (published by drone_controller) to color the beacon.
        self.create_subscription(
            DroneStatus, f'/{self.drone_id}/status', self._on_status, 10)
        self.pub = self.create_publisher(Heartbeat, '/swarm/heartbeat', 10)

        self.create_timer(1.0 / self.rate_hz, self._beat)
        self.get_logger().info(f'heartbeat_node up for {self.drone_id} @ {self.rate_hz} Hz')

    def _on_status(self, msg: DroneStatus):
        # arming_state is unavailable on this firmware/px4_msgs combo, so key
        # health on offboard_active: the controller sets it once it is actively
        # commanding the vehicle in offboard.
        self.status_level = (Heartbeat.STATUS_OK if msg.offboard_active
                             else Heartbeat.STATUS_WARN)

    def _beat(self):
        hb = Heartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.drone_id = self.drone_id
        hb.seq = self.seq
        hb.status = self.status_level
        self.pub.publish(hb)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
