#!/usr/bin/env python3
"""PX4 pose -> ROS odometry + TF, so rtabmap can map with the flight controller's own
localisation instead of fragile visual odometry.

This is glue, not a mapper: it republishes the drone's EKF pose (which PX4 already
computes) as the `odom -> base_link` transform + a nav_msgs/Odometry message that
rtabmap consumes. rtabmap then builds the map/point-cloud; the camera TF
(base_link -> camera) is static in the launch.

Frame conversion: PX4 VehicleLocalPosition is NED (x=North, y=East, z=Down, heading
CW from North). ROS uses ENU (x=East, y=North, z=Up, yaw CCW from East):
    x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned, yaw_enu = pi/2 - heading.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)


def _yaw_quat(yaw):
    """Quaternion (z,w) for a yaw about Z (roll=pitch=0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Px4OdomTf(Node):

    def __init__(self):
        super().__init__('px4_odom_tf')
        self.drone_id = self.declare_parameter('drone_id', 'px4_1').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.pub = self.create_publisher(Odometry, f'/{self.drone_id}/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.drone_id}/fmu/out/vehicle_local_position_v1', self._cb, SENSOR_QOS)
        self.get_logger().info(f'px4_odom_tf up: {self.drone_id} -> /odom + {self.odom_frame}'
                               f'->{self.base_frame} TF')

    def _cb(self, m):
        ex, ny, uz = m.y, m.x, -m.z                 # NED -> ENU position
        qz, qw = _yaw_quat(math.pi / 2.0 - m.heading)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(ex)
        odom.pose.pose.position.y = float(ny)
        odom.pose.pose.position.z = float(uz)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(ex)
        t.transform.translation.y = float(ny)
        t.transform.translation.z = float(uz)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
