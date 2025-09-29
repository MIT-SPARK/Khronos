#!/usr/bin/env python3
"""
Publishes TF transform from odometry messages.
This is needed when bag files don't contain /tf topics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # Declare parameters
        self.declare_parameter('odom_topic', '/sparkal1/jackal_velocity_controller/odom')

        # Get parameters
        odom_topic = self.get_parameter('odom_topic').value

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Publishing TF from odometry topic: {odom_topic}')

    def odom_callback(self, msg):
        """Convert odometry message to TF transform and publish."""
        transform = TransformStamped()

        # Copy header
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id

        # Copy transform
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        # Publish transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()