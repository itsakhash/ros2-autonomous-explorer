#!/usr/bin/env python3
"""Converts Nav2's plain Twist commands into the TwistStamped the Gazebo bridge expects.

Listens for geometry_msgs/Twist on /cmd_vel and republishes geometry_msgs/TwistStamped
on /cmd_vel. Both use the same topic name; ROS only connects matching message types,
so Nav2 feeds this node and this node feeds the bridge.
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_nav2', self.on_twist, 10)
        self.get_logger().info('twist_relay started: Twist -> TwistStamped on /cmd_vel')

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
