#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class GestureToCmdVel(Node):

    def __init__(self):
        super().__init__('gesture_to_cmdvel')
        self.sub = self.create_subscription(
            Bool,
            '/gesture_detected',
            self.cb,
            10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Gesture → cmd_vel node started')

    def cb(self, msg: Bool):
        twist = Twist()
        if msg.data:
            twist.linear.x = 0.3
        else:
            twist.linear.x = 0.0
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = GestureToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
