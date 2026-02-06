#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
import math

class SkeletonMarkerNode(Node):
    def __init__(self):
        super().__init__('skeleton_marker_node')

        self.pub = self.create_publisher(
            MarkerArray,
            '/skeleton_markers',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_cb)
        self.t = 0.0

    def timer_cb(self):
        markers = MarkerArray()

        # simple moving "skeleton" demo (2 points + bone)
        p1 = Point(x=math.sin(self.t), y=0.0, z=1.0)
        p2 = Point(x=math.sin(self.t), y=0.0, z=0.5)

        m = Marker()
        m.header.frame_id = 'camera_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bones'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.points = [p1, p2]

        markers.markers.append(m)
        self.pub.publish(markers)

        self.t += 0.1

def main():
    rclpy.init()
    node = SkeletonMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
