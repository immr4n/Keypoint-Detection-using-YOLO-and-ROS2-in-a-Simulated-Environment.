import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray

class Kalman3DNode(Node):
    def __init__(self):
        super().__init__('kalman_3d_node')

        self.sub = self.create_subscription(
            MarkerArray,
            '/skeleton_markers',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            '/skeleton_markers_filtered',
            10
        )

        self.get_logger().info('Kalman 3D node started')

    def cb(self, msg):
        # (Placeholder: pass-through, add Kalman math later)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Kalman3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
