from visualization_msgs.msg import Marker

import rclpy

from rclpy.node import Node

from visualization_msgs.msg import MarkerArray



class SkeletonBones(Node):


    BONES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),(10,11),(0,12),(12,13),(13,14)]



    def cb(self, msg):

        from visualization_msgs.msg import Marker

        bones = Marker()

        bones.header = msg.markers[0].header

        bones.ns = "bones"

        bones.id = 0

        bones.type = Marker.LINE_LIST

        bones.action = Marker.ADD

        bones.scale.x = 0.03

        bones.color.r = 0.0

        bones.color.g = 1.0

        bones.color.b = 0.0

        bones.color.a = 1.0



        joints = {m.id: m.pose.position for m in msg.markers}



        for a,b in self.BONES:

            if a in joints and b in joints:

                bones.points.append(joints[a])

                bones.points.append(joints[b])



        self.pub.publish(bones)


    def __init__(self):

        super().__init__("skeleton_bones_node")

        self.pub = self.create_publisher(Marker, "/skeleton_bones", 10)

        self.sub = self.create_subscription(

            MarkerArray,

            "/skeleton_markers",

            self.cb,

            10

        )

        self.get_logger().info("SkeletonBones node started")



    def cb(self, msg):

        pass




def main():
    import rclpy
    rclpy.init()
    node = SkeletonBones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
