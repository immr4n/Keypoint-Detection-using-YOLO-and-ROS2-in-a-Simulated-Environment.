from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='yolo_keypoint_sim',
            executable='skeleton_marker_node',
            output='screen'
        ),

        Node(
            package='yolo_keypoint_sim',
            executable='skeleton_bones_node',
            output='screen'
        ),

        Node(
            package='yolo_keypoint_sim',
            executable='keypoints_tf_broadcaster',
            output='screen'
        ),

        Node(
            package='yolo_keypoint_sim',
            executable='gesture_to_cmdvel',
            output='screen'
        ),
    ])
