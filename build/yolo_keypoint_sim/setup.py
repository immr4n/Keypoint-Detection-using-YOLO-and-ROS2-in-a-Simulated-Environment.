from setuptools import setup
from glob import glob
import os

package_name = 'yolo_keypoint_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],

    entry_points={
        'console_scripts': [
            'skeleton_marker_node = yolo_keypoint_sim.skeleton_marker_node:main',
            'skeleton_bones_node = yolo_keypoint_sim.skeleton_bones_node:main',
            'keypoints_tf_broadcaster = yolo_keypoint_sim.keypoints_tf_broadcaster:main',
            'gesture_to_cmdvel = yolo_keypoint_sim.gesture_to_cmdvel:main',
        ],
    },
)
