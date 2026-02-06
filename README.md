YOLO Keypoint Simulation ROS2 Package
📌 Overview

This project implements a ROS2-based human pose keypoint detection and simulation system using YOLOv8 Pose Estimation. The package processes pose keypoints, performs 3D estimation, applies filtering, visualizes skeleton markers, and converts gestures into robot velocity commands.

The system is designed for robotics applications such as:

Human gesture control

Human pose tracking

Human-robot interaction

Simulation-based robotics research

🧰 Features

YOLOv8 pose detection integration

3D keypoint estimation

Kalman filter smoothing

Skeleton visualization using RViz markers

TF broadcasting for robot frames

Gesture-to-velocity command conversion

Full system ROS2 launch support

🏗️ Project Structure
ros2_ws/
│
├── src/
│   ├── yolo_keypoint_sim/
│   │   ├── launch/
│   │   │   └── full_system.launch.py
│   │   ├── yolo_keypoint_sim/
│   │   │   ├── kalman_3d_node.py
│   │   │   ├── skeleton_marker_node.py
│   │   │   ├── skeleton_bones_node.py
│   │   │   ├── keypoints_tf_broadcaster.py
│   │   │   ├── keypoints_to_markers.py
│   │   │   └── gesture_to_cmdvel.py
│   │   ├── requirements.txt
│   │   ├── package.xml
│   │   └── setup.py
│
├── yolov8n-pose.pt

⚙️ Requirements
Software

ROS2 (Humble / Iron / Rolling recommended)

Python 3.10+

OpenCV

Ultralytics YOLOv8

RViz2

Python Dependencies

Install dependencies:

pip install -r ros2_ws/src/yolo_keypoint_sim/requirements.txt

📥 Installation
1️⃣ Clone or Extract Project
cd ~
unzip yolo_keypoint_sim.zip

2️⃣ Navigate to Workspace
cd ros2_ws

3️⃣ Install ROS Dependencies
rosdep install --from-paths src --ignore-src -r -y

4️⃣ Build Workspace
colcon build

5️⃣ Source Workspace
source install/setup.bash

▶️ Running the System

Launch the full system:

ros2 launch yolo_keypoint_sim full_system.launch.py

🤖 Node Descriptions
kalman_3d_node

Applies Kalman filtering to smooth 3D keypoint estimation.

skeleton_marker_node

Publishes visualization markers for detected skeleton joints.

skeleton_bones_node

Connects keypoints to create skeleton bone visualization.

keypoints_tf_broadcaster

Broadcasts keypoint transforms using ROS TF.

keypoints_to_markers

Converts keypoints into visualization markers for RViz.

gesture_to_cmdvel

Converts human gestures into robot velocity commands (cmd_vel).

📊 Visualization

Run RViz2 to visualize skeleton tracking:

rviz2


Add:

Marker Topics

TF Frames

🧠 Model Used
yolov8n-pose.pt


Ultralytics lightweight pose estimation model.

🔧 Customization

You can:

Replace YOLO model

Modify gesture mapping logic

Extend skeleton visualization

Integrate with real robot hardware

🧪 Testing

Run code style and validation tests:

colcon test

🚀 Applications

Gesture-controlled mobile robots

Autonomous robot navigation

Rehabilitation robotics

AI human motion tracking research

👨‍💻 Author

Mohammed Imran Ibrahim
MSc Artificial Intelligence
International University of Applied Sciences Berlin

📜 License

This project is for educational and research purposes.
