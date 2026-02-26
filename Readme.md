# Robotics Lab - Homework 2: Kinematic & Vision-Based Control
**Student:** Dhruv Yadav  


## System Overview
This repository contains the solution for Homework 2. It implements a custom C++ ROS 2 control node (`ros2_kdl_node`) that interfaces with the 7-DOF LBR IIWA robotic manipulator. The project demonstrates two core robotic capabilities:
1. **Kinematic Control with Redundancy Resolution:** A velocity-resolved controller utilizing the Moore-Penrose Pseudo-Inverse and Null Space Projection to achieve target end-effector poses while optimizing joint posture.
2. **Vision-Based Control (IBVS):** A "Look-at-Point" visual servoing controller that uses an eye-in-hand camera configuration to detect and track an ArUco fiducial marker (ID 115) using an Interaction Matrix $L(s)$.

---

## Prerequisites & Dependency Management

Before building the workspace, ensure you have resolved the Python dependency conflicts required for the `aruco_ros` computer vision pipeline. 

Run the following commands to install the required matrix math and vision libraries, ensuring compatibility with the ROS 2 message serialization tools:
```bash
pip3 install "numpy<2.0"
pip3 install opencv-contrib-python transforms3d

Workspace Setup and Build Instructions
This project requires three main packages to function. Follow these steps to set up the complete environment.

1. Clone the repositories into your src folder:

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1. Clone this assignment repository
git clone [https://github.com/rdhruvyadav5151/myHW2_Dhruv_Yadav.git](https://github.com/rdhruvyadav5151/myHW2_Dhruv_Yadav.git)

# 2. Clone the required base simulation and vision packages 
git clone <URL_FOR_ROS2_IIWA> ros2_iiwa
git clone <URL_FOR_ARUCO_ROS> aruco_ros


2. Build the workspace:

Part 1: Kinematic Control & Action Server (Questions 1a, 1b, 1c)
1. Launch the Controller Node:
The node loads operational parameters (traj_duration, Kp, end_position, etc.) dynamically from config/kdl_params.yaml.

ros2 launch ros2_kdl_package kdl_control.launch.py cmd_interface:="velocity"

Part 2: Vision-Based Control & Service Bridge (Questions 2a, 2b, 2c)
1. Launch the Visual Servoing Controller:
To activate the IBVS controller, override the command interface parameter at launch. The robot will subscribe to the /aruco_single/pose topic and adjust its joints to center the marker.

ros2 launch ros2_kdl_package kdl_control.launch.py cmd_interface:="look_at_point"

2. Dynamically Update Marker Position (Gazebo Service Bridge):
A ros_gz_bridge is included in the launch file to expose Gazebo's /set_pose service to the ROS 2 network. To test the robot's dynamic tracking capabilities, move the ArUco marker by opening a new terminal and running:

ros2 service call /set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'aruco_tag'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"

