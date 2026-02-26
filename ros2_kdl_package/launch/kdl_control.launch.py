import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the parameters file
    config = os.path.join(get_package_share_directory('ros2_kdl_package'), 'config', 'kdl_params.yaml')
    
    # 1. The Main Control Node
    kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        parameters=[config],
        output='screen'
    )
    
    # 2. Q2(c): Gazebo Parameter Bridge for /set_pose service
    # Note: Depending on your specific ROS 2 version, 'ros_gz_bridge' might be named 'ros_ign_bridge'.
    # The assignment text refers to the "ign service", so this bridges the Ignition/Gazebo pose service.
    gazebo_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_service_bridge',
        arguments=[
            '/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    return LaunchDescription([
        kdl_node,
        gazebo_service_bridge
    ])