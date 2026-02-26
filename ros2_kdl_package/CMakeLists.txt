cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find the interface generator
find_package(rosidl_default_generators REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Trajectory.action"
)

include_directories(include)

# The Main Node (Action Server)
add_executable(ros2_kdl_node src/ros2_kdl_node.cpp src/kdl_control.cpp)
ament_target_dependencies(ros2_kdl_node rclcpp rclcpp_action orocos_kdl geometry_msgs sensor_msgs std_msgs)
rosidl_target_interfaces(ros2_kdl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# The Action Client
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action)
rosidl_target_interfaces(action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
  ros2_kdl_node
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()