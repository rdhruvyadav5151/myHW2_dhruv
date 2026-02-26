#ifndef ROS2_KDL_NODE_HPP
#define ROS2_KDL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "ros2_kdl_package/kdl_control.hpp"
#include "ros2_kdl_package/action/trajectory.hpp" 

class ROS2KDLNode : public rclcpp::Node {
public:
    using TrajectoryAction = ros2_kdl_package::action::Trajectory;
    using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<TrajectoryAction>;

    ROS2KDLNode();

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void update();

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TrajectoryAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTrajectory> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle);
    void execute_trajectory(const std::shared_ptr<GoalHandleTrajectory> goal_handle);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_aruco_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<TrajectoryAction>::SharedPtr action_server_;

    std::shared_ptr<KDLControl> controller_;
    KDL::JntArray q_curr_;
    KDL::JntArray q_dot_curr_;
    KDL::Frame aruco_pose_;
    
    std::string cmd_mode_;
    double t_;
    double total_time_;
    bool state_received_;
    std::vector<double> current_target_;
};

#endif