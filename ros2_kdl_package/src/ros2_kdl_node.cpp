#include "ros2_kdl_package/ros2_kdl_node.hpp"
#include <thread>

ROS2KDLNode::ROS2KDLNode() : Node("ros2_kdl_node"), t_(0.0), state_received_(false) {
    this->declare_parameter("cmd_interface", "velocity");
    this->declare_parameter("traj_duration", 10.0);
    this->declare_parameter("acc_duration", 2.0);
    this->declare_parameter("total_time", 20.0);
    this->declare_parameter("trajectory_len", 200);
    this->declare_parameter("Kp", std::vector<double>{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    this->declare_parameter("end_position", std::vector<double>{0.5, 0.0, 0.5});

    cmd_mode_ = this->get_parameter("cmd_interface").as_string();
    total_time_ = this->get_parameter("total_time").as_double();
    current_target_ = this->get_parameter("end_position").as_double_array();

    controller_ = std::make_shared<KDLControl>();

    sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&ROS2KDLNode::joint_callback, this, std::placeholders::_1));
    
    sub_aruco_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_single/pose", 10, std::bind(&ROS2KDLNode::aruco_callback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/iiwa_arm_controller/commands", 10);
    
    // Initialize Action Server
    action_server_ = rclcpp_action::create_server<TrajectoryAction>(
        this, "execute_trajectory",
        std::bind(&ROS2KDLNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ROS2KDLNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ROS2KDLNode::handle_accepted, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ROS2KDLNode::update, this));
}

rclcpp_action::GoalResponse ROS2KDLNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TrajectoryAction::Goal> goal) {
    (void)uuid;
    if (goal->end_position.size() != 3) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ROS2KDLNode::handle_cancel(const std::shared_ptr<GoalHandleTrajectory> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ROS2KDLNode::handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle) {
    std::thread{std::bind(&ROS2KDLNode::execute_trajectory, this, std::placeholders::_1), goal_handle}.detach();
}

void ROS2KDLNode::execute_trajectory(const std::shared_ptr<GoalHandleTrajectory> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrajectoryAction::Feedback>();
    auto result = std::make_shared<TrajectoryAction::Result>();

    // Update target for the control loop
    current_target_ = goal->end_position;
    
    rclcpp::Rate loop_rate(10); // 10 Hz feedback
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // [Inference] Simulating position error feedback calculation for demonstration
        feedback->position_error = {0.1, 0.1, 0.1}; 
        goal_handle->publish_feedback(feedback);

        if (t_ >= total_time_) {
            break;
        }
        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
    }
}

void ROS2KDLNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (q_curr_.data.size() != msg->position.size()) {
        q_curr_.resize(msg->position.size());
        q_dot_curr_.resize(msg->velocity.size());
    }
    for (size_t i = 0; i < msg->position.size(); ++i) {
        q_curr_(i) = msg->position[i];
        q_dot_curr_(i) = msg->velocity[i];
    }
    state_received_ = true;
}

void ROS2KDLNode::aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    aruco_pose_.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    aruco_pose_.M = KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void ROS2KDLNode::update() {
    if (!state_received_) return;

    KDL::JntArray q_dot(7);
    t_ += 0.01;
    if (t_ > total_time_) t_ = total_time_;

    if (cmd_mode_ == "look_at_point") {
        q_dot = controller_->look_at_point_control(q_curr_, aruco_pose_);
    } else {
        q_dot = controller_->velocity_ctrl_null(q_curr_, t_);
    }

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(7);
    for (int i = 0; i < 7; ++i) cmd_msg.data[i] = q_dot(i);
    pub_cmd_->publish(cmd_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROS2KDLNode>());
    rclcpp::shutdown();
    return 0;
}