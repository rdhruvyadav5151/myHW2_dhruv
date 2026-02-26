#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_kdl_package/action/trajectory.hpp"

using TrajectoryAction = ros2_kdl_package::action::Trajectory;
using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<TrajectoryAction>;

class TrajectoryActionClient : public rclcpp::Node {
public:
    TrajectoryActionClient() : Node("trajectory_action_client") {
        client_ = rclcpp_action::create_client<TrajectoryAction>(this, "execute_trajectory");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryActionClient::send_goal, this));
    }

private:
    void send_goal() {
        timer_->cancel();
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.end_position = {0.6, 0.1, 0.4};

        auto send_goal_options = rclcpp_action::Client<TrajectoryAction>::SendGoalOptions();
        send_goal_options.feedback_callback = [](GoalHandleTrajectory::SharedPtr, const std::shared_ptr<const TrajectoryAction::Feedback> feedback) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feedback Error: [%.2f, %.2f, %.2f]", feedback->position_error[0], feedback->position_error[1], feedback->position_error[2]);
        };
        send_goal_options.result_callback = [](const GoalHandleTrajectory::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded!");
            rclcpp::shutdown();
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<TrajectoryAction>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}