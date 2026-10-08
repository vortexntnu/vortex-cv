#include "vortex_bt_nodes/actuators/set_gripper.hpp"

#include <spdlog/spdlog.h>

#include <vortex_msgs/msg/gripper_waypoint.hpp>

namespace vortex_bt_nodes::actuators {

BT::PortsList SetGripper::providedPorts() {
    return {BT::InputPort<double>("roll", 0.0, "[rad]"),
            BT::InputPort<double>("pinch", 0.0, "[rad]"),
            BT::InputPort<std::string>("mode", "ROLL_AND_PINCH",
                                       "ROLL_AND_PINCH, ONLY_ROLL, ONLY_PINCH"),
            BT::InputPort<std::string>("action", "gripper_reference_filter",
                                       "Action server")};
}

BT::NodeStatus SetGripper::onStart() {
    if (!client_) {
        client_ = rclcpp_action::create_client<Action>(
            node_, getInput<std::string>("action").value_or(
                       "gripper_reference_filter"));
    }
    const std::string mode =
        getInput<std::string>("mode").value_or("ROLL_AND_PINCH");
    Action::Goal goal;
    if (mode == "ROLL_AND_PINCH") {
        goal.waypoint.mode = vortex_msgs::msg::GripperWaypoint::ROLL_AND_PINCH;
    } else if (mode == "ONLY_ROLL") {
        goal.waypoint.mode = vortex_msgs::msg::GripperWaypoint::ONLY_ROLL;
    } else if (mode == "ONLY_PINCH") {
        goal.waypoint.mode = vortex_msgs::msg::GripperWaypoint::ONLY_PINCH;
    } else {
        spdlog::warn("[{}] unknown mode '{}'", name(), mode);
        return BT::NodeStatus::FAILURE;
    }
    goal.waypoint.roll.roll = getInput<double>("roll").value_or(0.0);
    goal.waypoint.pinch.pinch = getInput<double>("pinch").value_or(0.0);
    goal.convergence_threshold = 0.05;

    start_ = node_->now();
    handle_.reset();
    result_.reset();
    rejected_ = false;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr h) {
        handle_ = h;
        rejected_ = !h;
    };
    options.result_callback = [this](const GoalHandle::WrappedResult& r) {
        result_ = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
                  r.result->success;
    };
    if (!client_->action_server_is_ready()) {
        spdlog::error("[{}] gripper action server not available", name());
        return BT::NodeStatus::FAILURE;
    }
    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetGripper::onRunning() {
    if (rejected_) {
        spdlog::warn("[{}] goal rejected", name());
        return BT::NodeStatus::FAILURE;
    }
    if (result_) {
        return *result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void SetGripper::onHalted() {
    if (handle_ && !result_) {
        client_->async_cancel_goal(handle_);
    }
    handle_.reset();
}

}  // namespace vortex_bt_nodes::actuators
