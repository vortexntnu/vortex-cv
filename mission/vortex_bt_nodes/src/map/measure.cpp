#include "vortex_bt_nodes/map/measure.hpp"

#include <spdlog/spdlog.h>

#include <utility>

namespace vortex_bt_nodes::map {

Measure::Measure(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(std::move(node)) {
    client_ =
        rclcpp_action::create_client<Action>(node_, "landmark_server/measure");
}

BT::PortsList Measure::providedPorts() {
    return {BT::InputPort<std::string>("class_name"),
            BT::InputPort<std::string>("out_frame", "", ""),
            BT::InputPort<double>("duration_s", 0.0, "0 = server default"),
            BT::InputPort<double>("max_range_m", 0.0, "0 = server default"),
            BT::InputPort<std::string>("fallback_from", "", ""),
            BT::InputPort<bool>("accept_fallback", true, "")};
}

BT::NodeStatus Measure::onStart() {
    const auto cls = getInput<std::string>("class_name");
    if (!cls || !client_->action_server_is_ready()) {
        spdlog::error("[{}] no class_name or no measure server", name());
        return BT::NodeStatus::FAILURE;
    }
    Action::Goal goal;
    goal.class_name = *cls;
    goal.out_frame = getInput<std::string>("out_frame").value_or("");
    goal.duration_s = getInput<double>("duration_s").value_or(0.0);
    goal.max_range_m = getInput<double>("max_range_m").value_or(0.0);
    goal.fallback_from = getInput<std::string>("fallback_from").value_or("");
    result_.reset();
    handle_.reset();
    auto options = rclcpp_action::Client<Action>::SendGoalOptions();
    options.goal_response_callback = [this](Handle::SharedPtr h) {
        handle_ = h;
        if (!h) {
            rejected_ = true;
        }
    };
    options.result_callback = [this](const Handle::WrappedResult& r) {
        result_ = r;
    };
    rejected_ = false;
    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Measure::onRunning() {
    if (rejected_) {
        spdlog::warn("[{}] measure rejected", name());
        return BT::NodeStatus::FAILURE;
    }
    if (!result_) {
        return BT::NodeStatus::RUNNING;
    }
    const auto& r = *result_->result;
    spdlog::info("[{}] {}: {}{}", name(),
                 getInput<std::string>("class_name").value_or(""),
                 r.success ? "measured, " : "failed, ", r.message);
    const bool fallback_ok =
        r.used_fallback && getInput<bool>("accept_fallback").value_or(true);
    return r.success || fallback_ok ? BT::NodeStatus::SUCCESS
                                    : BT::NodeStatus::FAILURE;
}

void Measure::onHalted() {
    if (handle_) {
        client_->async_cancel_goal(handle_);
    }
}

}  // namespace vortex_bt_nodes::map
