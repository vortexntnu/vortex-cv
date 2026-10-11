#include "vortex_bt_nodes/mission/wait.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::mission {

Wait::Wait(const std::string& name,
           const BT::NodeConfig& config,
           rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

BT::PortsList Wait::providedPorts() {
    return {BT::InputPort<double>("seconds", "Time to wait [s]")};
}

BT::NodeStatus Wait::onStart() {
    const auto seconds = getInput<double>("seconds");
    if (!seconds) {
        spdlog::error("[{}] {}", name(), seconds.error());
        return BT::NodeStatus::FAILURE;
    }
    if (*seconds < 0.0) {
        spdlog::error("[{}] seconds is negative: {}", name(), *seconds);
        return BT::NodeStatus::FAILURE;
    }
    end_time_ = node_->now() + rclcpp::Duration::from_seconds(*seconds);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning() {
    return node_->now() >= end_time_ ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::RUNNING;
}

void Wait::onHalted() {
    // Nothing was started, so nothing to stop.
    spdlog::info("[{}] halted", name());
}

}  // namespace vortex_bt_nodes::mission
