#include "vortex_bt_nodes/actuators/fire_torpedo_node.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::actuators {

FireTorpedo::FireTorpedo(const std::string& name,
                         const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList FireTorpedo::providedPorts() {
    return {
        BT::InputPort<std::string>("side", "left",
                                   "What side to fire (left / right)"),
    };
}

BT::NodeStatus FireTorpedo::onStart() {
    torpedo_launched_ = false;

    const auto side = getInput<std::string>("side");

    if (!side) {
        spdlog::error("[{}] undefined side: {}", name(), side.error());
        return BT::NodeStatus::FAILURE;
    }
    if (*side == "left") {
        side_ = 0;
    } else if (*side == "right") {
        side_ = 1;
    } else {
        spdlog::error("[{}] unknown side: {}", name(), *side);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FireTorpedo::onRunning() {
    if (!torpedo_launched_) {
        spdlog::info("[{}] launched on side {}", name(),
                     static_cast<int>(side_));
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
}

void FireTorpedo::onHalted() {
    spdlog::warn("[{}] halted", name());
    // A launch already published cannot be taken back, so there is nothing to
    // undo. The publisher is dropped and made again on the next start.
}

}  // namespace vortex_bt_nodes::actuators
