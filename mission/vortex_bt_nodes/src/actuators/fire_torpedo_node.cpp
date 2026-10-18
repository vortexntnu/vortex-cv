#include "vortex_bt_nodes/actuators/fire_torpedo_node.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::actuators {

FireTorpedo::FireTorpedo(const std::string& name,
                         const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList FireTorpedo::providedPorts() {
    return {
        BT::InputPort<double>("settle_s", "5", "Seconds to wait after launch"),
        BT::InputPort<std::string>("side", "left",
                                   "What side to fire (left / right)"),
        BT::InputPort<std::string>("topic", "torpedo_launch",
                                   "What topic to publish the launch")};
}

BT::NodeStatus FireTorpedo::onStart() {
    // Set every member here: the tree starts over at onStart() and must not
    // depend on what the last launch left behind.
    torpedo_launched_ = false;

    const auto settle_s = getInput<double>("settle_s");

    if (!settle_s) {
        spdlog::error("[{}] undefined settle_s: {}", name(), settle_s.error());
        return BT::NodeStatus::FAILURE;
    }
    if (*settle_s < 0.0) {
        spdlog::error("[{}] negative settle_s: {}", name(), *settle_s);
        return BT::NodeStatus::FAILURE;
    }

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

    const auto topic = getInput<std::string>("topic");

    if (!topic) {
        spdlog::error("[{}] undefined topic: {}", name(), topic.error());
        return BT::NodeStatus::FAILURE;
    }
    topic_ = *topic;

    end_time_ = node_->now() + rclcpp::Duration::from_seconds(*settle_s);

    // Made here rather than in the constructor because the topic comes from a
    // port. The publish happens on the next tick, which gives DDS a tick to
    // finish discovery.
    fire_torpedo_pub_ =
        node_->create_publisher<std_msgs::msg::UInt8>(topic_, 10);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FireTorpedo::onRunning() {
    if (!torpedo_launched_) {
        std_msgs::msg::UInt8 msg;
        msg.data = side_;

        fire_torpedo_pub_->publish(msg);
        torpedo_launched_ = true;

        spdlog::info("[{}] launched on side {} (topic {})", name(),
                     static_cast<int>(side_), topic_);
    }
    return node_->now() >= end_time_ ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::RUNNING;
}

void FireTorpedo::onHalted() {
    spdlog::warn("[{}] halted", name());
    // A launch already published cannot be taken back, so there is nothing to
    // undo. The publisher is dropped and made again on the next start.
}

}  // namespace vortex_bt_nodes::actuators
