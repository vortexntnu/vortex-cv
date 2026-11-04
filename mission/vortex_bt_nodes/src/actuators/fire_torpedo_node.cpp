#include "vortex_bt_nodes/actuators/fire_torpedo_node.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

namespace vortex_bt_nodes::actuators {

FireTorpedo::FireTorpedo(const std::string& name,
                         const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList FireTorpedo::providedPorts() {
    return {BT::InputPort<std::string>("service", "fire_torpedo", ""),
            BT::InputPort<double>("settle_s", 2.0, "Wait after the shot [s]"),
            BT::InputPort<double>("timeout_s", 5.0, "Wait for the service [s]")};
}

BT::NodeStatus FireTorpedo::onStart() {
    const auto service = getInput<std::string>("service");
    const auto settle_s = getInput<double>("settle_s");
    const auto timeout_s = getInput<double>("timeout_s");
    if (!service || !settle_s || !timeout_s || *settle_s < 0.0) {
        spdlog::error("[{}] bad ports", name());
        return BT::NodeStatus::FAILURE;
    }
    settle_s_ = *settle_s;
    give_up_at_ = node_->now() + rclcpp::Duration::from_seconds(*timeout_s);
    end_time_.reset();
    pending_.reset();
    client_ = node_->create_client<Srv>(*service);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FireTorpedo::onRunning() {
    if (end_time_) {
        return node_->now() >= *end_time_ ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::RUNNING;
    }
    if (!pending_) {
        if (client_->service_is_ready()) {
            pending_ = client_->async_send_request(
                std::make_shared<Srv::Request>(),
                [](rclcpp::Client<Srv>::SharedFuture) {});
        } else if (node_->now() >= give_up_at_) {
            spdlog::error("[{}] no service {}", name(),
                          client_->get_service_name());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    if (pending_->future.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }
    const auto response = pending_->future.get();
    if (!response->success) {
        spdlog::error("[{}] {}", name(), response->message);
        return BT::NodeStatus::FAILURE;
    }
    spdlog::info("[{}] fired", name());
    end_time_ = node_->now() + rclcpp::Duration::from_seconds(settle_s_);
    return BT::NodeStatus::RUNNING;
}

void FireTorpedo::onHalted() {
    pending_.reset();
    client_.reset();
}

}  // namespace vortex_bt_nodes::actuators
