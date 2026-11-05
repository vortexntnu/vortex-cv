#include "vortex_bt_nodes/mission/wait_for_start.hpp"

#include <spdlog/spdlog.h>

#include <exception>
#include <future>
#include <memory>
#include <utility>

namespace vortex_bt_nodes::mission {

WaitForStart::WaitForStart(const std::string& name,
                           const BT::NodeConfig& config,
                           rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

BT::PortsList WaitForStart::providedPorts() {
    return {BT::InputPort<std::string>("service", "get_operation_mode",
                                       "Service name")};
}

BT::NodeStatus WaitForStart::onStart() {
    const auto service_name = getInput<std::string>("service");
    if (!service_name || service_name->empty()) {
        spdlog::error("[{}] Missing or empty port", name());
        return BT::NodeStatus::FAILURE;
    }

    client_ = node_->create_client<Service>(*service_name);
    future_.reset();
    next_request_ = Clock::now();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForStart::onRunning() {
    const auto now = Clock::now();

    if (future_ && future_->future.wait_for(std::chrono::seconds(0)) ==
                       std::future_status::ready) {
        const auto response = future_->future.get();
        future_.reset();

        if (!response->killswitch_status &&
            response->current_operation_mode.operation_mode ==
                vortex_msgs::msg::OperationMode::AUTONOMOUS) {
            spdlog::info("[{}] Autonomous mode enabled", name());
            return BT::NodeStatus::SUCCESS;
        }

        next_request_ = now + request_interval_;
    }

    if (!future_ && now >= next_request_) {
        if (client_->service_is_ready()) {
            auto request = std::make_shared<Service::Request>();
            future_.emplace(client_->async_send_request(request));

            spdlog::info("[{}] Service request sent", name());
        } else {
            spdlog::warn("[{}] Service is not available", name());
        }

        next_request_ = now + request_interval_;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForStart::onHalted() {
    if (future_) {
        client_->remove_pending_request(*future_);
        future_.reset();
    }

    spdlog::debug("[{}] Halted", name());
}

}  // namespace vortex_bt_nodes::mission
