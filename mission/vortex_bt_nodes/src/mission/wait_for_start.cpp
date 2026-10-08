#include "vortex_bt_nodes/mission/wait_for_start.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

#include <vortex_msgs/msg/operation_mode.hpp>

namespace vortex_bt_nodes::mission {

namespace {
constexpr double kAskPeriodS = 0.5;
}  // namespace

BT::PortsList WaitForStart::providedPorts() {
    return {BT::InputPort<std::string>("service", "get_operation_mode",
                                       "GetOperationMode service")};
}

BT::NodeStatus WaitForStart::onStart() {
    if (!client_) {
        client_ = node_->create_client<Srv>(
            getInput<std::string>("service").value_or("get_operation_mode"));
    }
    pending_.reset();
    last_ask_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    logged_ = false;
    return onRunning();
}

BT::NodeStatus WaitForStart::onRunning() {
    if (pending_ && pending_->future.wait_for(std::chrono::seconds(0)) ==
                        std::future_status::ready) {
        const auto response = pending_->future.get();
        pending_.reset();
        // Not response->success: the manager leaves it false on a get.
        if (!response->killswitch_status &&
            response->current_operation_mode.operation_mode ==
                vortex_msgs::msg::OperationMode::AUTONOMOUS) {
            spdlog::info("[{}] killswitch off, autonomous: starting", name());
            return BT::NodeStatus::SUCCESS;
        }
        if (!logged_) {
            spdlog::info("[{}] waiting for killswitch off and autonomous mode",
                         name());
            logged_ = true;
        }
    }
    const rclcpp::Time now = node_->now();
    if (!pending_ && (now - last_ask_).seconds() > kAskPeriodS &&
        client_->service_is_ready()) {
        pending_ =
            client_->async_send_request(std::make_shared<Srv::Request>());
        last_ask_ = now;
    }
    return BT::NodeStatus::RUNNING;
}

}  // namespace vortex_bt_nodes::mission
