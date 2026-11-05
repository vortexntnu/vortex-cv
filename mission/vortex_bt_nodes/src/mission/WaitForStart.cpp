#include "vortex_bt_nodes/mission/WaitForStart.hpp"

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
    return {
        BT::InputPort<std::string>(
            "service", "get_operation_mode", "Service name")
    };
}

BT::NodeStatus WaitForStart::onStart() {
    const auto service_name = getInput<std::string>("service");
    if (!service_name || service_name->empty()) {
        spdlog::error("Manglende eller tom port");
        return BT::NodeStatus::FAILURE;
    }

    client_ = node_->create_client<Service>(*service_name);
    future_.reset();
    next_request_ = Clock::now();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForStart::onRunning() {
    const auto now = Clock::now();

    if (future_) {
        const auto status =
            future_->future.wait_for(std::chrono::seconds(0));

        if (status == std::future_status::ready) {
            try {
                const auto response = future_->future.get();
                future_.reset();

                if (!response) {
                    spdlog::warn("Service responsen er tom");
                    next_request_ = Clock::now() + request_interval_;
                    return BT::NodeStatus::RUNNING;
                }

                const bool killswitch_off = !response->killswitch_status;
                const bool autonomous_on =
                    response->current_operation_mode.operation_mode ==
                    vortex_msgs::msg::OperationMode::AUTONOMOUS;

                if (killswitch_off && autonomous_on) {
                    spdlog::info(
                        "Killswitch er av og autonomous er på");
                    return BT::NodeStatus::SUCCESS;
                }

                next_request_ = Clock::now() + request_interval_;
                return BT::NodeStatus::RUNNING;
            } catch (const std::exception& error) {
                future_.reset();
                spdlog::warn("Service request feila");
                next_request_ = Clock::now() + request_interval_;
                return BT::NodeStatus::RUNNING;
            }
        }

        if (now >= request_deadline_) {
            client_->remove_pending_request(*future_);
            future_.reset();
            spdlog::warn("Service request timed out");
            next_request_ = Clock::now() + request_interval_;
        }

        return BT::NodeStatus::RUNNING;
    }

    if (now < next_request_) {
        return BT::NodeStatus::RUNNING;
    }

    if (!client_->service_is_ready()) {
        spdlog::debug("Venter for at service skal bli tilgjengelig");
        next_request_ = now + request_interval_;
        return BT::NodeStatus::RUNNING;
    }

    try {
        auto request = std::make_shared<Service::Request>();
        future_.emplace(client_->async_send_request(request));
        request_deadline_ = Clock::now() + request_timeout_;
        spdlog::debug("request sendt");
    } catch (const std::exception& error) {
        spdlog::warn("Kunne ikke sende request");
        next_request_ = Clock::now() + request_interval_;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForStart::onHalted() {
    if (future_) {
        client_->remove_pending_request(*future_);
        future_.reset();
    }

    spdlog::debug("Halted");
}

}  // namespace vortex_bt_nodes::mission