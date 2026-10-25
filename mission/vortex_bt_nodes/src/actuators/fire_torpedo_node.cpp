#include "vortex_bt_nodes/actuators/fire_torpedo_node.hpp"

#include <spdlog/spdlog.h>
#include <chrono>

namespace vortex_bt_nodes::actuators {

// k... are placeholders for the config
constexpr char* kFireTorpedoService = "torpedo/launch";

FireTorpedo::FireTorpedo(const std::string& name,
                         const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node)
    : BT::StatefulActionNode(name, config), node_(node) {
    fire_torpedo_cli_ = node_->create_client<Srv>(kFireTorpedoService);
}

BT::PortsList FireTorpedo::providedPorts() {
    return {
        BT::InputPort<std::string>("side", "undefined", "left or right"),
    };
}

BT::NodeStatus FireTorpedo::onStart() {
    torpedo_launched_ = false;
    torpedo_request_sent_ = false;
    future_ = {};
    req_ID_.reset();

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
    if (!torpedo_request_sent_) {
        if (!fire_torpedo_cli_->service_is_ready()) {
            return BT::NodeStatus::RUNNING;
        }

        auto request = std::make_shared<Srv::Request>();
        request->side = side_;

        auto pending = fire_torpedo_cli_->async_send_request(request);

        future_ = pending.future.share();
        req_ID_ = pending.request_id;

        torpedo_request_sent_ = true;

        spdlog::info("[{}] request for torpedo launch on side {} ", name(),
                     side_ ? "right" : "left");
        return BT::NodeStatus::RUNNING;
    }
    if (!future_.valid() || future_.wait_for(std::chrono::seconds(0)) !=
                                std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    const auto response = future_.get();

    if (!response->success) {
        spdlog::error("[{}] launch failed: {} ", name(), response->message);
        return BT::NodeStatus::FAILURE;
    }

    torpedo_launched_ = true;
    spdlog::info("[{}] launched on side {}", name(), static_cast<int>(side_));
    return BT::NodeStatus::SUCCESS;
}

void FireTorpedo::onHalted() {
    spdlog::warn("[{}] halted", name());
    if (torpedo_request_sent_ && req_ID_) {
        fire_torpedo_cli_->remove_pending_request(*req_ID_);
        spdlog::warn("[{}] Request sent, no answer received", name());
    }
    if (!future_.valid()) {
        spdlog::error("[{}] no FireTorpedo service on {}", name(),
                      kFireTorpedoService);
    }
}

}  // namespace vortex_bt_nodes::actuators
