#include "vortex_bt_nodes/map/set_map_focus.hpp"
#include <spdlog/spdlog.h>
#include <sstream>

namespace vortex_bt_nodes::map {

std::vector<std::string> split_task_list(const std::string& text) {
    std::vector<std::string> out;
    std::stringstream ss(text);
    for (std::string item; std::getline(ss, item, ',');) {
        const auto first = item.find_first_not_of(" \t");
        if (first == std::string::npos) {
            continue;
        }
        const auto last = item.find_last_not_of(" \t");
        out.push_back(item.substr(first, last - first + 1));
    }
    return out;
}

SetMapFocus::SetMapFocus(const std::string& name,
                         const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node,
                         const std::string& service_name)
    : BT::StatefulActionNode(name, config),
      node_(std::move(node)),
      client_(node_->create_client<Srv>(service_name)) {}

BT::PortsList SetMapFocus::providedPorts() {
    return {BT::InputPort<std::string>("tasks", "",
                                       "tasks in focus, comma-separated; empty = all"),
            BT::InputPort<bool>("lock_others", true,
                                "freeze the tasks outside the focus"),
            BT::InputPort<std::string>("commit", "", "tasks to freeze"),
            BT::InputPort<std::string>("uncommit", "", "tasks to release"),
            BT::InputPort<double>("service_timeout_s", 2.0,
                                  "wait this long for landmark_server")};
}

BT::NodeStatus SetMapFocus::onStart() {
    request_ = std::make_shared<Srv::Request>();
    request_->tasks = split_task_list(getInput<std::string>("tasks").value_or(""));
    request_->lock_others = getInput<bool>("lock_others").value_or(true);
    request_->commit = split_task_list(getInput<std::string>("commit").value_or(""));
    request_->uncommit =
        split_task_list(getInput<std::string>("uncommit").value_or(""));
    const double timeout = getInput<double>("service_timeout_s").value_or(2.0);
    deadline_ = node_->now() + rclcpp::Duration::from_seconds(timeout);
    future_.reset();
    return onRunning();
}

BT::NodeStatus SetMapFocus::onRunning() {
    if (!future_) {
        if (!client_->service_is_ready()) {
            if (node_->now() > deadline_) {
                spdlog::warn("[{}] {} is not available", name(),
                             client_->get_service_name());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        auto sent = client_->async_send_request(request_);
        request_id_ = sent.request_id;
        future_ = sent.future.share();
    }
    if (future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        if (node_->now() > deadline_) {
            spdlog::warn("[{}] no answer from {}", name(), client_->get_service_name());
            onHalted();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    const auto response = future_->get();
    future_.reset();
    request_id_.reset();
    if (!response->success) {
        spdlog::warn("[{}] set_focus refused: {}", name(), response->message);
        return BT::NodeStatus::FAILURE;
    }
    spdlog::info("[{}] {}", name(), response->message);
    return BT::NodeStatus::SUCCESS;
}

void SetMapFocus::onHalted() {
    if (request_id_) {
        client_->remove_pending_request(*request_id_);
    }
    future_.reset();
    request_id_.reset();
}

}  // namespace vortex_bt_nodes::map
