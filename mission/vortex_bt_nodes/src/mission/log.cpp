#include "vortex_bt_nodes/mission/log.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::mission {

Log::Log(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList Log::providedPorts() {
    return {
        BT::InputPort<std::string>("message", "Message to log"),
        BT::InputPort<std::string>("level", "Log level: info, warn or error")};
}

BT::NodeStatus Log::tick() {
    const auto message = getInput<std::string>("message");
    if (!message) {
        spdlog::error("[{}] {}", name(), message.error());
        return BT::NodeStatus::FAILURE;
    }

    const auto level = getInput<std::string>("level");
    if (!level) {
        spdlog::error("[{}] {}", name(), level.error());
        return BT::NodeStatus::FAILURE;
    }

    if (*level == "info") {
        spdlog::info("[{}] {}", name(), *message);
    } else if (*level == "warn") {
        spdlog::warn("[{}] {}", name(), *message);
    } else if (*level == "error") {
        spdlog::error("[{}] {}", name(), *message);
    } else {
        spdlog::error("[{}] unknown log level: {}", name(), *level);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::mission