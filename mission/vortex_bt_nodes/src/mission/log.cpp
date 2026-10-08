#include "vortex_bt_nodes/mission/log.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::mission {

BT::PortsList Log::providedPorts() {
    return {BT::InputPort<std::string>("message"),
            BT::InputPort<std::string>("level", "info", "info, warn, error")};
}

BT::NodeStatus Log::tick() {
    const std::string message = getInput<std::string>("message").value_or("");
    const std::string level = getInput<std::string>("level").value_or("info");
    if (level == "error") {
        spdlog::error("[mission] {}", message);
    } else if (level == "warn") {
        spdlog::warn("[mission] {}", message);
    } else {
        spdlog::info("[mission] {}", message);
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::mission
