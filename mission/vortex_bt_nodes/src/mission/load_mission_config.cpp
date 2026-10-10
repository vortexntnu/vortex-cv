#include "vortex_bt_nodes/mission/load_mission_config.hpp"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <functional>

namespace vortex_bt_nodes::mission {

BT::PortsList LoadMissionConfig::providedPorts() {
    return {BT::InputPort<std::string>("path", "mission.yaml")};
}

BT::NodeStatus LoadMissionConfig::tick() {
    const auto path = getInput<std::string>("path");
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.value_or(""));
    } catch (const std::exception& e) {
        spdlog::error("[{}] can't read '{}': {}", name(), path.value_or(""),
                      e.what());
        return BT::NodeStatus::FAILURE;
    }
    int count = 0;
    std::function<void(const YAML::Node&, const std::string&)> load =
        [&](const YAML::Node& node, const std::string& prefix) {
            for (const auto& entry : node) {
                const std::string key = prefix + entry.first.as<std::string>();
                if (entry.second.IsMap()) {
                    load(entry.second, key + ".");
                } else if (entry.second.IsScalar()) {
                    config().blackboard->set(key,
                                             entry.second.as<std::string>());
                    ++count;
                }
            }
        };
    load(root, "");
    spdlog::info("[{}] {} values from {}", name(), count, *path);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::mission
