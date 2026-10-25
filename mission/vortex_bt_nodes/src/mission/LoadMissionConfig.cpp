#include "vortex_bt_nodes/mission/LoadMissionConfig.hpp"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace vortex_bt_nodes::mission {
namespace {

using BlackboardValues =
    std::vector<std::pair<std::string, std::string>>;

bool flatten_yaml(const YAML::Node& node,
                  const std::string& prefix,
                  BlackboardValues& values,
                  std::string& error) {
    if (node.IsMap()) {
        for (const auto& entry : node) {
            if (!entry.first.IsScalar()) {
                error = "YAML map keys must be scalar values";
                return false;
            }

            const std::string key = entry.first.Scalar();
            if (key.empty()) {
                error = "YAML map keys must not be empty";
                return false;
            }

            const std::string full_key =
                prefix.empty() ? key : prefix + "." + key;

            if (!flatten_yaml(entry.second, full_key, values, error)) {
                return false;
            }
        }

        return true;
    }

    if (node.IsScalar()) {
        values.emplace_back(prefix, node.Scalar());
        return true;
    }

    if (node.IsSequence()) {
        std::string value;

        for (std::size_t index = 0; index < node.size(); ++index) {
            if (!node[index].IsScalar()) {
                error = "YAML sequences may contain only scalar values";
                return false;
            }

            if (index > 0) {
                value += ",";
            }
            value += node[index].Scalar();
        }

        values.emplace_back(prefix, value);
        return true;
    }

    error = "YAML value must be a map, sequence, or scalar";
    return false;
}

}  // namespace

LoadMissionConfig::LoadMissionConfig(const std::string& name,
                                     const BT::NodeConfig& config,
                                     rclcpp::Node::SharedPtr /*node*/)
    : BT::SyncActionNode(name, config) {}

BT::PortsList LoadMissionConfig::providedPorts() {
    return {
        BT::InputPort<std::string>("path", "Path to the mission YAML file")
    };
}

BT::NodeStatus LoadMissionConfig::tick() {
    const auto path = getInput<std::string>("path");
    if (!path || path->empty()) {
        spdlog::error("[{}] Missing or empty path port", name());
        return BT::NodeStatus::FAILURE;
    }

    const auto blackboard = config().blackboard;
    if (!blackboard) {
        spdlog::error("[{}] Blackboard is not available", name());
        return BT::NodeStatus::FAILURE;
    }

    const std::filesystem::path yaml_path(*path);
    std::error_code filesystem_error;
    const bool is_regular_file =
        std::filesystem::is_regular_file(yaml_path, filesystem_error);

    if (filesystem_error) {
        spdlog::error("[{}] Could not inspect config file '{}': {}",
                      name(), *path, filesystem_error.message());
        return BT::NodeStatus::FAILURE;
    }

    if (!is_regular_file) {
        spdlog::error("[{}] Config path is not a regular file: {}",
                      name(), *path);
        return BT::NodeStatus::FAILURE;
    }

    std::ifstream input(yaml_path);
    if (!input) {
        spdlog::error("[{}] Could not open config file: {}", name(), *path);
        return BT::NodeStatus::FAILURE;
    }

    const YAML::Node doc = YAML::Load(input);
    if (!doc.IsMap()) {
        spdlog::error("[{}] YAML document root must be a map: {}",
                      name(), *path);
        return BT::NodeStatus::FAILURE;
    }

    BlackboardValues values;
    std::string error;
    if (!flatten_yaml(doc, "", values, error)) {
        spdlog::error("[{}] Invalid config file '{}': {}",
                      name(), *path, error);
        return BT::NodeStatus::FAILURE;
    }

    for (const auto& [key, value] : values) {
        blackboard->set(key, value);
    }

    spdlog::info("[{}] Loaded mission config from {}", name(), *path);
    return BT::NodeStatus::SUCCESS;
}

}  