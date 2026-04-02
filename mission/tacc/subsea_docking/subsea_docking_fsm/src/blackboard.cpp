#include <spdlog/spdlog.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "subsea_docking_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_yaml_path =
        node->declare_parameter<std::string>("fsm_waypoint_config");
    config.start_mission_service =
        node->declare_parameter<std::string>("topics.start_mission");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.skip_search = node->declare_parameter<bool>("skip_search");
    config.use_service_waypoint =
        node->declare_parameter<bool>("use_service_waypoint");

    if (config.skip_search) {
        spdlog::info(
            "Start in range of docking station, will route directly to "
            "LANDMARK_POLLING");
    } else if (config.use_service_waypoint) {
        config.docking_position_service =
            node->declare_parameter<std::string>("docking_position_service");
        spdlog::info(
            "Search enabled with service-based waypoint input. Will wait for "
            "docking station position to be estimated.");
    } else {
        spdlog::info(
            "Using docking position estimate directly from yaml config.");
    }

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    if (!config.skip_search && !config.use_service_waypoint) {
        const auto waypoint_goal =
            vortex::utils::waypoints::load_waypoint_goal_from_yaml(
                config.waypoint_yaml_path, "initial_docking_position_guess");
        bb->set<vortex::utils::waypoints::WaypointGoal>("search_waypoint_goal",
                                                        waypoint_goal);
    }

    return bb;
}
