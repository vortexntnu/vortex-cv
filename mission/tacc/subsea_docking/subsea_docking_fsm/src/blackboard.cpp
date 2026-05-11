#include <spdlog/spdlog.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "subsea_docking_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_yaml_path =
        node->declare_parameter<std::string>("fsm_waypoint_config");
    config.landmark_convergence_yaml_path =
        node->declare_parameter<std::string>("landmark_convergence_config");
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");

    config.use_wall_detection =
        node->declare_parameter<bool>("use_wall_detection");

    config.service_request_timeout_sec =
        node->declare_parameter<double>("service_request_timeout_sec");
    config.use_service_waypoint =
        node->declare_parameter<bool>("use_service_waypoint");
    config.docking_estimator_start_service =
        node->declare_parameter<std::string>("docking_estimator_start_service");

    if (!config.use_wall_detection) {
        spdlog::info(
            "use_wall_detection disabled: going directly to dock config "
            "waypoint.");
    } else if (config.use_service_waypoint) {
        config.docking_position_service =
            node->declare_parameter<std::string>("docking_position_service");
        spdlog::info(
            "Search enabled with service-based waypoint input. Waiting for "
            "pose request concurrently with landmark polling.");
    } else {
        spdlog::info(
            "Using docking position estimate directly from yaml config.");
    }

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    const auto dock_config_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.waypoint_yaml_path, "dock_config_waypoint");

    const auto power_puck_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, "power_puck_waypoint");

    const auto above_dock_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, "above_dock_waypoint");

    bb->set<vortex::utils::waypoints::WaypointGoal>("dock_config_waypoint_goal",
                                                    dock_config_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("power_puck_waypoint_goal",
                                                    power_puck_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("above_dock_waypoint_goal",
                                                    above_dock_waypoint_goal);

    return bb;
}
