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
    config.landmark_convergence_action_server =
        node->declare_parameter<std::string>(
            "action_servers.landmark_convergence");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.skip_search = node->declare_parameter<bool>("skip_search", false);
    config.service_request_timeout_sec =
        node->declare_parameter<double>("service_request_timeout_sec", 20.0);
    config.use_service_waypoint =
        node->declare_parameter<bool>("use_service_waypoint");
    config.fallback_waypoint_id = node->declare_parameter<std::string>(
        "fallback_waypoint_id", "fallback_docking_waypoint");
    config.landmark_convergence_goal_id = node->declare_parameter<std::string>(
        "landmark_convergence_goal_id", "power_puck_landmark_convergence");
    config.pre_dock_convergence_goal_id = node->declare_parameter<std::string>(
        "pre_dock_convergence_goal_id", "pre_dock_landmark_convergence");

    if (config.skip_search) {
        spdlog::info("skip_search enabled: search phase will be skipped.");
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

    const auto fallback_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.waypoint_yaml_path, config.fallback_waypoint_id);

    const auto landmark_convergence_goal =
        vortex::utils::waypoints::load_landmark_goal_from_yaml(
            config.landmark_convergence_yaml_path,
            config.landmark_convergence_goal_id);

    const auto pre_dock_convergence_goal =
        vortex::utils::waypoints::load_landmark_goal_from_yaml(
            config.landmark_convergence_yaml_path,
            config.pre_dock_convergence_goal_id);

    bb->set<vortex::utils::waypoints::WaypointGoal>("fallback_waypoint_goal",
                                                    fallback_waypoint_goal);
    bb->set<vortex::utils::waypoints::LandmarkConvergenceGoal>(
        "landmark_convergence_goal", landmark_convergence_goal);
    bb->set<vortex::utils::waypoints::LandmarkConvergenceGoal>(
        "pre_dock_convergence_goal", pre_dock_convergence_goal);

    return bb;
}
