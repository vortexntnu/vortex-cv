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
    config.use_camera_direction =
        node->declare_parameter<bool>("use_camera_direction");
    config.start_in_range =
        node->declare_parameter<bool>("start_in_range");

    config.wall_detection_estimate_timeout_sec =
        node->declare_parameter<double>("wall_detection_estimate_timeout_sec");
    config.camera_direction_timeout_sec =
        node->declare_parameter<double>("camera_direction_timeout_sec");
    config.wait_before_fallback_sec =
        node->declare_parameter<double>("wait_before_fallback_sec");
    config.use_service_waypoint =
        node->declare_parameter<bool>("use_service_waypoint");
    config.docking_estimator_start_service =
        node->declare_parameter<std::string>("docking_estimator_start_service");

    if (config.start_in_range) {
        spdlog::info(
            "start_in_range enabled: skipping all estimation, polling directly "
            "for ArUco detection.");
    }

    if (config.use_service_waypoint) {
        spdlog::info(
            "use_service_waypoint enabled: skipping all estimation, navigating "
            "directly to dock config waypoint.");
    }

    if (config.use_camera_direction) {
        spdlog::info(
            "Camera direction estimation enabled: will poll for YOLO direction "
            "landmark (timeout: {:.1f}s).",
            config.camera_direction_timeout_sec);
    }

    if (config.use_wall_detection) {
        config.docking_position_service =
            node->declare_parameter<std::string>("docking_position_service");
        spdlog::info(
            "Wall detection enabled: waiting for sonar pose estimate "
            "(timeout: {:.1f}s).",
            config.wall_detection_estimate_timeout_sec);
    }

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    const auto depth_level_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.waypoint_yaml_path, "depth_level_waypoint");

    const auto dock_config_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.waypoint_yaml_path, "dock_config_waypoint");

    const auto power_puck_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, "power_puck_waypoint");

    const auto above_dock_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, "above_dock_waypoint");

    const auto camera_direction_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, "camera_direction_waypoint");

    bb->set<vortex::utils::waypoints::WaypointGoal>("depth_level_waypoint_goal",
                                                    depth_level_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("dock_config_waypoint_goal",
                                                    dock_config_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("power_puck_waypoint_goal",
                                                    power_puck_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("above_dock_waypoint_goal",
                                                    above_dock_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>(
        "camera_direction_waypoint_goal", camera_direction_waypoint_goal);

    return bb;
}
