#include "pipeline_inspection_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.bearing_direction_action_server =
        node->declare_parameter<std::string>("action_servers.bearing_direction");
    config.bearing_collection_timeout_sec =
        node->declare_parameter<double>("bearing_collection_timeout_sec", 10.0);
    config.bearing_projection_distance =
        node->declare_parameter<double>("bearing_projection_distance", 10.0);
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.start_pipeline_following_service =
        node->declare_parameter<std::string>(
            "services.start_pipeline_following");
    config.start_end_pipeline_detection_service =
        node->declare_parameter<std::string>(
            "services.start_end_pipeline_detection");
    config.end_of_pipeline_service =
        node->declare_parameter<std::string>("services.end_of_pipeline");
    config.waypoint_yaml_path =
        node->declare_parameter<std::string>("fsm_waypoint_config");
    config.convergence_yaml_path =
        node->declare_parameter<std::string>("pipeline_convergence_config");
    config.start_in_camera_range =
        node->declare_parameter<bool>("start_in_camera_range", false);
    config.start_above_pipe =
        node->declare_parameter<bool>("start_above_pipe", false);

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    if (!config.start_above_pipe && !config.start_in_camera_range) {
        std::vector<vortex::utils::waypoints::WaypointGoal> search_waypoints;
        for (int i = 1;; ++i) {
            try {
                search_waypoints.push_back(
                    vortex::utils::waypoints::load_waypoint_goal_from_yaml(
                        config.waypoint_yaml_path,
                        "search_waypoint_" + std::to_string(i)));
            } catch (const std::runtime_error&) {
                break;
            }
        }
        if (search_waypoints.empty()) {
            throw std::runtime_error("No search waypoints configured");
        }
        bb->set<std::vector<vortex::utils::waypoints::WaypointGoal>>(
            "search_waypoints", search_waypoints);
    }

    if (!config.start_above_pipe) {
        const auto convergence_goal =
            vortex::utils::waypoints::load_waypoint_goal_from_yaml(
                config.convergence_yaml_path, "pipeline_start_convergence");
        bb->set<vortex::utils::waypoints::WaypointGoal>("convergence_goal",
                                                        convergence_goal);
    }

    return bb;
}
