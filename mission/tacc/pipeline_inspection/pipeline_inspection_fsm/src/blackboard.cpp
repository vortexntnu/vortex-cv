#include "pipeline_inspection_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.start_pipeline_following_service =
        node->declare_parameter<std::string>(
            "services.start_pipeline_following");
    config.end_of_pipeline_service =
        node->declare_parameter<std::string>("services.end_of_pipeline");
    config.waypoint_yaml_path =
        node->declare_parameter<std::string>("fsm_waypoint_config");
    config.convergence_yaml_path =
        node->declare_parameter<std::string>("pipeline_convergence_config");

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

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

    const auto convergence_goal =
        vortex::utils::waypoints::load_landmark_goal_from_yaml(
            config.convergence_yaml_path, "pipeline_start_convergence");

    bb->set<std::vector<vortex::utils::waypoints::WaypointGoal>>(
        "search_waypoints", search_waypoints);
    bb->set<vortex::utils::waypoints::LandmarkConvergenceGoal>(
        "convergence_goal", convergence_goal);

    return bb;
}
