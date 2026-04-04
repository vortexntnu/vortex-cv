#include <spdlog/spdlog.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "valve_inspection_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.landmark_convergence_action_server =
        node->declare_parameter<std::string>(
            "action_servers.landmark_convergence");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.landmark_convergence_yaml_path =
        node->declare_parameter<std::string>("landmark_convergence_config");
    config.landmark_convergence_goal_id = node->declare_parameter<std::string>(
        "landmark_convergence_goal_id", "visual_inspection_convergence");
    config.vertical_mounted_valve =
        node->declare_parameter<bool>("vertical_mounted_valve");

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    const auto landmark_convergence_goal =
        vortex::utils::waypoints::load_landmark_goal_from_yaml(
            config.landmark_convergence_yaml_path,
            config.landmark_convergence_goal_id);

    bb->set<vortex::utils::waypoints::LandmarkConvergenceGoal>(
        "landmark_convergence_goal", landmark_convergence_goal);

    return bb;
}
