#include <spdlog/spdlog.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "valve_inspection_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.landmark_convergence_yaml_path =
        node->declare_parameter<std::string>("landmark_convergence_config");
    config.standoff_waypoint_goal_id =
        node->declare_parameter<std::string>("standoff_waypoint_goal_id");
    config.tcp_offset_goal_id =
        node->declare_parameter<std::string>("tcp_offset_goal_id");
    config.vertical_mounted_valve =
        node->declare_parameter<bool>("vertical_mounted_valve");

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    const auto standoff_waypoint_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path,
            config.standoff_waypoint_goal_id);

    const auto tcp_offset_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, config.tcp_offset_goal_id);

    bb->set<vortex::utils::waypoints::WaypointGoal>("standoff_waypoint_goal",
                                                    standoff_waypoint_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("tcp_offset_goal",
                                                    tcp_offset_goal);

    return bb;
}
