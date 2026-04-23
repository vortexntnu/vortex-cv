#include "visual_inspection_fsm/states.hpp"

#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif

#include <yaml-cpp/yaml.h>

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
    config.tcp_base_frame = node->declare_parameter<std::string>(
        "tcp_base_frame", "nautilus/base_link");
    config.tcp_tip_frame =
        node->declare_parameter<std::string>("tcp_tip_frame", "nautilus/arm");

    // Read valve_z_offset from the convergence YAML (under tcp_offset section).
    try {
        const YAML::Node yaml =
            YAML::LoadFile(config.landmark_convergence_yaml_path);
        if (yaml[config.tcp_offset_goal_id] &&
            yaml[config.tcp_offset_goal_id]["valve_z_offset"]) {
            config.valve_z_offset =
                yaml[config.tcp_offset_goal_id]["valve_z_offset"].as<double>();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(),
                    "Could not read valve_z_offset from config: %s", e.what());
    }

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    rclcpp::Node::SharedPtr node,
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    const auto standoff_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path,
            config.standoff_waypoint_goal_id);

    const auto tcp_offset_goal =
        vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.landmark_convergence_yaml_path, config.tcp_offset_goal_id);

    bb->set<vortex::utils::waypoints::WaypointGoal>("standoff_goal",
                                                    standoff_goal);
    bb->set<vortex::utils::waypoints::WaypointGoal>("tcp_offset_goal",
                                                    tcp_offset_goal);

    // Create TF buffer and listener so states can look up transforms.
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

    bb->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
    bb->set<std::shared_ptr<tf2_ros::TransformListener>>("tf_listener",
                                                         tf_listener);

    return bb;
}
