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
    config.gripper_frame =
        node->declare_parameter<std::string>("frames.gripper_frame");
    config.base_frame =
        node->declare_parameter<std::string>("frames.base_frame");
    config.convergence_threshold =
        node->declare_parameter<double>("convergence_threshold", 0.05);
    config.dead_reckoning_threshold =
        node->declare_parameter<double>("dead_reckoning_threshold", 0.5);
    config.track_loss_timeout_sec =
        node->declare_parameter<double>("track_loss_timeout_sec", 5.0);

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& /*config*/) {
    return std::make_shared<yasmin::Blackboard>();
}
