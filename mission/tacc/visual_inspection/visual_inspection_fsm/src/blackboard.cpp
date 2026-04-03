#include "valve_inspection_fsm/states.hpp"

#include <rclcpp/rclcpp.hpp>

std::shared_ptr<yasmin::Blackboard> initialize_blackboard() {
    auto params = rclcpp::Node::make_shared("valve_inspection_params");

    const std::string lc_action = params->declare_parameter<std::string>(
        "action_servers.landmark_convergence");
    const std::string lp_action = params->declare_parameter<std::string>(
        "action_servers.landmark_polling");
    const std::string start_service =
        params->declare_parameter<std::string>("services.start_mission");
    const std::string gripper_frame =
        params->declare_parameter<std::string>("frames.gripper_frame");
    const std::string base_frame =
        params->declare_parameter<std::string>("frames.base_frame");

    auto bb = std::make_shared<yasmin::Blackboard>();

    bb->set<std::string>("action_server.landmark_convergence", lc_action);
    bb->set<std::string>("action_server.landmark_polling", lp_action);
    bb->set<std::string>("service.start_mission", start_service);
    bb->set<std::string>("frames.gripper_frame", gripper_frame);
    bb->set<std::string>("frames.base_frame", base_frame);

    return bb;
}
