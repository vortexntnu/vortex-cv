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

    auto bb = std::make_shared<yasmin::Blackboard>();

    bb->set<std::string>("action_server.landmark_convergence", lc_action);
    bb->set<std::string>("action_server.landmark_polling", lp_action);
    bb->set<std::string>("service.start_mission", start_service);

    return bb;
}
