#include "learning_period/states.hpp"

#include <vortex/utils/waypoint_utils.hpp>

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.waypoint_yaml_path =
        node->declare_parameter<std::string>("waypoint_config");

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config) {
    auto bb = std::make_shared<yasmin::Blackboard>();

    // The 4 waypoint identifiers below must match the top-level keys in
    // config/waypoints.yaml. Each is loaded into a WaypointGoal (pose, mode,
    // convergence threshold) and stashed on the blackboard under
    // "<id>_goal" for build_state_machine() to hand to a WaypointGoalState.
    static constexpr const char* kWaypointIds[] = {"waypoint_1", "waypoint_2",
                                                    "waypoint_3", "waypoint_4"};

    for (const auto* id : kWaypointIds) {
        const auto goal = vortex::utils::waypoints::load_waypoint_goal_from_yaml(
            config.waypoint_yaml_path, id);
        bb->set<vortex::utils::waypoints::WaypointGoal>(
            std::string(id) + "_goal", goal);
    }

    return bb;
}
