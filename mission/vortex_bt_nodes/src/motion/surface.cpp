#include "vortex_bt_nodes/motion/surface.hpp"

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::motion {

BT::PortsList Surface::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("z", 0.2, "Depth to stop at [m]")});
}

std::optional<NavAction::Goal> Surface::make_goal() {
    Pose pose;
    pose.position.z = getInput<double>("z").value_or(0.2);
    Goal goal;
    goal.waypoints.push_back(
        make_waypoint(pose, vortex_msgs::msg::WaypointMode::ONLY_Z));
    goal.convergence_threshold = 0.1;
    return goal;
}

}  // namespace vortex_bt_nodes::motion
