#include "vortex_yasmin_utils/bearing_waypoint_state.hpp"

#include <stdexcept>

#include <geometry_msgs/msg/pose.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <yasmin_ros/ros_logs.hpp>

namespace vortex_yasmin_utils {

BearingWaypointState::BearingWaypointState(
    const std::string& action_server_name,
    double convergence_threshold,
    double desired_altitude,
    const std::string& pose_bb_key)
    : ActionState(action_server_name,
                  std::bind(&BearingWaypointState::create_goal,
                            this,
                            std::placeholders::_1)),
      convergence_threshold_(convergence_threshold),
      desired_altitude_(desired_altitude),
      pose_bb_key_(pose_bb_key) {}

WaypointManagerAction::Goal BearingWaypointState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    if (!blackboard->contains(pose_bb_key_)) {
        throw std::runtime_error(
            "BearingWaypointState: blackboard key '" + pose_bb_key_ +
            "' not found — CollectBearingDirectionState must run first");
    }

    const auto& pose =
        blackboard->get<geometry_msgs::msg::Pose>(pose_bb_key_);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = pose;
    if (desired_altitude_ >= 0.0) {
        wp.pose.position.z = desired_altitude_;
    }
    wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::FORWARD_HEADING;

    WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = convergence_threshold_;

    YASMIN_LOG_INFO(
        "BearingWaypointState: navigating to bearing waypoint (%.3f, %.3f, %.3f), "
        "altitude=%.1fm",
        pose.position.x, pose.position.y, pose.position.z, desired_altitude_);
    return goal;
}

}  // namespace vortex_yasmin_utils
