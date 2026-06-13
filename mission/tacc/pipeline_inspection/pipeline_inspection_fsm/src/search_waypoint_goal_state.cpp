#include "pipeline_inspection_fsm/states.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

SearchWaypointGoalState::SearchWaypointGoalState(
    const std::string& action_server_name,
    std::vector<vortex::utils::waypoints::WaypointGoal> waypoints)
    : ActionState(action_server_name,
                  std::bind(&SearchWaypointGoalState::create_goal,
                            this,
                            std::placeholders::_1)),
      waypoints_(std::move(waypoints)) {}

pipeline_inspection_fsm::WaypointManagerAction::Goal
SearchWaypointGoalState::create_goal(yasmin::Blackboard::SharedPtr blackboard) {
    pipeline_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.persistent = false;
    goal.convergence_threshold = waypoints_.front().convergence_threshold;

    // Use the snapshotted start pose when available (set by STORE_ORIGIN
    // state).
    if (blackboard->contains("origin_pose")) {
        vortex_msgs::msg::Waypoint wp;
        wp.pose = blackboard->get<geometry_msgs::msg::Pose>("origin_pose");
        wp.waypoint_mode.mode = static_cast<uint8_t>(
            vortex::utils::waypoints::WaypointMode::ONLY_POSITION);
        goal.waypoints = {wp};
        return goal;
    }

    std::vector<vortex_msgs::msg::Waypoint> wp_msgs;
    wp_msgs.reserve(waypoints_.size());
    for (const auto& wp : waypoints_) {
        vortex_msgs::msg::Waypoint msg;
        msg.pose = vortex::utils::ros_conversions::to_pose_msg(wp.pose);
        msg.waypoint_mode.mode = static_cast<uint8_t>(wp.mode);
        msg.keep_altitude = wp.keep_altitude;
        msg.desired_altitude = wp.desired_altitude;
        msg.require_altitude_convergence = wp.require_altitude_convergence;
        wp_msgs.push_back(msg);
    }
    goal.waypoints = wp_msgs;
    return goal;
}
