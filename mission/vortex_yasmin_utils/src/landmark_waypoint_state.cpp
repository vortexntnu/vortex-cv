#include "vortex_yasmin_utils/landmark_waypoint_state.hpp"

#include <stdexcept>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_yasmin_utils {

LandmarkWaypointState::LandmarkWaypointState(
    const std::string& action_server_name,
    vortex::utils::waypoints::WaypointGoal waypoint_goal,
    const std::string& landmarks_bb_key)
    : ActionState(action_server_name,
                  std::bind(&LandmarkWaypointState::create_goal,
                            this,
                            std::placeholders::_1)),
      waypoint_goal_(std::move(waypoint_goal)),
      landmarks_bb_key_(landmarks_bb_key) {}

WaypointManagerAction::Goal LandmarkWaypointState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    if (!blackboard->contains(landmarks_bb_key_)) {
        throw std::runtime_error("LandmarkWaypointState: blackboard key '" +
                                 landmarks_bb_key_ + "' not found");
    }

    auto landmarks = blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
        landmarks_bb_key_);

    if (landmarks.empty()) {
        throw std::runtime_error(
            "LandmarkWaypointState: landmark vector is empty");
    }

    auto landmark_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    auto target_pose = vortex::utils::waypoints::apply_pose_offset(
        landmark_pose, waypoint_goal_.pose);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode =
        vortex::utils::waypoints::waypoint_mode_to_ros(waypoint_goal_.mode);

    WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = waypoint_goal_.convergence_threshold;

    return goal;
}

}  // namespace vortex_yasmin_utils
