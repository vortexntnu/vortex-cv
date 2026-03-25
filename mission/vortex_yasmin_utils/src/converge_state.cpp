#include "vortex_yasmin_utils/converge_state.hpp"

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_yasmin_utils {

ConvergeState::ConvergeState(
    const std::string& action_server_name,
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal)
    : ActionState(action_server_name,
                  std::bind(&ConvergeState::create_goal,
                            this,
                            std::placeholders::_1)),
      convergence_goal_(std::move(convergence_goal)) {}

WaypointManagerAction::Goal ConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto landmark =
        blackboard->get<vortex_msgs::msg::Landmark>("found_landmark");

    const auto landmark_pose =
        vortex::utils::ros_conversions::ros_pose_to_pose(landmark.pose.pose);
    const auto target_pose = vortex::utils::waypoints::apply_pose_offset(
        landmark_pose, convergence_goal_.convergence_offset);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode.mode = static_cast<uint8_t>(convergence_goal_.mode);

    WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = convergence_goal_.convergence_threshold;

    return goal;
}

}  // namespace vortex_yasmin_utils
