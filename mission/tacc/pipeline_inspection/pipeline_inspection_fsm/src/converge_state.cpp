#include "pipeline_inspection_fsm/states.hpp"

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

LandmarkConvergeState::LandmarkConvergeState(
    const std::string& action_server_name,
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal)
    : ActionState(action_server_name,
                  std::bind(&LandmarkConvergeState::create_goal,
                            this,
                            std::placeholders::_1)),
      convergence_goal_(std::move(convergence_goal)) {}

pipeline_inspection_fsm::WaypointManagerAction::Goal
LandmarkConvergeState::create_goal(yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "pipeline_landmarks");

    const auto landmark_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const auto target_pose = vortex::utils::waypoints::apply_pose_offset(
        landmark_pose, convergence_goal_.convergence_offset);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode.mode = static_cast<uint8_t>(convergence_goal_.mode);

    pipeline_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = convergence_goal_.convergence_threshold;

    return goal;
}
