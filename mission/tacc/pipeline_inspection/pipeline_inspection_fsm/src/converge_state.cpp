#include "pipeline_inspection_fsm/states.hpp"

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

ConvergeState::ConvergeState(yasmin::Blackboard::SharedPtr blackboard)
    : ActionState(
          blackboard->get<std::string>("action_server.waypoint_manager"),
          std::bind(&ConvergeState::create_goal, this, std::placeholders::_1)),
      convergence_file_path_(
          blackboard->get<std::string>("convergence_file_path")) {
    convergence_goal_ = vortex::utils::waypoints::load_landmark_goal_from_yaml(
        convergence_file_path_, "pipeline_start_convergence");
}

pipeline_inspection_fsm::WaypointManagerAction::Goal ConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "found_landmarks");
    auto landmark = landmarks.front();

    const auto landmark_pose =
        vortex::utils::ros_conversions::ros_pose_to_pose(landmark.pose.pose);
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
