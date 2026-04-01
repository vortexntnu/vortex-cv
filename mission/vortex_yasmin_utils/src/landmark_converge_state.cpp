#include "vortex_yasmin_utils/landmark_converge_state.hpp"

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_yasmin_utils {

LandmarkConvergeState::LandmarkConvergeState(
    const std::string& action_server_name,
    vortex_msgs::msg::LandmarkType type,
    vortex_msgs::msg::LandmarkSubtype subtype,
    geometry_msgs::msg::Pose convergence_offset,
    double convergence_threshold,
    double dead_reckoning_threshold,
    double track_loss_timeout_sec)
    : ActionState(action_server_name,
                  std::bind(&LandmarkConvergeState::create_goal,
                            this,
                            std::placeholders::_1)),
      type_(type),
      subtype_(subtype),
      convergence_offset_(convergence_offset),
      convergence_threshold_(convergence_threshold),
      dead_reckoning_threshold_(dead_reckoning_threshold),
      track_loss_timeout_sec_(track_loss_timeout_sec) {}

LandmarkConvergenceAction::Goal LandmarkConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    LandmarkConvergenceAction::Goal goal;
    goal.type = type_;
    goal.subtype = subtype_;
    goal.convergence_offset = convergence_offset_;
    goal.convergence_threshold = convergence_threshold_;
    goal.dead_reckoning_threshold = dead_reckoning_threshold_;
    goal.track_loss_timeout_sec = track_loss_timeout_sec_;
    goal.convergence_mode.mode = vortex_msgs::msg::WaypointMode::FULL_POSE;
    return goal;
}

}  // namespace vortex_yasmin_utils
