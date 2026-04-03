#include "vortex_yasmin_utils/landmark_converge_state.hpp"

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/landmark.hpp>

namespace vortex_yasmin_utils {

LandmarkConvergeState::LandmarkConvergeState(
    const std::string& action_server_name,
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal,
    vortex_msgs::msg::LandmarkType type,
    vortex_msgs::msg::LandmarkSubtype subtype)
    : ActionState(action_server_name,
                  std::bind(&LandmarkConvergeState::create_goal,
                            this,
                            std::placeholders::_1)),
      convergence_goal_(std::move(convergence_goal)),
      type_(type),
      subtype_(subtype) {}

LandmarkConvergenceAction::Goal LandmarkConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    LandmarkConvergenceAction::Goal goal;

    if (blackboard->contains("landmarks")) {
        auto landmarks =
            blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
                "landmarks");
        if (!landmarks.empty()) {
            goal.type = landmarks.front().type;
            goal.subtype = landmarks.front().subtype;
        } else {
            goal.type = type_;
            goal.subtype = subtype_;
        }
    } else {
        goal.type = type_;
        goal.subtype = subtype_;
    }
    goal.convergence_offset = vortex::utils::ros_conversions::to_pose_msg(
        convergence_goal_.convergence_offset);
    goal.convergence_threshold = convergence_goal_.convergence_threshold;
    goal.dead_reckoning_threshold = convergence_goal_.dead_reckoning_threshold;
    goal.track_loss_timeout_sec = convergence_goal_.track_loss_timeout_sec;
    goal.convergence_mode.mode = static_cast<uint8_t>(convergence_goal_.mode);

    return goal;
}

}  // namespace vortex_yasmin_utils
