#ifndef VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_
#define VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_

#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using LandmarkConvergenceAction = vortex_msgs::action::LandmarkConvergence;

/**
 * @brief State that sends a LandmarkConvergence action goal with a
 *        fixed convergence offset.
 *
 * @param action_server_name      Name of the LandmarkConvergence action server.
 * @param type                    Landmark type to converge on.
 * @param subtype                 Landmark subtype to converge on.
 * @param convergence_offset      Fixed offset applied to the landmark pose
 *                                (default: identity — converge on the centre).
 * @param convergence_threshold   Distance threshold to declare convergence.
 * @param dead_reckoning_threshold  Distance before switching to dead reckoning.
 * @param track_loss_timeout_sec  Seconds to wait after losing the landmark.
 */
class LandmarkConvergeState
    : public yasmin_ros::ActionState<LandmarkConvergenceAction> {
   public:
    LandmarkConvergeState(
        const std::string& action_server_name,
        vortex_msgs::msg::LandmarkType type,
        vortex_msgs::msg::LandmarkSubtype subtype,
        geometry_msgs::msg::Pose convergence_offset = identity_pose(),
        double convergence_threshold = 0.05,
        double dead_reckoning_threshold = 0.5,
        double track_loss_timeout_sec = 5.0);

    LandmarkConvergenceAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    static geometry_msgs::msg::Pose identity_pose() {
        geometry_msgs::msg::Pose p;
        p.orientation.w = 1.0;
        return p;
    }

   private:
    vortex_msgs::msg::LandmarkType type_;
    vortex_msgs::msg::LandmarkSubtype subtype_;
    geometry_msgs::msg::Pose convergence_offset_;
    double convergence_threshold_;
    double dead_reckoning_threshold_;
    double track_loss_timeout_sec_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_
