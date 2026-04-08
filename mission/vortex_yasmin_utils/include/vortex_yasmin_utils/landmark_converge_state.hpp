#ifndef VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_
#define VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_

#include <string>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using LandmarkConvergenceAction = vortex_msgs::action::LandmarkConvergence;

/**
 * @brief Converges on a landmark of a specific type using the LandmarkServer.
 *
 * Sends a LandmarkConvergence action goal to the landmark server,
 * which internally tracks and converges on the target landmark.
 *
 * @param action_server_name  Name of the LandmarkConvergence action server.
 * @param convergence_goal    Offset, mode, and thresholds for convergence.
 * @param type                Landmark type to converge on.
 * @param subtype             Landmark subtype to converge on.
 */
class LandmarkConvergeState
    : public yasmin_ros::ActionState<LandmarkConvergenceAction> {
   public:
    LandmarkConvergeState(
        const std::string& action_server_name,
        vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal,
        vortex_msgs::msg::LandmarkType type,
        vortex_msgs::msg::LandmarkSubtype subtype);

    LandmarkConvergenceAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal_;
    vortex_msgs::msg::LandmarkType type_;
    vortex_msgs::msg::LandmarkSubtype subtype_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__LANDMARK_CONVERGE_STATE_HPP_
