#ifndef VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_
#define VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_

#include <string>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Converges on the first landmark in a vector read from the blackboard.
 *
 * Reads a std::vector<vortex_msgs::msg::Landmark> from the blackboard
 * under the caller-specified key, takes the first element, applies the
 * convergence offset, and sends the resulting waypoint to the
 * WaypointManager action server.
 *
 * @param action_server_name  Name of the WaypointManager action server.
 * @param convergence_goal    Offset, mode, and threshold for convergence.
 * @param landmarks_bb_key    Blackboard key for the input landmarks vector.
 */
class ConvergeState : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    ConvergeState(
        const std::string& action_server_name,
        vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal,
        const std::string& landmarks_bb_key);

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal_;
    std::string landmarks_bb_key_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_
