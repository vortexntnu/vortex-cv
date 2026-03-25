#ifndef VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_
#define VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_

#include <string>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Converges on a detected landmark with a configurable offset.
 *
 * Reads "found_landmark" (vortex_msgs::msg::Landmark) from the blackboard,
 * applies the convergence offset, and sends the resulting waypoint to the
 * WaypointManager action server.
 */
class ConvergeState : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    ConvergeState(
        const std::string& action_server_name,
        vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal);

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__CONVERGE_STATE_HPP_
