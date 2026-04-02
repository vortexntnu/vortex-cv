#ifndef VORTEX_YASMIN_UTILS__WAYPOINT_GOAL_STATE_HPP_
#define VORTEX_YASMIN_UTILS__WAYPOINT_GOAL_STATE_HPP_

#include <string>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Sends a single waypoint to the WaypointManager action server.
 *
 * Accepts a WaypointGoal directly (e.g. from load_waypoint_goal_from_yaml),
 * sends it as a one-shot action (persistent = false), and returns based on
 * the action result. Feedback is ignored.
 *
 * All configuration is stored as member variables — no blackboard reads or
 * writes — so instances are safe to use inside a yasmin concurrent container.
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name  Name of the WaypointManager action server.
 * @param waypoint_goal       Waypoint to send, including pose, mode, and
 *                            convergence threshold.
 */
class WaypointGoalState
    : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    WaypointGoalState(const std::string& action_server_name,
                      vortex::utils::waypoints::WaypointGoal waypoint_goal);

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal waypoint_goal_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__WAYPOINT_GOAL_STATE_HPP_
