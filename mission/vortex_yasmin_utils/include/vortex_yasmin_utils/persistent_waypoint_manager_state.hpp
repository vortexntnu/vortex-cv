#ifndef VORTEX_YASMIN_UTILS__PERSISTENT_WAYPOINT_MANAGER_STATE_HPP_
#define VORTEX_YASMIN_UTILS__PERSISTENT_WAYPOINT_MANAGER_STATE_HPP_

#include <string>

#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Starts the WaypointManager action server in persistent mode.
 *
 * Sends an empty waypoint list with persistent = true, so the action server
 * stays alive and accepts waypoints from other sources. Does not send any
 * waypoints itself.
 *
 * All configuration is stored as member variables — no blackboard reads or
 * writes — so instances are safe to use inside a yasmin concurrent container.
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name    Name of the WaypointManager action server.
 * @param convergence_threshold Convergence threshold for the action server.
 */
class PersistentWaypointManagerState
    : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    PersistentWaypointManagerState(const std::string& action_server_name,
                                   double convergence_threshold = 0.1);

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    double convergence_threshold_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__PERSISTENT_WAYPOINT_MANAGER_STATE_HPP_
