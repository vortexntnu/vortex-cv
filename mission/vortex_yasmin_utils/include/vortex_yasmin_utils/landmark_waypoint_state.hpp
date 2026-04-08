#ifndef VORTEX_YASMIN_UTILS__LANDMARK_WAYPOINT_STATE_HPP_
#define VORTEX_YASMIN_UTILS__LANDMARK_WAYPOINT_STATE_HPP_

#include <string>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Sends a waypoint to the WaypointManager based on a BB landmark pose +
 * offset.
 *
 * Reads the landmark vector posted by LandmarkPollingState from the blackboard,
 * applies the configured pose offset to the first landmark's position, and
 * sends the resulting waypoint to the WaypointManager action server.
 *
 * The offset is expressed in the landmark frame and is loaded from a YAML file
 * via load_waypoint_goal_from_yaml() before constructing this state.
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name  Name of the WaypointManager action server.
 * @param waypoint_goal       Offset pose, waypoint mode, and convergence
 *                            threshold (loaded from YAML).
 * @param landmarks_bb_key    Blackboard key where LandmarkPollingState stores
 *                            the landmark vector (default: "landmarks").
 */
class LandmarkWaypointState
    : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    LandmarkWaypointState(const std::string& action_server_name,
                          vortex::utils::waypoints::WaypointGoal waypoint_goal,
                          const std::string& landmarks_bb_key = "landmarks");

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal waypoint_goal_;
    std::string landmarks_bb_key_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__LANDMARK_WAYPOINT_STATE_HPP_
