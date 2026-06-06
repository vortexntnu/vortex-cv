#ifndef VORTEX_YASMIN_UTILS__BEARING_WAYPOINT_STATE_HPP_
#define VORTEX_YASMIN_UTILS__BEARING_WAYPOINT_STATE_HPP_

#include <string>

#include <vortex_msgs/action/waypoint_manager.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/**
 * @brief Navigates to a pose stored in the blackboard by a bearing collection step.
 *
 * Reads a geometry_msgs::msg::Pose from the blackboard under pose_bb_key
 * (stored by CollectBearingDirectionState) and sends it to the WaypointManager
 * action server as a one-shot POSITION_AND_YAW waypoint.
 *
 * The yaw is taken from the pose orientation (heading toward the bearing target).
 * If desired_altitude >= 0, the waypoint z is overridden with that absolute value.
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name    Name of the WaypointManager action server.
 * @param convergence_threshold Convergence distance threshold in metres.
 * @param desired_altitude      Absolute z to use for the waypoint. Negative = use bearing server z.
 * @param pose_bb_key           Blackboard key to read the pose from.
 */
class BearingWaypointState
    : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    explicit BearingWaypointState(
        const std::string& action_server_name,
        double convergence_threshold = 0.5,
        double desired_altitude = 1.0,
        const std::string& pose_bb_key = "bearing_waypoint_pose");

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    double convergence_threshold_;
    double desired_altitude_;
    std::string pose_bb_key_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__BEARING_WAYPOINT_STATE_HPP_
