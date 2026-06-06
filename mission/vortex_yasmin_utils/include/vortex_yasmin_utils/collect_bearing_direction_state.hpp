#ifndef VORTEX_YASMIN_UTILS__COLLECT_BEARING_DIRECTION_STATE_HPP_
#define VORTEX_YASMIN_UTILS__COLLECT_BEARING_DIRECTION_STATE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <vortex_msgs/action/collect_bearing_direction.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using CollectBearingAction = vortex_msgs::action::CollectBearingDirection;

/**
 * @brief Collects bearing measurements from a BearingDirection action server
 *        and stores the resulting pose estimate on the blackboard.
 *
 * Sends a CollectBearingDirection goal with the configured timeout and
 * projection distance. On success, stores the pose on the blackboard under
 * pose_bb_key for downstream states (e.g. BearingWaypointState).
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name  Name of the CollectBearingDirection action server
 *                            (e.g. "acoustics_bearing_direction").
 * @param timeout_sec         How long to collect bearing measurements.
 * @param projection_distance Distance along the bearing to project the waypoint.
 * @param min_measurements    Minimum data points required; returns ABORT if not
 *                            met before timeout.
 * @param max_measurements    Early-exit: return SUCCEED as soon as this many are
 *                            collected; 0 = wait for full timeout.
 * @param pose_bb_key         Blackboard key for the output pose.
 */
class CollectBearingDirectionState
    : public yasmin_ros::ActionState<CollectBearingAction> {
   public:
    CollectBearingDirectionState(
        const std::string& action_server_name,
        double timeout_sec,
        double projection_distance,
        int min_measurements = 1,
        int max_measurements = 0,
        const std::string& pose_bb_key = "bearing_waypoint_pose");

    CollectBearingAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        CollectBearingAction::Result::SharedPtr result);

    void feedback_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        std::shared_ptr<const CollectBearingAction::Feedback> feedback);

   private:
    double timeout_sec_;
    double projection_distance_;
    int min_measurements_;
    int max_measurements_;
    std::string pose_bb_key_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__COLLECT_BEARING_DIRECTION_STATE_HPP_
