#include "vortex_yasmin_utils/collect_bearing_direction_state.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

namespace vortex_yasmin_utils {

CollectBearingDirectionState::CollectBearingDirectionState(
    const std::string& action_server_name,
    double timeout_sec,
    double projection_distance,
    int min_measurements,
    int max_measurements,
    const std::string& pose_bb_key)
    : ActionState(
          action_server_name,
          std::bind(&CollectBearingDirectionState::create_goal,
                    this,
                    std::placeholders::_1),
          std::bind(&CollectBearingDirectionState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&CollectBearingDirectionState::feedback_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)),
      timeout_sec_(timeout_sec),
      projection_distance_(projection_distance),
      min_measurements_(min_measurements),
      max_measurements_(max_measurements),
      pose_bb_key_(pose_bb_key) {}

CollectBearingAction::Goal CollectBearingDirectionState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    CollectBearingAction::Goal goal;
    goal.timeout_sec = timeout_sec_;
    goal.distance = projection_distance_;
    goal.min_measurements = min_measurements_;
    goal.max_measurements = max_measurements_;
    YASMIN_LOG_INFO(
        "CollectBearingDirectionState: collecting for %.1fs, "
        "projecting %.1fm along bearing, min=%d max=%d",
        timeout_sec_, projection_distance_, min_measurements_,
        max_measurements_);
    return goal;
}

std::string CollectBearingDirectionState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    CollectBearingAction::Result::SharedPtr result) {
    if (!result->success) {
        YASMIN_LOG_ERROR(
            "CollectBearingDirectionState: collection failed — "
            "no valid bearings received");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    blackboard->set<geometry_msgs::msg::Pose>(pose_bb_key_, result->pose);
    YASMIN_LOG_INFO(
        "CollectBearingDirectionState: bearing waypoint at (%.3f, %.3f, %.3f)",
        result->pose.position.x, result->pose.position.y,
        result->pose.position.z);
    return yasmin_ros::basic_outcomes::SUCCEED;
}

void CollectBearingDirectionState::feedback_handler(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    std::shared_ptr<const CollectBearingAction::Feedback> feedback) {
    YASMIN_LOG_INFO(
        "CollectBearingDirectionState: %d measurements, %.1fs remaining",
        feedback->measurements_collected, feedback->time_remaining_sec);
}

}  // namespace vortex_yasmin_utils
