#include "vortex_yasmin_utils/gripper_state.hpp"

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

namespace vortex_yasmin_utils {

GripperState::GripperState(const std::string& action_server_name,
                           double roll,
                           double pinch,
                           uint8_t mode,
                           double convergence_threshold)
    : ActionState(
          action_server_name,
          std::bind(&GripperState::create_goal, this, std::placeholders::_1),
          yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                           yasmin_ros::basic_outcomes::ABORT},
          std::bind(&GripperState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&GripperState::on_feedback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)),
      roll_(roll),
      pinch_(pinch),
      mode_(mode),
      convergence_threshold_(convergence_threshold) {}

GripperAction::Goal GripperState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    vortex_msgs::msg::GripperReferenceFilter roll_ref;
    roll_ref.roll = roll_;

    vortex_msgs::msg::GripperReferenceFilter pinch_ref;
    pinch_ref.pinch = pinch_;

    vortex_msgs::msg::GripperWaypoint waypoint;
    waypoint.roll = roll_ref;
    waypoint.pinch = pinch_ref;
    waypoint.mode = mode_;

    GripperAction::Goal goal;
    goal.waypoint = waypoint;
    goal.convergence_threshold = convergence_threshold_;

    return goal;
}

std::string GripperState::result_handler(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    GripperAction::Result::SharedPtr result) {
    if (!result->success)
        return yasmin_ros::basic_outcomes::ABORT;

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void GripperState::on_feedback(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    std::shared_ptr<const GripperAction::Feedback> feedback) {
    YASMIN_LOG_INFO("Gripper feedback — roll: %.4f  pinch: %.4f",
                    feedback->reference.roll, feedback->reference.pinch);
}

}  // namespace vortex_yasmin_utils
