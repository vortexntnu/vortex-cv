#include "visual_inspection_fsm/states.hpp"

#include <cmath>

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

TwistHandleState::TwistHandleState(const std::string& action_server_name,
                                   double convergence_threshold)
    : ActionState(
          action_server_name,
          std::bind(&TwistHandleState::create_goal,
                    this,
                    std::placeholders::_1),
          yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                           yasmin_ros::basic_outcomes::ABORT},
          std::bind(&TwistHandleState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&TwistHandleState::on_feedback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)),
      convergence_threshold_(convergence_threshold) {}

valve_inspection_fsm::GripperAction::Goal TwistHandleState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const double stored_roll = blackboard->get<double>("gripper_roll");

    // Gripper was aligned perpendicular to the handle (roll = π/2 - handle_angle).
    // Twisting by π/2 drives it to the opposite extreme:
    //   handle at 90° → gripper at 0°, twist to π/2  (positive)
    //   handle at  0° → gripper at π/2, twist to 0   (negative)
    // Both cases are unambiguous shortest-path for the action server.
    const double target_roll = M_PI / 2.0 - stored_roll;

    YASMIN_LOG_INFO("TwistHandle: stored_roll=%.4f rad → target_roll=%.4f rad",
                    stored_roll, target_roll);

    vortex_msgs::msg::GripperReferenceFilter roll_ref;
    roll_ref.roll = target_roll;

    vortex_msgs::msg::GripperWaypoint waypoint;
    waypoint.roll = roll_ref;
    waypoint.mode = vortex_msgs::msg::GripperWaypoint::ONLY_ROLL;

    valve_inspection_fsm::GripperAction::Goal goal;
    goal.waypoint = waypoint;
    goal.convergence_threshold = convergence_threshold_;

    return goal;
}

std::string TwistHandleState::result_handler(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    valve_inspection_fsm::GripperAction::Result::SharedPtr result) {
    if (!result->success)
        return yasmin_ros::basic_outcomes::ABORT;

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void TwistHandleState::on_feedback(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    std::shared_ptr<const valve_inspection_fsm::GripperAction::Feedback>
        feedback) {
    YASMIN_LOG_INFO("TwistHandle feedback — roll: %.4f  pinch: %.4f",
                    feedback->reference.roll, feedback->reference.pinch);
}
