#include "visual_inspection_fsm/states.hpp"

#include <cmath>

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

TwistHandleState::TwistHandleState(const std::string& action_server_name,
                                   double convergence_threshold)
    : ActionState(action_server_name,
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

    // Gripper was aligned perpendicular to the handle (roll = π/2 -
    // handle_angle). Twist to the opposite extreme, then overshoot slightly so
    // the gripper presses against the valve's mechanical stop rather than
    // stopping just short.
    //   handle at 90° → stored≈0, target > π/2  (positive overshoot)
    //   handle at  0° → stored≈π/2, target < 0  (negative overshoot)
    constexpr double kOvershoot = 0.15;  // ~8.6° past the endpoint
    const double base_target = M_PI / 2.0 - stored_roll;
    const double overshoot =
        (stored_roll < M_PI / 4.0) ? kOvershoot : -kOvershoot;
    const double target_roll = base_target + overshoot;

    YASMIN_LOG_INFO(
        "TwistHandle: stored_roll=%.4f rad → base=%.4f rad, target=%.4f rad",
        stored_roll, base_target, target_roll);

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
