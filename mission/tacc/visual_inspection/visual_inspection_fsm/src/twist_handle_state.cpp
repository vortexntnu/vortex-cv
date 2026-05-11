#include "visual_inspection_fsm/states.hpp"

#include <cmath>

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

TwistHandleState::TwistHandleState(const std::string& action_server_name,
                                   double convergence_threshold,
                                   int turn_direction)
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
      convergence_threshold_(convergence_threshold),
      turn_direction_(turn_direction) {}

valve_inspection_fsm::GripperAction::Goal TwistHandleState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const double stored_roll = blackboard->get<double>("gripper_roll");

    // Twist π/2 in the configured direction from the current gripper roll,
    // then overshoot slightly to press against the valve's mechanical stop.
    // turn_direction_: +1 = CCW (positive roll), -1 = CW (negative roll)
    //   from the drone's perspective when facing the valve.
    constexpr double kOvershoot = 0.15;  // ~8.6° past the endpoint
    const double target_roll =
        stored_roll + turn_direction_ * (M_PI / 2.0 + kOvershoot);

    YASMIN_LOG_INFO(
        "TwistHandle: stored_roll=%.4f rad, direction=%+d → target=%.4f rad",
        stored_roll, turn_direction_, target_roll);

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
