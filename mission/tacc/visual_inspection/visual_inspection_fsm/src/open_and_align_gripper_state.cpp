#include "visual_inspection_fsm/states.hpp"

#include <cmath>

#include <eigen3/Eigen/Geometry>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

OpenAndAlignGripperState::OpenAndAlignGripperState(
    const std::string& action_server_name,
    double convergence_threshold)
    : ActionState(
          action_server_name,
          std::bind(&OpenAndAlignGripperState::create_goal,
                    this,
                    std::placeholders::_1),
          yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                           yasmin_ros::basic_outcomes::ABORT},
          std::bind(&OpenAndAlignGripperState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&OpenAndAlignGripperState::on_feedback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)),
      convergence_threshold_(convergence_threshold) {}

valve_inspection_fsm::GripperAction::Goal
OpenAndAlignGripperState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "valve_landmarks");

    const auto valve_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const Eigen::Quaterniond q = valve_pose.ori_quaternion();

    // Extract yaw (rotation about world Z) from the valve quaternion.
    const double yaw = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    // The gripper jaws must be perpendicular to the handle to clamp it, so the
    // roll is offset by 90° from the handle angle. Take absolute value first
    // because perception sometimes returns the mirror angle (e.g. -π/2 vs +π/2).
    computed_roll_ = std::clamp(M_PI / 2.0 - std::abs(yaw), 0.0, M_PI / 2.0);

    YASMIN_LOG_INFO("OpenAndAlignGripper: valve yaw=%.4f rad, gripper roll → %.4f rad",
                    yaw, computed_roll_);

    vortex_msgs::msg::GripperReferenceFilter roll_ref;
    roll_ref.roll = computed_roll_;

    vortex_msgs::msg::GripperReferenceFilter pinch_ref;
    pinch_ref.pinch = -0.3333;  // fully open — ready to receive handle

    vortex_msgs::msg::GripperWaypoint waypoint;
    waypoint.roll = roll_ref;
    waypoint.pinch = pinch_ref;
    waypoint.mode = vortex_msgs::msg::GripperWaypoint::ROLL_AND_PINCH;

    valve_inspection_fsm::GripperAction::Goal goal;
    goal.waypoint = waypoint;
    goal.convergence_threshold = convergence_threshold_;

    return goal;
}

std::string OpenAndAlignGripperState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    valve_inspection_fsm::GripperAction::Result::SharedPtr result) {
    if (!result->success)
        return yasmin_ros::basic_outcomes::ABORT;

    // Store the ACTUAL achieved roll (last feedback value) so TwistHandleState
    // always applies a full π/2 twist from where the gripper really is, not
    // from the commanded target (which may not have been fully reached).
    blackboard->set<double>("gripper_roll", actual_roll_);
    YASMIN_LOG_INFO("OpenAndAlignGripper: commanded=%.4f rad, actual=%.4f rad",
                    computed_roll_, actual_roll_);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void OpenAndAlignGripperState::on_feedback(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    std::shared_ptr<const valve_inspection_fsm::GripperAction::Feedback>
        feedback) {
    actual_roll_ = feedback->reference.roll;
    YASMIN_LOG_INFO("OpenAndAlignGripper feedback — roll: %.4f  pinch: %.4f",
                    feedback->reference.roll, feedback->reference.pinch);
}
