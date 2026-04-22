#include "visual_inspection_fsm/states.hpp"

#include <eigen3/Eigen/Geometry>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

RetreatState::RetreatState(const std::string& action_server_name,
                           vortex::utils::waypoints::WaypointGoal standoff_goal)
    : ActionState(
          action_server_name,
          std::bind(&RetreatState::create_goal, this, std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)) {}

valve_inspection_fsm::WaypointManagerAction::Goal RetreatState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "valve_landmarks");

    const auto valve_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const Eigen::Quaterniond q_valve = valve_pose.ori_quaternion();

    // Retreat along +Z_valve (outward normal) using the same offset as
    // standoff.
    const Eigen::Vector3d retreat_offset_valve{
        standoff_goal_.pose.x, standoff_goal_.pose.y, standoff_goal_.pose.z};
    const Eigen::Vector3d retreat_pos =
        valve_pose.pos_vector() + q_valve * retreat_offset_valve;

    // Maintain approach orientation: +X facing valve (-Z_valve).
    // Build a fully constrained NED frame to avoid undetermined roll.
    // When the valve faces up (floor-mounted), approach_dir ≈ -UnitZ and the
    // primary reference becomes degenerate, so fall back to UnitX.
    const Eigen::Vector3d z_valve = q_valve * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d x_axis = (-z_valve).normalized();
    const Eigen::Vector3d ref =
        (std::abs(x_axis.dot(Eigen::Vector3d::UnitZ())) > 0.9)
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d y_axis = ref.cross(x_axis).normalized();
    const Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    const Eigen::Quaterniond q_drone(R);

    const auto target_pose =
        vortex::utils::types::Pose::from_eigen(retreat_pos, q_drone);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode =
        vortex::utils::waypoints::waypoint_mode_to_ros(standoff_goal_.mode);

    valve_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = standoff_goal_.convergence_threshold;

    return goal;
}
