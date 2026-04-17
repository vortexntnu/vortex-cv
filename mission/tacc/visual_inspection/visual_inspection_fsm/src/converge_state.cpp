#include "valve_inspection_fsm/states.hpp"

#include <eigen3/Eigen/Geometry>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

ConvergeState::ConvergeState(
    const std::string& action_server_name,
    vortex::utils::waypoints::WaypointGoal standoff_goal,
    vortex::utils::waypoints::WaypointGoal tcp_offset_goal)
    : ActionState(
          action_server_name,
          std::bind(&ConvergeState::create_goal, this, std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)),
      tcp_offset_goal_(std::move(tcp_offset_goal)) {}

valve_inspection_fsm::WaypointManagerAction::Goal ConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "valve_landmarks");

    const auto valve_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const Eigen::Quaterniond q_valve = valve_pose.ori_quaternion();

    // Drone orientation: +X aligned with -Z_valve (facing the valve).
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

    // Rotate the gripper-tip offset (in base_link frame) into odom frame.
    const Eigen::Vector3d tcp_local{tcp_offset_goal_.pose.x,
                                    tcp_offset_goal_.pose.y,
                                    tcp_offset_goal_.pose.z};
    const Eigen::Vector3d tcp_odom = q_drone * tcp_local;

    // base_link target: when reached, gripper tip coincides with valve handle.
    const Eigen::Vector3d base_link_target = valve_pose.pos_vector() - tcp_odom;

    const auto target_pose =
        vortex::utils::types::Pose::from_eigen(base_link_target, q_drone);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode =
        vortex::utils::waypoints::waypoint_mode_to_ros(tcp_offset_goal_.mode);

    valve_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = tcp_offset_goal_.convergence_threshold;

    return goal;
}
