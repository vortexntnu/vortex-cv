#include "visual_inspection_fsm/states.hpp"

#include <eigen3/Eigen/Geometry>

#if __has_include(<tf2/exceptions.hpp>)
#include <tf2/exceptions.hpp>
#else
#include <tf2/exceptions.h>
#endif

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

AlignHeightState::AlignHeightState(
    const std::string& action_server_name,
    vortex::utils::waypoints::WaypointGoal standoff_goal,
    vortex::utils::waypoints::WaypointGoal tcp_offset_goal,
    std::string tcp_base_frame,
    std::string tcp_tip_frame,
    double valve_z_offset)
    : ActionState(
          action_server_name,
          std::bind(&AlignHeightState::create_goal, this,
                    std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)),
      tcp_offset_goal_(std::move(tcp_offset_goal)),
      tcp_base_frame_(std::move(tcp_base_frame)),
      tcp_tip_frame_(std::move(tcp_tip_frame)),
      valve_z_offset_(valve_z_offset) {}

valve_inspection_fsm::WaypointManagerAction::Goal AlignHeightState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "valve_landmarks");

    const auto valve_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const Eigen::Quaterniond q_valve = valve_pose.ori_quaternion();

    // Drone orientation: +X aligned with -Z_valve (facing the valve).
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

    // Standoff position: valve_pos + valve-frame offset rotated into odom.
    const Eigen::Vector3d standoff_offset_valve{
        standoff_goal_.pose.x, standoff_goal_.pose.y, standoff_goal_.pose.z};
    const Eigen::Vector3d standoff_pos =
        valve_pose.pos_vector() + q_valve * standoff_offset_valve;

    // TCP offset from TF.
    const auto& tf_buffer =
        blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer->lookupTransform(
            tcp_base_frame_, tcp_tip_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        throw std::runtime_error(
            std::string("TCP TF lookup failed (") + tcp_base_frame_ + " → " +
            tcp_tip_frame_ + "): " + ex.what());
    }
    const auto& t = tf_stamped.transform.translation;
    const Eigen::Vector3d tcp_odom = q_drone * Eigen::Vector3d{t.x, t.y, t.z};

    // Apply offset along valve outward Z so TCP stops in front of valve face.
    const Eigen::Vector3d valve_target =
        valve_pose.pos_vector() + z_valve * valve_z_offset_;

    // Final converge target (where base_link needs to be for TCP to reach target).
    const Eigen::Vector3d converge_pos = valve_target - tcp_odom;

    // Intermediate: keep standoff X/Y, use converge Z so we only correct height.
    const Eigen::Vector3d align_pos{standoff_pos.x(), standoff_pos.y(),
                                    converge_pos.z()};

    const auto target_pose =
        vortex::utils::types::Pose::from_eigen(align_pos, q_drone);

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
