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
    std::string depth_camera_frame,
    double valve_z_offset,
    double arm_z_correction)
    : ActionState(action_server_name,
                  std::bind(&AlignHeightState::create_goal,
                            this,
                            std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)),
      tcp_offset_goal_(std::move(tcp_offset_goal)),
      tcp_base_frame_(std::move(tcp_base_frame)),
      tcp_tip_frame_(std::move(tcp_tip_frame)),
      depth_camera_frame_(std::move(depth_camera_frame)),
      valve_z_offset_(valve_z_offset),
      arm_z_correction_(arm_z_correction) {}

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

    const auto& tf_buffer =
        blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    const tf2::TimePoint obs_time =
        tf2_ros::fromMsg(landmarks.front().header.stamp);

    // Camera TF for XY: same logic as AlignHeightCameraState.
    geometry_msgs::msg::TransformStamped cam_tf;
    try {
        cam_tf = tf_buffer->lookupTransform(tcp_base_frame_,
                                            depth_camera_frame_, obs_time);
    } catch (const tf2::TransformException& ex) {
        throw std::runtime_error(std::string("Camera TF lookup failed (") +
                                 tcp_base_frame_ + " -> " +
                                 depth_camera_frame_ + "): " + ex.what());
    }

    const Eigen::Vector3d cam_pos_in_base{cam_tf.transform.translation.x,
                                          cam_tf.transform.translation.y,
                                          cam_tf.transform.translation.z};
    const Eigen::Vector3d cam_pos_odom = q_drone * cam_pos_in_base;

    const Eigen::Quaterniond q_cam_in_base{
        cam_tf.transform.rotation.w, cam_tf.transform.rotation.x,
        cam_tf.transform.rotation.y, cam_tf.transform.rotation.z};
    const Eigen::Vector3d cam_z_odom =
        (q_drone * q_cam_in_base) * Eigen::Vector3d::UnitZ();

    const double standoff_dist = standoff_goal_.pose.z;
    const Eigen::Vector3d cam_target =
        valve_pose.pos_vector() - cam_z_odom * standoff_dist;
    const Eigen::Vector3d align_pos = cam_target - cam_pos_odom;

    // TCP TF for Z: compute the full converge target (identical to
    // ConvergeState) and take its z, so the AlignHeight→Converge transition is
    // purely horizontal.
    geometry_msgs::msg::TransformStamped tcp_tf;
    try {
        tcp_tf = tf_buffer->lookupTransform(tcp_base_frame_, tcp_tip_frame_,
                                            obs_time);
    } catch (const tf2::TransformException& ex) {
        throw std::runtime_error(std::string("TCP TF lookup failed (") +
                                 tcp_base_frame_ + " -> " + tcp_tip_frame_ +
                                 "): " + ex.what());
    }

    const auto& t = tcp_tf.transform.translation;
    const Eigen::Vector3d tcp_odom = q_drone * Eigen::Vector3d{t.x, t.y, t.z};

    // Project valve_z_offset along the valve normal (same as ConvergeState) so
    // the z target is correct for any valve orientation, not just
    // upward-facing.
    const Eigen::Vector3d converge_pos =
        valve_pose.pos_vector() + z_valve * valve_z_offset_ - tcp_odom;

    // XY from camera standoff; Z from the full converge target plus tunable
    // correction for any TCP TF height error.
    Eigen::Vector3d target_pos = align_pos;
    target_pos.z() = converge_pos.z() + arm_z_correction_;

    const auto target_pose =
        vortex::utils::types::Pose::from_eigen(target_pos, q_drone);

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
