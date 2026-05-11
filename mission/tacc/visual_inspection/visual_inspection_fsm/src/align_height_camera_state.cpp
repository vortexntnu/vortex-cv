#include "visual_inspection_fsm/states.hpp"

#include <eigen3/Eigen/Geometry>

#include <tf2/exceptions.hpp>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

AlignHeightCameraState::AlignHeightCameraState(
    const std::string& action_server_name,
    vortex::utils::waypoints::WaypointGoal standoff_goal,
    std::string tcp_base_frame,
    std::string depth_camera_frame)
    : ActionState(action_server_name,
                  std::bind(&AlignHeightCameraState::create_goal,
                            this,
                            std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)),
      tcp_base_frame_(std::move(tcp_base_frame)),
      depth_camera_frame_(std::move(depth_camera_frame)) {}

valve_inspection_fsm::WaypointManagerAction::Goal
AlignHeightCameraState::create_goal(yasmin::Blackboard::SharedPtr blackboard) {
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

    // Get the full camera transform (translation + rotation) relative to
    // base_link.
    const auto& tf_buffer =
        blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    geometry_msgs::msg::TransformStamped cam_tf;
    try {
        cam_tf = tf_buffer->lookupTransform(
            tcp_base_frame_, depth_camera_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        throw std::runtime_error(std::string("Camera TF lookup failed (") +
                                 tcp_base_frame_ + " -> " +
                                 depth_camera_frame_ + "): " + ex.what());
    }

    // Camera position relative to base_link, rotated into odom at the target
    // drone orientation.
    const Eigen::Vector3d cam_pos_in_base{cam_tf.transform.translation.x,
                                          cam_tf.transform.translation.y,
                                          cam_tf.transform.translation.z};
    const Eigen::Vector3d cam_pos_odom = q_drone * cam_pos_in_base;

    // Camera optical axis (Z) in odom: q_drone * q_cam_in_base * UnitZ.
    // This is the direction the depth camera is looking at the target pose.
    const Eigen::Quaterniond q_cam_in_base{
        cam_tf.transform.rotation.w, cam_tf.transform.rotation.x,
        cam_tf.transform.rotation.y, cam_tf.transform.rotation.z};
    const Eigen::Vector3d cam_z_odom =
        (q_drone * q_cam_in_base) * Eigen::Vector3d::UnitZ();

    // Place the camera at standoff_dist along its optical axis from the valve.
    // Equivalent to: transform landmark into camera frame and set x=0, y=0
    // (valve on optical axis) at the configured standoff depth.
    const double standoff_dist = standoff_goal_.pose.z;
    const Eigen::Vector3d cam_target =
        valve_pose.pos_vector() - cam_z_odom * standoff_dist;

    // base_link target: shift back from camera target by camera offset.
    const Eigen::Vector3d align_pos = cam_target - cam_pos_odom;

    const auto target_pose =
        vortex::utils::types::Pose::from_eigen(align_pos, q_drone);

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
