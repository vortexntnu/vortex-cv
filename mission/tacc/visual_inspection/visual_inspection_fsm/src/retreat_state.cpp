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

RetreatState::RetreatState(const std::string& action_server_name,
                           vortex::utils::waypoints::WaypointGoal standoff_goal,
                           std::string tcp_base_frame)
    : ActionState(
          action_server_name,
          std::bind(&RetreatState::create_goal, this, std::placeholders::_1)),
      standoff_goal_(std::move(standoff_goal)),
      tcp_base_frame_(std::move(tcp_base_frame)) {}

valve_inspection_fsm::WaypointManagerAction::Goal RetreatState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto& landmarks =
        blackboard->get<std::vector<vortex_msgs::msg::Landmark>>(
            "valve_landmarks");

    const auto valve_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        landmarks.front().pose.pose);

    const Eigen::Quaterniond q_valve = valve_pose.ori_quaternion();
    const Eigen::Vector3d z_valve = q_valve * Eigen::Vector3d::UnitZ();

    // Move standoff distance along the valve's outward normal (horizontal
    // component only), then lock z to the drone's current height so the
    // retreat is purely horizontal — no sinking.
    const Eigen::Vector3d retreat_xy =
        valve_pose.pos_vector() + z_valve * standoff_goal_.pose.z;

    // Look up current drone height from TF.
    const auto& tf_buffer =
        blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    const std::string odom_frame = landmarks.front().header.frame_id;
    geometry_msgs::msg::TransformStamped base_tf;
    try {
        base_tf = tf_buffer->lookupTransform(odom_frame, tcp_base_frame_,
                                             tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        throw std::runtime_error(
            std::string("Retreat TF lookup failed (") + odom_frame + " -> " +
            tcp_base_frame_ + "): " + ex.what());
    }

    Eigen::Vector3d retreat_pos = retreat_xy;
    retreat_pos.z() = base_tf.transform.translation.z;

    // Maintain approach orientation: +X facing valve (-Z_valve).
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
