// valve_subtype_resolver.cpp
// Subscribes to valve landmarks (subtype unset) published by valve_detection.
// For each landmark, looks up the full transform from the landmark frame to the
// world frame via TF2, rotates the valve plane normal into the world frame, and
// classifies the valve as VALVE_VERTICAL or VALVE_HORIZONTAL based on how much
// the normal deviates from the world Z-axis.  Republishes with subtype set.
#include "valve_subtype_resolver/valve_subtype_resolver.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>

#include <cmath>

namespace valve_subtype_resolver {

using std::placeholders::_1;

ValveSubtypeResolverNode::ValveSubtypeResolverNode(
    const rclcpp::NodeOptions& options)
    : Node("valve_subtype_resolver_node", options) {
    const std::string drone = declare_parameter<std::string>("drone", "moby");
    const std::string odom_frame_base =
        declare_parameter<std::string>("odom_frame", "odom");
    world_frame_ = drone + "/" + odom_frame_base;
    vertical_threshold_ = declare_parameter<double>("vertical_threshold", 0.5);
    clamp_yaw_ = declare_parameter<bool>("clamp_yaw", false);

    const auto landmarks_in = declare_parameter<std::string>(
        "landmarks_sub_topic", "/valve_landmarks");
    const auto landmarks_out = declare_parameter<std::string>(
        "landmarks_pub_topic", "/valve_landmarks_typed");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    landmark_sub_ = create_subscription<vortex_msgs::msg::LandmarkArray>(
        landmarks_in, qos,
        std::bind(&ValveSubtypeResolverNode::landmarks_cb, this, _1));

    landmark_pub_ =
        create_publisher<vortex_msgs::msg::LandmarkArray>(landmarks_out, qos);
}

void ValveSubtypeResolverNode::landmarks_cb(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
    vortex_msgs::msg::LandmarkArray out = *msg;
    for (auto& lm : out.landmarks) {
        lm.subtype.value = static_cast<uint16_t>(resolve_subtype(lm));
        if (clamp_yaw_)
            fold_yaw_world(lm);
    }
    landmark_pub_->publish(out);
}

// Folds the landmark's yaw into [0, π/2] in the world frame.  The valve
// handle is line-symmetric (180°) and additionally reflected at 90°, so
// any physically-equivalent orientation maps to the same yaw value.
//
// Procedure:
//   1. Transform the landmark pose from its frame into world_frame_.
//   2. Extract yaw about world Z via atan2.
//   3. Fold to [0, π/2].
//   4. Pre-rotate the world-frame pose about world Z by Δ = folded − yaw
//      so the new world-frame yaw equals `folded`.  Pitch/roll are
//      preserved.
//   5. Transform the adjusted pose back into the original frame and
//      replace the orientation.  The landmark's frame_id is unchanged,
//      so downstream TF chains keep working.
//
// Because the fold is computed in the world frame, the output yaw is
// invariant to drone roll/pitch — rolling the camera around the valve
// does not change which side of the fold the value lands on.
void ValveSubtypeResolverNode::fold_yaw_world(vortex_msgs::msg::Landmark& lm) {
    geometry_msgs::msg::TransformStamped tf_to_world, tf_from_world;
    try {
        tf_to_world = tf_buffer_->lookupTransform(
            world_frame_, lm.header.frame_id, tf2::TimePointZero);
        tf_from_world = tf_buffer_->lookupTransform(
            lm.header.frame_id, world_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "TF lookup failed during yaw fold: %s", ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped ps_in, ps_world;
    ps_in.header = lm.header;
    ps_in.pose = lm.pose.pose;
    tf2::doTransform(ps_in, ps_world, tf_to_world);

    const auto& qo = ps_world.pose.orientation;
    tf2::Quaternion q_world(qo.x, qo.y, qo.z, qo.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_world).getRPY(roll, pitch, yaw);

    double folded = std::fmod(std::abs(yaw), M_PI);
    if (folded > M_PI / 2.0)
        folded = M_PI - folded;

    tf2::Quaternion q_delta;
    q_delta.setRPY(0.0, 0.0, folded - yaw);
    tf2::Quaternion q_world_new = q_delta * q_world;
    q_world_new.normalize();

    ps_world.pose.orientation.x = q_world_new.x();
    ps_world.pose.orientation.y = q_world_new.y();
    ps_world.pose.orientation.z = q_world_new.z();
    ps_world.pose.orientation.w = q_world_new.w();

    geometry_msgs::msg::PoseStamped ps_back;
    tf2::doTransform(ps_world, ps_back, tf_from_world);
    lm.pose.pose.orientation = ps_back.pose.orientation;
}

// Resolves the valve subtype by rotating the valve plane normal into the world
// frame and checking its Z component.
//
// The valve plane normal is the Z-axis of the landmark orientation quaternion.
// It is transformed from the landmark frame into the world frame via a TF2
// lookup that traverses the full chain:
//   <drone>/front_camera_depth_optical → <drone>/front_camera_depth
//   → <drone>/front_camera_link → <drone>/base_link → <drone>/odom
//
// Classification:
//   |world_nz| >= vertical_threshold → VALVE_VERTICAL (1)
//     The normal points mostly up or down; the valve face is horizontal,
//     meaning the valve wheel spins around a vertical axis.
//   |world_nz| < vertical_threshold → VALVE_HORIZONTAL (2)
//     The normal is mostly in the XY plane; the valve face is vertical,
//     meaning the valve wheel spins around a horizontal axis.
//
// Returns 0 (unknown) if the TF lookup fails.
int ValveSubtypeResolverNode::resolve_subtype(
    const vortex_msgs::msg::Landmark& landmark) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            world_frame_, landmark.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "TF lookup failed: %s", ex.what());
        return 0;
    }

    // Extract the valve plane normal (Z-axis of the landmark orientation).
    // Rotating [0,0,1] by quaternion q gives:
    //   nx = 2(qx*qz + qw*qy)
    //   ny = 2(qy*qz - qw*qx)
    //   nz = 1 - 2(qx^2 + qy^2)
    const auto& lq = landmark.pose.pose.orientation;
    geometry_msgs::msg::Vector3Stamped normal_cam;
    normal_cam.header = landmark.header;
    normal_cam.vector.x = 2.0 * (lq.x * lq.z + lq.w * lq.y);
    normal_cam.vector.y = 2.0 * (lq.y * lq.z - lq.w * lq.x);
    normal_cam.vector.z = 1.0 - 2.0 * (lq.x * lq.x + lq.y * lq.y);

    // Rotate the normal into the world frame.
    // tf2::doTransform on a Vector3Stamped applies only the rotation.
    geometry_msgs::msg::Vector3Stamped normal_world;
    tf2::doTransform(normal_cam, normal_world, transform);

    using ST = vortex_msgs::msg::LandmarkSubtype;
    if (std::abs(normal_world.vector.z) >= vertical_threshold_)
        return ST::VALVE_VERTICAL;
    return ST::VALVE_HORIZONTAL;
}

}  // namespace valve_subtype_resolver

RCLCPP_COMPONENTS_REGISTER_NODE(
    valve_subtype_resolver::ValveSubtypeResolverNode)
