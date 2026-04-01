#include "valve_inspection_fsm/valve_converge_state.hpp"

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <yasmin/logs.hpp>
#include <yasmin_ros/yasmin_node.hpp>

namespace valve_inspection_fsm {

ValveConvergeState::ValveConvergeState(
    const std::string& action_server_name,
    vortex_msgs::msg::LandmarkType type,
    vortex_msgs::msg::LandmarkSubtype subtype,
    double convergence_threshold,
    double dead_reckoning_threshold,
    double track_loss_timeout_sec,
    const std::string& gripper_frame,
    const std::string& base_frame)
    : ActionState(action_server_name,
                  std::bind(&ValveConvergeState::create_goal,
                            this,
                            std::placeholders::_1)),
      type_(type),
      subtype_(subtype),
      convergence_threshold_(convergence_threshold),
      dead_reckoning_threshold_(dead_reckoning_threshold),
      track_loss_timeout_sec_(track_loss_timeout_sec),
      gripper_frame_(gripper_frame),
      base_frame_(base_frame) {
    auto node = yasmin_ros::YasminNode::get_instance();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LandmarkConvergenceAction::Goal ValveConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    geometry_msgs::msg::Pose convergence_offset;
    convergence_offset.orientation.w = 1.0;

    try {
        auto tf = tf_buffer_->lookupTransform(base_frame_, gripper_frame_,
                                              tf2::TimePointZero,
                                              tf2::durationFromSec(1.0));

        convergence_offset.position.x = -tf.transform.translation.x;
        convergence_offset.position.y = -tf.transform.translation.y;
        convergence_offset.position.z = -tf.transform.translation.z;

        YASMIN_LOG_INFO(
            "ValveConvergeState: TF %s -> %s [%.3f, %.3f, %.3f] "
            "=> offset [%.3f, %.3f, %.3f]",
            base_frame_.c_str(), gripper_frame_.c_str(),
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z, convergence_offset.position.x,
            convergence_offset.position.y, convergence_offset.position.z);
    } catch (const tf2::TransformException& e) {
        YASMIN_LOG_WARN(
            "ValveConvergeState: TF lookup failed (%s -> %s): %s. "
            "Converging with zero offset.",
            base_frame_.c_str(), gripper_frame_.c_str(), e.what());
    }

    LandmarkConvergenceAction::Goal goal;
    goal.type = type_;
    goal.subtype = subtype_;
    goal.convergence_offset = convergence_offset;
    goal.convergence_threshold = convergence_threshold_;
    goal.dead_reckoning_threshold = dead_reckoning_threshold_;
    goal.track_loss_timeout_sec = track_loss_timeout_sec_;
    goal.convergence_mode.mode = vortex_msgs::msg::WaypointMode::FULL_POSE;

    return goal;
}

}  // namespace valve_inspection_fsm
