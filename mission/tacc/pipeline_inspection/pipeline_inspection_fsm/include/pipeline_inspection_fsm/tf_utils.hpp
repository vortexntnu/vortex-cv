#ifndef PIPELINE_INSPECTION_FSM__TF_UTILS_HPP_
#define PIPELINE_INSPECTION_FSM__TF_UTILS_HPP_

#include <optional>
#include <string>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace pipeline_inspection_fsm::tf_utils {

tf2_ros::Buffer& get_tf_buffer();

std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
    const geometry_msgs::msg::PoseStamped& pose_in,
    const std::string& target_frame,
    tf2::Duration timeout = tf2::durationFromSec(1.0));

}  // namespace pipeline_inspection_fsm::tf_utils

#endif  // PIPELINE_INSPECTION_FSM__TF_UTILS_HPP_
