#include "pipeline_inspection_fsm/tf_utils.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include <vortex/utils/ros/ros_transforms.hpp>

namespace pipeline_inspection_fsm::tf_utils {

tf2_ros::Buffer& get_tf_buffer() {
    auto node = yasmin_ros::YasminNode::get_instance();

    static auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    static auto listener =
        std::make_shared<tf2_ros::TransformListener>(*buffer);

    return *buffer;
}

std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
    const geometry_msgs::msg::PoseStamped& pose_in,
    const std::string& target_frame,
    tf2::Duration timeout) {
    geometry_msgs::msg::PoseStamped pose_out;
    try {
        vortex::utils::ros_transforms::transform_pose(
            get_tf_buffer(), pose_in, target_frame, pose_out, timeout);
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
    return pose_out;
}

}  // namespace pipeline_inspection_fsm::tf_utils
